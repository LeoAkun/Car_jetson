#!/usr/bin/python3
"""Offline read-only SQLite/CDR audit; never initializes ROS or plays a bag."""
import collections, datetime as dt, json, math, pathlib, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=pathlib.Path(__file__).resolve().parent
TZ=dt.timezone(dt.timedelta(hours=8))
def local(t): return dt.datetime.fromtimestamp(float(t),TZ).isoformat(timespec='microseconds')
def stamp(s): return s.sec+s.nanosec*1e-9
def vec(v): return [v.x,v.y,v.z]
def quat(q): return [q.x,q.y,q.z,q.w]
def desc(a):
    a=np.asarray(a); a=a[np.isfinite(a)]
    if not len(a): return None
    return {'min':float(a.min()),'p01':float(np.quantile(a,.01)),'median':float(np.median(a)),'p99':float(np.quantile(a,.99)),'max':float(a.max()),'mean':float(a.mean())}
def load(run):
    data=collections.defaultdict(list); logs=[]; counts=collections.Counter()
    names={'/imu','/lio_sam/mapping/odometry_incremental','/lio_sam/mapping/odometry','/odom_incremental','/odom','/rosout'}
    for path in sorted((run/'navigation_core_bag').glob('*.db3'),key=lambda p:int(p.stem.rsplit('_',1)[1])):
        c=sqlite3.connect('file:'+str(path)+'?mode=ro&immutable=1',uri=True)
        topics={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics') if n in names}
        ids=','.join(str(i) for i in topics)
        for ns,i,raw in c.execute('select timestamp,topic_id,data from messages where topic_id in ('+ids+') order by timestamp'):
            n,typ=topics[i];m=deserialize_message(raw,typ);t=ns*1e-9;counts[n]+=1
            if n=='/imu':
                data[n].append([t,stamp(m.header.stamp),*vec(m.linear_acceleration),*vec(m.angular_velocity),*quat(m.orientation),*m.linear_acceleration_covariance,*m.angular_velocity_covariance,*m.orientation_covariance])
            elif n=='/rosout':
                logs.append({'record_time':local(t),'record_epoch':t,'stamp':stamp(m.stamp),'node':m.name,'level':m.level,'message':m.msg})
            else:
                data[n].append([t,stamp(m.header.stamp),*vec(m.pose.pose.position),*quat(m.pose.pose.orientation),*vec(m.twist.twist.linear),*vec(m.twist.twist.angular),*m.pose.covariance,*m.twist.covariance])
        c.close()
        print(run.name,path.name,'read',flush=True)
    for n in data: data[n]=np.asarray(data[n])
    return data,logs,counts
def imu_stats(a):
    if not len(a): return {'count':0}
    ds=np.diff(a[:,1]);dr=np.diff(a[:,0]); bad=np.flatnonzero(ds<=0)
    gaps=np.argsort(ds)[-8:][::-1]
    qn=np.linalg.norm(a[:,8:12],axis=1);acc=np.linalg.norm(a[:,2:5],axis=1);gyr=np.linalg.norm(a[:,5:8],axis=1)
    return {'count':len(a),'first_record':local(a[0,0]),'last_record':local(a[-1,0]),'record_hz':float((len(a)-1)/(a[-1,0]-a[0,0])),'header_hz':float((len(a)-1)/(a[-1,1]-a[0,1])),
      'header_dt_s':desc(ds),'record_dt_s':desc(dr),'latency_s':desc(a[:,0]-a[:,1]),'duplicate_header_count':int(np.sum(ds==0)),'negative_header_dt_count':int(np.sum(ds<0)),
      'header_gap_gt_30ms_count':int(np.sum(ds>.03)),'header_gap_gt_100ms_count':int(np.sum(ds>.1)),
      'nonfinite_value_count':int(np.sum(~np.isfinite(a[:,1:]))),'accel_mps2':{'norm':desc(acc),'axis_min':a[:,2:5].min(axis=0).tolist(),'axis_max':a[:,2:5].max(axis=0).tolist()},
      'angular_radps':{'norm':desc(gyr),'axis_min':a[:,5:8].min(axis=0).tolist(),'axis_max':a[:,5:8].max(axis=0).tolist()},'quaternion_norm':desc(qn),
      'quaternion_outside_0p9_1p1_count':int(np.sum((qn<.9)|(qn>1.1))),
      'quaternion_outside_0p9_1p1_samples':[{'record_time':local(a[i,0]),'stamp':float(a[i,1]),'quaternion':a[i,8:12].tolist(),'norm':float(qn[i])} for i in np.flatnonzero((qn<.9)|(qn>1.1))[:30]],
      'covariance_unique_rows':np.unique(a[:,12:],axis=0).tolist(),
      'largest_header_gaps':[{'after_record':local(a[i,0]),'next_record':local(a[i+1,0]),'header_dt_s':float(ds[i]),'record_dt_s':float(dr[i])} for i in gaps],
      'nonincreasing_samples':[{'first_record':local(a[i,0]),'second_record':local(a[i+1,0]),'header_dt':float(ds[i])} for i in bad[:30]]}
def row_summary(row):
    return {'record_time':local(row[0]),'record_epoch':float(row[0]),'stamp':float(row[1]),'stamp_time':local(row[1]),'position':row[2:5].tolist(),'quaternion':row[5:9].tolist(),'quat_norm':float(np.linalg.norm(row[5:9])),'pose_covariance0':float(row[15]),'all_finite':bool(np.all(np.isfinite(row)))}
def simulate(data,arrival_start):
    # Recording arrival order approximates node callback order but cannot prove it.
    imu=data['/imu'];odom=data['/lio_sam/mapping/odometry_incremental']
    events=[(r[0],0,r) for r in imu if r[0]>=arrival_start]+[(r[0],1,r) for r in odom]
    events.sort(key=lambda r:r[0]);queue=collections.deque();last=-1;initialized=False;out=[]
    for t,typ,r in events:
        if typ==0: queue.append(r);continue
        current=r[1];entry=row_summary(r);entry['queue_len_before']=len(queue);entry['queue_front_stamp']=float(queue[0][1]) if queue else None
        if not queue: entry['operation']='would_return_empty';out.append(entry);continue
        if not initialized:
            n=0
            while queue and queue[0][1]<current:
                last=queue.popleft()[1];n+=1
            initialized=True;entry.update(operation='initialize_prior',discarded_old_imu=n,last_imu_stamp=float(last),queue_len_after=len(queue));out.append(entry);continue
        dts=[];times=[]
        while queue and queue[0][1]<current:
            im=queue.popleft();dts.append(.002 if last<0 else im[1]-last);last=im[1];times.append(float(last))
        entry.update(operation='integrate_and_optimize',integrated_imu_count=len(dts),deltaTij_s=float(sum(dts)),imu_dt_stats=desc(dts),first_integrated_stamp=times[0] if times else None,last_integrated_stamp=times[-1] if times else None,queue_len_after=len(queue),bias_between_sigma=[float(math.sqrt(sum(dts))*.0002),float(math.sqrt(sum(dts))*.00001)] if sum(dts)>=0 else None)
        out.append(entry)
    return out

all_results={}
for run in [ROOT,ROOT.parent/'2026-11-09_14-33-18']:
    data,logs,counts=load(run);imu=data['/imu']; result={'counts':dict(counts),'imu_all':imu_stats(imu)}
    if run==ROOT:
        start=1794210539.620768504
        result['imu_startup_window']=imu_stats(imu[(imu[:,0]>=start-2)&(imu[:,0]<=start+3)])
        result['imu_after_crash']=imu_stats(imu[imu[:,0]>=1794210542])
        corrections=data['/lio_sam/mapping/odometry_incremental']
        result['first_mapping_corrections']=[row_summary(r) for r in corrections[:20]]
        result['mapping_correction_quality']={'count':len(corrections),'nonfinite_value_count':int(np.sum(~np.isfinite(corrections))),'nonincreasing_header_count':int(np.sum(np.diff(corrections[:,1])<=0)),'header_dt_s':desc(np.diff(corrections[:,1])),'quat_norm':desc(np.linalg.norm(corrections[:,5:9],axis=1))}
        result['known_imu_preintegration_exit_epoch']=1794210541.0228119
        result['known_imu_preintegration_exit_time']=local(1794210541.0228119)
        result['interpretation_limits']=['Recording arrival order approximates process callback ordering but cannot prove it.','Simulation after the recorded process exit is hypothetical only; the preintegrator was already dead.','The two invalid orientation samples at driver startup occurred approximately 93 s before the LIO-SAM process started and do not prove the later crash cause.','All covariance fields are zero (unknown); LIO-SAM preintegrator uses configured noise instead of these sensor covariance arrays.']
        result['simulation_by_record_arrival']=simulate(data,start)[:15]
        result['simulation_from_earliest_start']=simulate(data,1794210539.52)[:15]
        result['relevant_rosout']=[x for x in logs if ('imu' in x['message'].lower() or 'imu' in x['node'].lower() or 1794210539<=x['record_epoch']<=1794210542)]
        np.savez_compressed(ROOT/'imu_crash_arrays.npz',**{n.strip('/').replace('/','_'):a for n,a in data.items()})
        result['imu_startup_samples']=[{'record':local(r[0]),'record_epoch':float(r[0]),'stamp':float(r[1]),'stamp_time':local(r[1]),'acc':r[2:5].tolist(),'gyr':r[5:8].tolist(),'q':r[8:12].tolist()} for r in imu[(imu[:,0]>=1794210539.60)&(imu[:,0]<=1794210541.1)]]
    else:
        result['startup_coverage_note']='Normal-run bag begins mid-navigation at 14:38:44; it does not contain the LIO-SAM initialization window around 14:33:39.'
        result['imu_first_5s']=imu_stats(imu[imu[:,0]<imu[0,0]+5])
    all_results[run.name]=result
(ROOT/'imu_crash_data_audit.json').write_text(json.dumps(all_results,ensure_ascii=False,indent=2))
print(json.dumps(all_results[ROOT.name]['simulation_by_record_arrival'][:6],ensure_ascii=False,indent=2),flush=True)
