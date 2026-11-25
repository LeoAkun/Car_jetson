#!/usr/bin/python3
"""Read-only SQLite/CDR analysis. Never initializes ROS or opens control channels."""
import collections, datetime as dt, glob, json, math, pathlib, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = pathlib.Path(__file__).resolve().parent
TZ = dt.timezone(dt.timedelta(hours=8))
def local(t): return dt.datetime.fromtimestamp(float(t), TZ).isoformat(timespec='milliseconds')
def stamp(s): return s.sec + s.nanosec*1e-9
def yaw(q): return math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
def quat(q): return [q.x,q.y,q.z,q.w]
def qmul(a,b):
    ax,ay,az,aw=a;bx,by,bz,bw=b
    return np.array([aw*bx+ax*bw+ay*bz-az*by,aw*by-ax*bz+ay*bw+az*bx,aw*bz+ax*by-ay*bx+az*bw,aw*bw-ax*bx-ay*by-az*bz])
def rot(q,v): return qmul(qmul(q,[*v,0]),[-q[0],-q[1],-q[2],q[3]])[:3]
def signed(points,a,b):
    d=np.asarray(b)-a; d=d/np.linalg.norm(d)
    delta=np.asarray(points)-a
    return d[0]*delta[...,1]-d[1]*delta[...,0]
def nearest(a, t):
    idx=np.searchsorted(a[:,0],t);idx=np.clip(idx,0,len(a)-1)
    prev=np.maximum(idx-1,0)
    return np.where(np.abs(a[prev,0]-t)<np.abs(a[idx,0]-t),prev,idx)

wanted={'/tf','/tf_static','/odom','/navigate_to_pose/_action/feedback','/plan','/vehicle_status','/cmd_vel','/cmd_vel_nav','/ecu','/lio_sam/mapping/odometry'}
data=collections.defaultdict(list); counts=collections.Counter(); tf=collections.defaultdict(list); plans=[]; first=1e20; last=0
for path in sorted((ROOT/'navigation_core_bag').glob('*.db3'),key=lambda p:int(p.stem.rsplit('_',1)[1])):
    con=sqlite3.connect('file:'+str(path)+'?mode=ro&immutable=1',uri=True)
    start,end=con.execute('select min(timestamp),max(timestamp) from messages').fetchone();first=min(first,start*1e-9);last=max(last,end*1e-9)
    topics={i:(name,get_message(typ)) for i,name,typ in con.execute('select id,name,type from topics') if name in wanted}
    ids=','.join(str(i) for i in topics)
    for timestamp,topicid,raw in con.execute('select timestamp,topic_id,data from messages where topic_id in ('+ids+') order by timestamp'):
        name,typ=topics[topicid]; m=deserialize_message(raw,typ);t=timestamp*1e-9;counts[name]+=1
        if name in ('/tf','/tf_static'):
            for tr in m.transforms:
                p=tr.transform.translation;q=quat(tr.transform.rotation)
                tf[tr.header.frame_id+'->'+tr.child_frame_id].append([t,stamp(tr.header.stamp),p.x,p.y,p.z,*q])
        elif name=='/navigate_to_pose/_action/feedback':
            fb=m.feedback;p=fb.current_pose.pose.position;q=quat(fb.current_pose.pose.orientation)
            data['feedback'].append([t,stamp(fb.current_pose.header.stamp),p.x,p.y,p.z,yaw(q),fb.distance_remaining,fb.number_of_recoveries])
        elif name in ('/odom','/lio_sam/mapping/odometry'):
            p=m.pose.pose.position;q=quat(m.pose.pose.orientation);tw=m.twist.twist
            data['odom' if name=='/odom' else 'mapping'].append([t,stamp(m.header.stamp),p.x,p.y,p.z,*q,tw.linear.x,tw.linear.y,tw.angular.z])
        elif name=='/vehicle_status':
            data['vehicle'].append([t,stamp(m.header.stamp),m.cur_speed,m.cur_steer,m.drive_mode,int(m.is_autodrive),m.shift_level,int(m.p_status)])
        elif name in ('/cmd_vel','/cmd_vel_nav'):
            data['cmd' if name=='/cmd_vel' else 'cmd_nav'].append([t,m.linear.x,m.angular.z])
        elif name=='/ecu': data['ecu'].append([t,m.motor,m.steer,int(m.brake),m.shift])
        elif name=='/plan':
            points=np.array([[p.pose.position.x,p.pose.position.y] for p in m.poses])
            plans.append({'t':t,'header':stamp(m.header.stamp),'frame':m.header.frame_id,'points':points.tolist()})
    con.close()
    print(path.name,'done',flush=True)
for key in data: data[key]=np.asarray(data[key])
for key in tf: tf[key]=np.asarray(tf[key])
np.savez_compressed(ROOT/'trajectory_arrays.npz',**data,**{'tf_'+k.replace('->','__'):v for k,v in tf.items()})
(ROOT/'trajectory_plans.json').write_text(json.dumps(plans,separators=(',',':')))
summary={'coverage':{'start':local(first),'end':local(last),'seconds':last-first},'counts':dict(counts),'tf_edges':{k:len(v) for k,v in tf.items()}}

veh=data['vehicle']; changes=np.r_[True,np.any(veh[1:,4:6]!=veh[:-1,4:6],axis=1)]
summary['drive_mode_changes']=[{'time':local(r[0]),'speed_kmh':r[2],'drive_mode':int(r[4]),'is_autodrive':bool(r[5])} for r in veh[changes]]
manual=veh[(veh[:,5]==0)&(veh[:,0]>1794206343)][0,0]
summary['first_manual_time']=local(manual);summary['first_manual_epoch']=manual
waypoints={113:np.array([109.31,-46.35]),106:np.array([115.12,-143.71]),107:np.array([122.66,-249.5])}
fb=data['feedback'];seg107start=1794206343.0
summary['segment_stats']={}
for name,mask,a,b in [('106_end',fb[:,0]<seg107start,waypoints[113],waypoints[106]),('107_auto',((fb[:,0]>=seg107start)&(fb[:,0]<manual)),waypoints[106],waypoints[107]),('107_manual',fb[:,0]>=manual,waypoints[106],waypoints[107])]:
    rows=fb[mask];c=signed(rows[:,2:4],a,b); il=np.argmax(c);ir=np.argmin(c)
    summary['segment_stats'][name]={'count':len(rows),'first':{'time':local(rows[0,0]),'x':rows[0,2],'y':rows[0,3],'yaw_deg':math.degrees(rows[0,5]),'cross_m':c[0]},'last':{'time':local(rows[-1,0]),'x':rows[-1,2],'y':rows[-1,3],'yaw_deg':math.degrees(rows[-1,5]),'cross_m':c[-1]},'max_left':{'time':local(rows[il,0]),'x':rows[il,2],'y':rows[il,3],'cross_m':c[il]},'max_right':{'time':local(rows[ir,0]),'x':rows[ir,2],'y':rows[ir,3],'cross_m':c[ir]}}

summary['plan_stats']=[]
for p in plans:
    pts=np.asarray(p['points']);goal=min(waypoints,key=lambda k:np.linalg.norm(pts[-1]-waypoints[k]));a=waypoints[113] if goal==106 else waypoints[106];b=waypoints[goal];cross=signed(pts,a,b)
    summary['plan_stats'].append({'time':local(p['t']),'epoch':p['t'],'goal':goal,'frame':p['frame'],'first':pts[0].tolist(),'last':pts[-1].tolist(),'length_m':float(np.linalg.norm(np.diff(pts,axis=0),axis=1).sum()),'max_left_m':float(cross.max()),'max_right_m':float(cross.min()),'first_2m_yaw_deg':math.degrees(math.atan2(*(pts[min(14,len(pts)-1)]-pts[0])[::-1]))})

summary['tf_stats']={}
for edge,rows in tf.items():
    d=np.linalg.norm(np.diff(rows[:,2:4],axis=0),axis=1);ang=np.unwrap(np.array([yaw(q) for q in rows[:,5:9]]));i=int(np.argmax(d)) if len(d) else 0
    summary['tf_stats'][edge]={'count':len(rows),'translation_range':np.ptp(rows[:,2:5],axis=0).tolist(),'yaw_range_deg':float(np.ptp(ang)*180/math.pi),'max_xy_step_m':float(d[i]) if len(d) else 0,'step_time':local(rows[min(i+1,len(rows)-1),0]),'step_header_dt_s':float(rows[min(i+1,len(rows)-1),1]-rows[i,1]),'max_yaw_step_deg':float(np.max(np.abs(np.diff(ang)))*180/math.pi) if len(ang)>1 else 0}

# Feedback is navigation's own TF pose, not external ground truth. Sample all stages.
summary['samples']=[]
for t in [1794206325,1794206331,1794206333,1794206343,1794206346,1794206350,1794206355,1794206360,1794206370,1794206380,1794206381,1794206390,1794206400,1794206401,manual,1794206430,1794206450,1794206500,1794206530]:
    row=fb[nearest(fb,t)];v=veh[nearest(veh,t)];c=data['cmd'][nearest(data['cmd'],t)];pn=max((p for p in plans if p['t']<=t),key=lambda p:p['t'],default=None)
    goal=106 if row[0]<seg107start else 107;a=waypoints[113] if goal==106 else waypoints[106];b=waypoints[goal]
    nearest_plan_distance=None
    if pn is not None:
        points=np.asarray(pn['points']);d=points[1:]-points[:-1];ratio=np.clip(np.sum((row[2:4]-points[:-1])*d,axis=1)/np.maximum(np.sum(d*d,axis=1),1e-20),0,1);nearest_plan_distance=float(np.min(np.linalg.norm(row[2:4]-(points[:-1]+ratio[:,None]*d),axis=1)))
    summary['samples'].append({'time':local(row[0]),'x':row[2],'y':row[3],'yaw_deg':math.degrees(row[5]),'cross_m':float(signed(row[2:4],a,b)),'distance_remaining':row[6],'speed_kmh':v[2],'steer_deg':v[3],'drive_mode':int(v[4]),'is_autodrive':bool(v[5]),'cmd_v':c[1],'cmd_w':c[2],'nearest_current_plan_m':nearest_plan_distance})

(ROOT/'trajectory_evidence.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k not in ('plan_stats','samples')},ensure_ascii=False,indent=2))
print('SAMPLES');print(json.dumps(summary['samples'],ensure_ascii=False,indent=2))

# Exact 106 success -> coasting turn -> stationary wait -> 107 start, using past samples.
def before(a,t): return a[max(0,np.searchsorted(a[:,0],t,side='right')-1)]
mo=tf['map->odom'][0];ob=tf['odom->base_link']; rows=[]
for t in [1794206331.0,1794206331.5,1794206331.918343,1794206332.0,1794206332.5,1794206333.0,1794206334.0,1794206340.0,1794206343.987593,1794206344.5,1794206345.0,1794206346.0,1794206350.0,1794206355.0,1794206359.028]:
    r=before(ob,t);position=mo[2:5]+rot(mo[5:9],r[2:5]);angle=yaw(qmul(mo[5:9],r[5:9]));v=before(veh,t);c=before(data['cmd'],t);n=before(data['cmd_nav'],t);e=before(data['ecu'],t)
    rows.append({'time':local(t),'tf_time':local(r[0]),'x':float(position[0]),'y':float(position[1]),'map_yaw_deg':math.degrees(angle),'cross_106_107_m':float(signed(position[:2],waypoints[106],waypoints[107])),'speed_kmh':float(v[2]),'steer_deg':float(v[3]),'cmd_v_mps':float(c[1]),'cmd_w_rps':float(c[2]),'cmd_age_s':t-c[0],'cmd_nav_v_mps':float(n[1]),'cmd_nav_w_rps':float(n[2]),'ecu_motor_kmh':float(e[1]),'ecu_steer_deg':float(e[2]),'ecu_brake':bool(e[3])})
summary['106_107_transition']=rows
summary['reference_direction_106_107_deg']=math.degrees(math.atan2(*(waypoints[107]-waypoints[106])[::-1]))
(ROOT/'trajectory_evidence.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
image=np.asarray(Image.open('/home/akun/workspace/Car_jetson/multi_map_navigation/src/map/3b2mus/3b2mus_clean.pgm'))
fig,axs=plt.subplots(1,3,figsize=(17,8),gridspec_kw={'width_ratios':[1.0,1.3,1.3]})
def background(ax,xlim,ylim):
    x0=max(0,int((xlim[0]+92.9)/.1)-1);x1=min(image.shape[1],int((xlim[1]+92.9)/.1)+2)
    y0=max(0,int((ylim[0]+532)/.1)-1);y1=min(image.shape[0],int((ylim[1]+532)/.1)+2)
    sub=np.flipud(image)[y0:y1,x0:x1]
    ax.imshow(sub,cmap='gray',origin='lower',extent=(-92.9+x0*.1,-92.9+x1*.1,-532+y0*.1,-532+y1*.1),vmin=0,vmax=255,alpha=.65)
    ax.set_xlim(*xlim);ax.set_ylim(*ylim);ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)');ax.grid(alpha=.2)
a,b=waypoints[106],waypoints[107]; auto=(fb[:,0]>=seg107start)&(fb[:,0]<manual);man=fb[:,0]>=manual
background(axs[0],(105,130),(-255,-135));axs[0].plot([a[0],b[0]],[a[1],b[1]],'r--',label='106 to 107 reference')
firstplan=next(p for p in plans if p['t']>seg107start);fp=np.asarray(firstplan['points']);axs[0].plot(fp[:,0],fp[:,1],color='orange',label='first 107 global plan')
axs[0].plot(fb[auto,2],fb[auto,3],color='tab:blue',label='auto TF feedback');axs[0].plot(fb[man,2],fb[man,3],color='tab:purple',label='manual TF feedback')
axs[0].scatter([a[0],b[0]],[a[1],b[1]],c='red',s=35);axs[0].text(*a,' 106');axs[0].text(*b,' 107');axs[0].set_title('Route + static PGM (not live costmap)');axs[0].legend(fontsize=8,loc='lower left')
background(axs[1],(112,120),(-151,-140));axs[1].set_aspect('equal');axs[1].plot([a[0],b[0]],[a[1],b[1]],'r--',label='straight reference')
axs[1].plot(fp[:,0],fp[:,1],color='orange',label='first 107 global plan');axs[1].plot(fb[auto,2],fb[auto,3],color='tab:blue',label='auto TF feedback');axs[1].plot(fb[man,2],fb[man,3],color='tab:purple',alpha=.65,label='manual TF feedback')
peak=fb[auto][np.argmax(signed(fb[auto,2:4],a,b))];axs[1].scatter([peak[2]],[peak[3]],c='blue',s=35);axs[1].annotate('+0.587 m left\n14:39:19.028',(peak[2],peak[3]),xytext=(peak[2]+.6,peak[3]+.8),arrowprops={'arrowstyle':'->'},bbox={'facecolor':'white','alpha':.85,'edgecolor':'none'},fontsize=9)
start=fb[auto][0];axs[1].arrow(start[2],start[3],math.cos(start[5])*1.3,math.sin(start[5])*1.3,color='green',width=.04,length_includes_head=True);axs[1].text(start[2]-.8,start[3]+.6,'start yaw -65.8 deg\nreference -85.9 deg',fontsize=9,bbox={'facecolor':'white','alpha':.85,'edgecolor':'none'})
axs[1].set_title('Recorded 107 start: TF pose is not ground truth');axs[1].legend(fontsize=8,loc='lower left')
secs=fb[:,0]-seg107start;cross=signed(fb[:,2:4],a,b);sel=(secs>=0)&(secs<110)
axs[2].plot(secs[sel],cross[sel],color='tab:blue',label='left cross-track (m)');axs[2].axvline(manual-seg107start,color='tab:purple',linestyle='--',label='manual 14:40:13.831');axs[2].axhline(0,color='gray',linewidth=.8);axs[2].set_xlabel('seconds after 14:39:03');axs[2].set_ylabel('signed cross-track (m), positive left');axs[2].set_ylim(-.4,.8);axs[2].grid(alpha=.3)
ay=axs[2].twinx();ay.plot(secs[sel],np.degrees(fb[sel,5]),color='tab:green',alpha=.65,label='map yaw (deg)');ay.axhline(summary['reference_direction_106_107_deg'],color='tab:green',linestyle=':',label='reference yaw');ay.set_ylabel('map yaw (deg)');ay.set_ylim(-130,-50)
axs[2].legend(fontsize=8,loc='upper left');ay.legend(fontsize=8,loc='lower right');axs[2].set_title('Auto left drift happens before manual takeover')
fig.suptitle('2026-11-09 106 to 107 navigation reconstruction | all times UTC+08:00',fontsize=14)
fig.tight_layout();fig.savefig(ROOT/'trajectory_deviation.png',dpi=160);plt.close(fig)

from scipy.ndimage import distance_transform_edt
static_binary=np.flipud(image)<89
distance=distance_transform_edt(~static_binary)*.1
auto_rows=fb[auto]
ix=((auto_rows[:,2]+92.9)/.1).astype(int);iy=((auto_rows[:,3]+532)/.1).astype(int)
summary['static_pgm_check']={'method':'PGM pixels gray<89 classified occupied; Euclidean pixel-center distance transform; current static file is background, not live costmap','automatic_center_min_nearest_occupied_pixel_m':float(distance[iy,ix].min()),'peak_left_nearest_occupied_pixel_m':float(distance[int((peak[3]+532)/.1),int((peak[2]+92.9)/.1)]),'automatic_center_occupied_pixel_count':int(np.sum(static_binary[iy,ix]))}
delta=fp[1:]-fp[:-1];norm=np.maximum(np.sum(delta*delta,axis=1),1e-20);distances=[]
for row in auto_rows[auto_rows[:,0]>=firstplan['t']]:
    ratio=np.clip(np.sum((row[2:4]-fp[:-1])*delta,axis=1)/norm,0,1)
    distances.append(np.linalg.norm(row[2:4]-(fp[:-1]+ratio[:,None]*delta),axis=1).min())
summary['107_auto_fixed_first_plan_tracking']={'first_plan_time':local(firstplan['t']),'max_distance_m':float(max(distances)),'p95_distance_m':float(np.percentile(distances,95))}
(ROOT/'trajectory_evidence.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
