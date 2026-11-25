#!/usr/bin/env python3
import collections, json, math, statistics, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from multi_map_navigation_msgs.msg import EpochOdometry, LioCorrectionEvent, LioMapOdomCorrection, LioReadiness

def ns(stamp): return stamp.sec*1_000_000_000+stamp.nanosec
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def angle(a): return abs(math.atan2(math.sin(a), math.cos(a)))
def pct(v,q):
    if not v: return None
    s=sorted(v); p=(len(s)-1)*q; lo=int(p); hi=math.ceil(p)
    return s[lo] if lo==hi else s[lo]*(hi-p)+s[hi]*(p-lo)
classes={
 '/odom':Odometry, '/odom_incremental':Odometry,
 '/lio_sam/epoch_odometry':EpochOdometry,
 '/lio_sam/correction_event':LioCorrectionEvent,
 '/lio_sam/map_odom_correction':LioMapOdomCorrection,
 '/lio_sam/localization_fault':String,
 '/lio_sam/imu_preintegration_ready':Bool,
 '/lio_sam/imu_preintegration_readiness':LioReadiness,
 '/lio_sam/mapping/odometry':Odometry,
 '/lio_sam/mapping/odometry_incremental':Odometry,
}
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
counts=collections.Counter(); odoms={}; epoch=[]; corrections=[]; mapcorr=[]; faults=[]; readiness=[]
while r.has_next():
 topic,data,bag_ns=r.read_next(); counts[topic]+=1
 cls=classes.get(topic)
 if not cls: continue
 m=deserialize_message(data,cls)
 if topic in ('/odom','/odom_incremental','/lio_sam/mapping/odometry','/lio_sam/mapping/odometry_incremental'):
  odoms.setdefault(topic,[]).append((ns(m.header.stamp),bag_ns,m.pose.pose))
 elif topic=='/lio_sam/epoch_odometry': epoch.append((ns(m.header.stamp),m.correction_epoch,m.instance_id,m.generation))
 elif topic=='/lio_sam/correction_event': corrections.append({'stamp_ns':ns(m.header.stamp),'epoch':m.correction_epoch,'source':m.source,'accepted':m.accepted,'translation_residual':m.translation_residual,'yaw_residual':m.yaw_residual,'arrival_delay_sec':m.arrival_delay_sec,'reason':m.reason})
 elif topic=='/lio_sam/map_odom_correction': mapcorr.append({'stamp_ns':ns(m.header.stamp),'epoch':m.correction_epoch,'state':m.state,'accepted':m.accepted,'translation_residual':m.translation_residual,'yaw_residual':m.yaw_residual,'reason':m.reason})
 elif topic=='/lio_sam/localization_fault': faults.append({'bag_ns':bag_ns,'data':m.data})
 elif topic=='/lio_sam/imu_preintegration_ready': readiness.append({'kind':'legacy','bag_ns':bag_ns,'ready':m.data})
 elif topic=='/lio_sam/imu_preintegration_readiness': readiness.append({'kind':'identity','bag_ns':bag_ns,'ready':m.ready,'instance':m.instance_id,'generation':m.generation})
summary={'counts':dict(counts),'odometry':{},'epoch':{},'correction':{},'map_odom_correction':{},'faults':faults,'readiness':readiness}
for topic,rows in odoms.items():
 steps=[]; ysteps=[]; dts=[]; worst=[]
 for prev,cur in zip(rows,rows[1:]):
  dt=(cur[0]-prev[0])/1e9; dts.append(dt)
  p,q=prev[2].position,cur[2].position
  d=math.sqrt((q.x-p.x)**2+(q.y-p.y)**2+(q.z-p.z)**2); ys=math.degrees(angle(yaw(cur[2].orientation)-yaw(prev[2].orientation)))
  steps.append(d); ysteps.append(ys); worst.append((d,ys,dt,cur[0],cur[1],q.x,q.y,q.z))
 summary['odometry'][topic]={'samples':len(rows),'first_stamp_ns':rows[0][0] if rows else None,'last_stamp_ns':rows[-1][0] if rows else None,'stamp_backwards_or_equal':sum(x<=0 for x in dts),'stamp_gap_over_0_2':sum(x>.2 for x in dts),'max_dt_sec':max(dts) if dts else None,'step_m_max':max(steps) if steps else None,'step_m_p99':pct(steps,.99),'step_m_p999':pct(steps,.999),'yaw_step_deg_max':max(ysteps) if ysteps else None,'hard_translation_over_0_05':sum(x>.05 for x in steps),'hard_yaw_over_1deg':sum(x>1 for x in ysteps),'worst_10':sorted(worst,reverse=True)[:10]}
summary['epoch']={'samples':len(epoch),'epochs':sorted(set(x[1] for x in epoch)),'epoch_decreases':sum(b[1]<a[1] for a,b in zip(epoch,epoch[1:])),'identities':sorted(set((x[2],x[3]) for x in epoch))}
summary['correction']={'samples':len(corrections),'accepted':sum(x['accepted'] for x in corrections),'rejected':sum(not x['accepted'] for x in corrections),'source_counts':dict(collections.Counter(x['source'] for x in corrections)),'max_translation_residual':max((x['translation_residual'] for x in corrections),default=None),'events':corrections}
summary['map_odom_correction']={'samples':len(mapcorr),'accepted':sum(x['accepted'] for x in mapcorr),'state_counts':dict(collections.Counter(x['state'] for x in mapcorr)),'max_translation_residual':max((x['translation_residual'] for x in mapcorr),default=None)}
print(json.dumps(summary,indent=2,ensure_ascii=False))
