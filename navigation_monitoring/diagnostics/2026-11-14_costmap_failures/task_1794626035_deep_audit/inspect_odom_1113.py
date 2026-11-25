import math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
BAG='/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-14_11-12-06_manual_publish_monitor/navigation_core_bag'
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
types={x.name:x.type for x in r.get_all_topics_and_types()}; topics=['/odom','/lio_sam/mapping/odometry','/lio_sam/mapping/odometry_incremental','/tf']
r.set_filter(rosbag2_py.StorageFilter(topics=[x for x in topics if x in types])); cls={x:get_message(types[x]) for x in topics if x in types}
rows={x:[] for x in topics}
while r.has_next():
 t,raw,tn=r.read_next(); m=deserialize_message(raw,cls[t]); ts=tn*1e-9
 if t=='/tf':
  for tr in m.transforms:
   if (tr.header.frame_id.strip('/'),tr.child_frame_id.strip('/')) in [('odom','base_link'),('map','odom')]:
    q=tr.transform.rotation; y=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)); rows.setdefault((t,tr.header.frame_id.strip('/'),tr.child_frame_id.strip('/')),[]).append((ts,tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9,tr.transform.translation.x,tr.transform.translation.y,y))
 else:
  p=m.pose.pose; q=p.orientation; y=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)); rows[t].append((ts,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.header.frame_id,m.child_frame_id,p.position.x,p.position.y,y,m.twist.twist.linear.x,m.twist.twist.angular.z, list(m.pose.covariance)[:3], list(m.twist.covariance)[:3]))
for k,v in rows.items():
 print('\n',k,'count',len(v))
 if not v: continue
 print('first',v[0]); print('last',v[-1])
 if isinstance(k,str) and k.startswith('/'):
  near=[z for z in v if abs(z[0]-1794626198.7)<1 or abs(z[0]-1794626199.7)<1]
 else:
  near=[z for z in v if abs(z[0]-1794626198.7)<.5 or abs(z[0]-1794626199.7)<.5]
 for z in near: print(z)
