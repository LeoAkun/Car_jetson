import importlib.util,sys,math,collections
from pathlib import Path
p=Path('/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-14_costmap_failures/analyze_first_lethal_raw.py')
s=importlib.util.spec_from_file_location('h',p);h=importlib.util.module_from_spec(s);sys.modules['h']=h;s.loader.exec_module(h)
bag=Path('/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-14_11-12-06_manual_publish_monitor/navigation_core_bag'); static=h.load_static_grid()
# map->odom exact constant
m=(156.80516052246094,-192.34902954101562,1.5683262699313045)
for t in [1794626198.289,1794626198.299,1794626198.311,1794626198.32,1794626198.8,1794626199.073,1794626199.299,1794626199.31,1794626199.5]:
 typ,rows=h.topic_rows(bag,'/tf',int((t-.03)*1e9),int((t+.03)*1e9)); found=[]
 for b,blob in rows:
  msg=h.decode(typ,blob)
  for tr in msg.transforms:
   if (tr.header.frame_id,tr.child_frame_id)==('odom','base_link'):
    o=(tr.transform.translation.x,tr.transform.translation.y,h.yaw(tr.transform.rotation));found.append((b/1e9,h.ns(tr.header.stamp)/1e9,o))
 if not found:continue
 b,head,o=min(found,key=lambda z:abs(z[0]-t));pose=h.compose(m,o);smac,abin,cell=h.quantized_smac_pose(static,pose); st=h.static_footprint_stats(static,smac)
 print('target bag',t,'tfbag',b,'header',head,'odom',o,'map',pose,'smac',smac,'static boundary',st['boundary_occupancy_counts'],'center',st['center_occupancy'],'collision',st['geometric_collision_if_unknown_disallowed'])
