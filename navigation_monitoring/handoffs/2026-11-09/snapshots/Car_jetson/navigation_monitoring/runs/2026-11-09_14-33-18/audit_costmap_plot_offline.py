from pathlib import Path
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from PIL import Image
import json,numpy as np,sqlite3,datetime,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-09_14-33-18');S=np.load(R/'costmap_deviation_snapshots.npz',allow_pickle=True);D=json.load(open(R/'costmap_deviation_evidence.json'))
# exact map<-odom near target
tr=[]
for p in sorted((R/'navigation_core_bag').glob('*.db3'),key=lambda p:int(p.stem.rsplit('_',1)[1])):
 c=sqlite3.connect('file:'+str(p)+'?mode=ro',uri=True)
 for t,b in c.execute("select m.timestamp,m.data from messages m join topics t on t.id=m.topic_id where t.name='/tf'"):
  if t>1794206420e9:continue
  for x in deserialize_message(b,get_message('tf2_msgs/msg/TFMessage')).transforms:
   if x.header.frame_id=='map':tr.append((x.header.stamp.sec+x.header.stamp.nanosec/1e9,[x.transform.translation.x,x.transform.translation.y,x.transform.translation.z],[x.transform.rotation.x,x.transform.rotation.y,x.transform.rotation.z,x.transform.rotation.w]))
 c.close()
ttimes=np.array([t[0] for t in tr])
def tfat(t):return tr[np.argmin(abs(ttimes-t))]
def mappts(points,t):
 _,v,q=tfat(t);return Rotation.from_quat(q).apply(points)+v
im=np.array(Image.open('/home/akun/workspace/Car_jetson/multi_map_navigation/src/map/3b2mus/3b2mus_clean.pgm'))
H,W=im.shape;res=.1;origin=np.array([-92.9,-532.])
yy,xx=np.where(im<89.25);occ=np.c_[origin[0]+(xx+.5)*res,origin[1]+(H-yy-.5)*res]
near=(occ[:,0]>105)&(occ[:,0]<125)&(occ[:,1]>-160)&(occ[:,1]<-130);occ=occ[near];tree=cKDTree(occ)
results=[]
for sname in S:
 s=S[sname][0]
 if not sname.startswith('odom'):continue
 row=s['row'];t=row['header'];a=s['data'];o=s['origin'];res=row['resolution'];p=np.array(row['pose']);mp=np.array(row['map_pose']);yaw=Rotation.from_quat(p[3:]).as_euler('xyz')[2]
 yy,xx=np.where(a==254);pts=np.c_[o[0]+(xx+.5)*res,o[1]+(yy+.5)*res,np.zeros(len(xx))];dist=np.linalg.norm(pts[:,:2]-p[:2],axis=1);i=np.argmin(dist)
 closest=pts[i];delta=closest[:2]-p[:2];body=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])@delta
 # planar map transform on z chosen currentbase height (local obstacle zdiscarded)
 wp=mappts(np.c_[pts[:,:2],np.full(len(pts),p[2])],t)
 mind,mini=tree.query(mp[:2]);result={'time':row['time'],'t':row['t'],'map_pose':mp[:3].tolist(),'nearest_local_lethal_m':float(dist[i]),'nearest_local_lethal_body_xy':body.tolist(),'nearest_local_lethal_map_xy':wp[i,:2].tolist(),'nearest_static_occupied_m':float(mind),'nearest_static_occupied_xy':occ[mini].tolist()};results.append(result)
print(json.dumps(results,indent=2))
# Plot latestlocal scan distribution in map near road/path
fig,axes=plt.subplots(1,3,figsize=(16,6))
for ax,target in zip(axes,[1794206350.,1794206360.,1794206410.]):
 s=S['odom_'+str(target)][0];row=s['row'];t=row['header'];a=s['data'];o=s['origin'];res=row['resolution'];p=np.array(row['pose']);mp=np.array(row['map_pose']);yaw=Rotation.from_quat(p[3:]).as_euler('xyz')[2]
 yy,xx=np.where(a>0);pts=np.c_[o[0]+(xx+.5)*res,o[1]+(yy+.5)*res,np.full(len(xx),p[2])];wp=mappts(pts,t);costs=a[yy,xx];ax.scatter(wp[:,0],wp[:,1],c=costs,cmap='YlOrRd',s=8,vmin=0,vmax=254,alpha=.6)
 ax.scatter(occ[:,0],occ[:,1],c='k',s=5,label='Static occupied cells')
 plan=np.array(s['plan']);pp=mappts(plan,t);ax.plot(pp[:,0],pp[:,1],c='lime',linewidth=2,label='Global plan')
 fb=np.array(D['feedback_samples_1hz']);f=fb[(fb[:,0]>=1794206343)&(fb[:,0]<=row['t'])];ax.plot(f[:,2],f[:,3],color='blue',linewidth=2,label='Recorded pose')
 xy=np.array([[-.28,-.44],[-.28,.44],[.28,.44],[.28,-.44],[-.28,-.44]])
 poly=xy@np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]).T+p[:2];polym=mappts(np.c_[poly,np.full(len(poly),p[2])],t);ax.plot(polym[:,0],polym[:,1],c='blue',lw=2)
 ax.set_title(row['time'][11:23]+' log time (UTC+8)');ax.set_xlim(112,119);ax.set_ylim(-151,-142);ax.set_aspect('equal');ax.grid(alpha=.3);ax.set_xlabel('Map X / m')
axes[0].set_ylabel('Map Y / m');axes[0].legend(fontsize=8);fig.suptitle('107 segment: static map, local obstacle costs, global path and recorded position');fig.tight_layout();fig.savefig(R/'costmap_deviation_comparison.png',dpi=160)
D['snapshot_nearest_obstacles']=results;D['static_map']={'path':'/home/akun/workspace/Car_jetson/multi_map_navigation/src/map/3b2mus/3b2mus_clean.pgm','resolution':.1,'origin':origin.tolist(),'occupied_pixel_threshold':89.25}
(R/'costmap_deviation_evidence.json').write_text(json.dumps(D,ensure_ascii=False,indent=2))
