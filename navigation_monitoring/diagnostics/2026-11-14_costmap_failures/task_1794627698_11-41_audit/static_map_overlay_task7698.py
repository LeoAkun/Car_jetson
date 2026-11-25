"""Create the estimated task_1794627698 footprint/static-map overlay.

This is an offline, read-only reconstruction.  The task's rosbag ended before
the NavigateToPose action, so the first-lethal pose is estimated by fitting the
observer's plan starts to its /odom samples.  It is not a replacement for the
missing global_costmap/costmap_raw message at 11:43:25.581.
"""

import json,numpy as np,math,datetime
from pathlib import Path
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
obs='/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-14_11-02-27_manual_publish_monitor/detail_observer.jsonl'
P=[];O=[]
for l in open(obs):
 try:o=json.loads(l)
 except:continue
 if 1794627735<=o.get('stamp',0)<=1794627855:
  if o.get('event')=='plan':P.append((o['stamp'],*o['first']))
  if o.get('event')=='pose' and o.get('topic')=='/odom':O.append((o['stamp'],o['x'],o['y'],o['q']))
pairs=[]
for s,x,y in P:
 q=min(O,key=lambda z:abs(z[0]-s))
 if abs(q[0]-s)<.15:pairs.append((np.array([q[1],q[2]]),np.array([x,y])))
X=np.array([z[0] for z in pairs]);Y=np.array([z[1] for z in pairs]);xc=X.mean(0);yc=Y.mean(0)
U,S,Vt=np.linalg.svd((X-xc).T@(Y-yc));R=U@Vt;t=yc-xc@R;rot=math.atan2(R[0,1],R[0,0])
def map_pose(stamp):
 q=min(O,key=lambda z:abs(z[0]-stamp));xy=np.array([q[1],q[2]])@R+t;z=q[3]
 yaw=math.atan2(2*(z[3]*z[2]+z[0]*z[1]),1-2*(z[1]**2+z[2]**2))+rot
 return xy[0],xy[1],yaw
def poly(pose):
 x,y,a=pose;fp=[(-.45,-.30),(-.45,.30),(.45,.30),(.45,-.30)]
 return np.array([[x+math.cos(a)*xx-math.sin(a)*yy,y+math.sin(a)*xx+math.cos(a)*yy] for xx,yy in fp])
im=np.array(Image.open('/home/akun/workspace/Car_jetson/multi_map_navigation/src/map/map2/map2_clean.pgm'));H,W=im.shape;ox,oy=-395,-148;res=.1
xmin,xmax=-79.5,-75.5;ymin,ymax=195.5,202.5
c0=int((xmin-ox)/res);c1=int((xmax-ox)/res);r0=H-1-int((ymax-oy)/res);r1=H-1-int((ymin-oy)/res)
crop=im[r0:r1+1,c0:c1+1]
fig,ax=plt.subplots(figsize=(8,10))
ax.imshow(crop,cmap='gray',origin='upper',extent=[ox+c0*res,ox+(c1+1)*res,oy+(H-(r1+1))*res,oy+(H-r0)*res],vmin=0,vmax=254)
pts=np.array([[x,y] for _,x,y in P if 1794627780<=_<=1794627804.6])
ax.plot(pts[:,0],pts[:,1],'-o',ms=2,lw=1.2,color='#00bcd4',label='valid plan start (~1 Hz)')
first_lethal_stamp = 1794627805.581230176
pre_lethal_plans = [p for p in P if p[0] < first_lethal_stamp]
last_plan = pre_lethal_plans[-1]
last=(last_plan[1],last_plan[2],map_pose(last_plan[0])[2])
lethal=map_pose(first_lethal_stamp)
ax.add_patch(Polygon(poly(last),fill=False,ec='#00e676',lw=2,label='last valid plan footprint'))
ax.add_patch(Polygon(poly(lethal),facecolor=(1,0,0,.18),ec='red',lw=2,label='estimated first-lethal footprint'))
ax.scatter([last[0],lethal[0]],[last[1],lethal[1]],c=['#00e676','red'],s=40,zorder=5)
ax.annotate(f'last plan\n({last[0]:.2f},{last[1]:.2f})',(last[0],last[1]),xytext=(-1.2,-.8),arrowprops=dict(arrowstyle='->'))
ax.annotate(f'first lethal estimate\n({lethal[0]:.2f},{lethal[1]:.2f})',(lethal[0],lethal[1]),xytext=(-1.6,1.2),arrowprops=dict(arrowstyle='->'))
ax.set(xlim=(xmin,xmax),ylim=(ymin,ymax),xlabel='map x (m)',ylabel='map y (m)',title='task_1794627698: map2 static occupancy vs configured footprint')
ax.grid(alpha=.25);ax.legend(loc='lower left');fig.tight_layout()
output_path = Path(__file__).with_name('static_map_overlay_estimated.png')
fig.savefig(output_path,dpi=180)
print(output_path,lethal,last)
