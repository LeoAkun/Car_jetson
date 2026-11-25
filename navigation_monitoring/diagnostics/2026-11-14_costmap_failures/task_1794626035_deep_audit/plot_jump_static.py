#!/usr/bin/env python3
import importlib.util,sys,math
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
BASE=Path('/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-14_costmap_failures')
s=importlib.util.spec_from_file_location('h',BASE/'analyze_first_lethal_raw.py');h=importlib.util.module_from_spec(s);sys.modules['h']=h;s.loader.exec_module(h)
grid=h.load_static_grid()
poses=[
 ('before TF jump\n11:16:38.299 bag',(151.7036645798904,-146.07292380808667,1.7168097293257778),'#1b9e77'),
 ('after TF jump\n11:16:38.310 bag',(150.76455236892255,-145.34687135608385,1.6813941877734229),'#d95f02'),
 ('first dense lethal\n11:16:39.073',(150.4416116441075,-144.8627734065141,1.3791364126527415),'#7570b3'),
]
# fixed crop
x0,x1=149.8,152.3;y0,y1=-146.7,-144.1
m0=grid.world_to_map(x0,y0);m1=grid.world_to_map(x1,y1)
arr=grid.data[m0[1]:m1[1]+1,m0[0]:m1[0]+1]
extent=[grid.origin_x+m0[0]*grid.resolution,grid.origin_x+(m1[0]+1)*grid.resolution,grid.origin_y+m0[1]*grid.resolution,grid.origin_y+(m1[1]+1)*grid.resolution]
fig,ax=plt.subplots(figsize=(8,7),constrained_layout=True)
# free white, unknown gray, occupied black
view=np.where(arr==100,0.0,np.where(arr==0,1.0,.5))
ax.imshow(view,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=1,interpolation='nearest')
for label,p,c in poses:
 poly=h.transform_polygon(h.FOOTPRINT,p);poly=np.vstack([poly,poly[0]])
 ax.plot(poly[:,0],poly[:,1],color=c,lw=2.5,label=label);ax.scatter([p[0]],[p[1]],color=c,s=40,zorder=5)
 ax.arrow(p[0],p[1],.35*math.cos(p[2]),.35*math.sin(p[2]),head_width=.07,head_length=.1,color=c,length_includes_head=True,zorder=5)
ax.annotate('~1.19 m estimate jump',xy=poses[1][1][:2],xytext=poses[0][1][:2],arrowprops=dict(arrowstyle='->',color='crimson',lw=2),ha='center',va='bottom',color='crimson',fontsize=11)
ax.set(xlim=(x0,x1),ylim=(y0,y1),xlabel='map x (m)',ylabel='map y (m)',title='11:13 failure: odom/TF correction moves footprint into static occupied map')
ax.set_aspect('equal');ax.grid(color='#999',alpha=.25);ax.legend(loc='upper right',fontsize=9,framealpha=.9)
fig.savefig(Path(__file__).with_name('jump_into_static_obstacle.png'),dpi=180)
