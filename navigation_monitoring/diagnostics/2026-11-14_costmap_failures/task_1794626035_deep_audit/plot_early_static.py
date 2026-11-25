#!/usr/bin/env python3
import importlib.util,sys,math
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
BASE=Path('/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-14_costmap_failures')
s=importlib.util.spec_from_file_location('h',BASE/'analyze_first_lethal_raw.py');h=importlib.util.module_from_spec(s);sys.modules['h']=h;s.loader.exec_module(h)
grid=h.load_static_grid()
poses=[
 ('11:14:57.75 plan start',(156.950002,-183.149995,1.374447),'#1b9e77'),
 ('11:14:58.812 event',(156.930222,-183.371829,1.346527),'#d95f02'),
 ('11:14:59.339 event',(156.889632,-183.304819,1.289180),'#7570b3'),
]
x0,x1=156.1,157.9;y0,y1=-184.2,-182.5;m0=grid.world_to_map(x0,y0);m1=grid.world_to_map(x1,y1);arr=grid.data[m0[1]:m1[1]+1,m0[0]:m1[0]+1];extent=[grid.origin_x+m0[0]*grid.resolution,grid.origin_x+(m1[0]+1)*grid.resolution,grid.origin_y+m0[1]*grid.resolution,grid.origin_y+(m1[1]+1)*grid.resolution]
fig,ax=plt.subplots(figsize=(9,7),constrained_layout=True);view=np.where(arr==100,0.,np.where(arr==0,1.,.5));ax.imshow(view,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=1,interpolation='nearest')
for label,p,c in poses:
 poly=h.transform_polygon(h.FOOTPRINT,p);poly=np.vstack([poly,poly[0]]);ax.plot(poly[:,0],poly[:,1],color=c,lw=2.5,label=label);ax.scatter([p[0]],[p[1]],color=c,s=40,zorder=5);ax.arrow(p[0],p[1],.25*math.cos(p[2]),.25*math.sin(p[2]),head_width=.04,head_length=.07,color=c,length_includes_head=True,zorder=5)
ax.set(xlim=(x0,x1),ylim=(y0,y1),xlabel='map x (m)',ylabel='map y (m)',title='11:13 failure early warnings: static-map boundary at footprint edge');ax.set_aspect('equal');ax.grid(color='#999',alpha=.25);ax.legend(loc='upper right',fontsize=9);fig.savefig(BASE/'task_1794626035_deep_audit/early_static_boundary.png',dpi=180)
