import sqlite3,glob,math,sys,datetime
import numpy as np
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ROOT=sys.argv[1];infos={}
for p in glob.glob(ROOT+'/*.db3'):
 c=sqlite3.connect('file:'+p+'?mode=ro',uri=True)
 for n,t,i in c.execute('select name,type,id from topics'):infos.setdefault(n,[]).append((p,t,i))
 c.close()
def rows(topic,t0,t1):
 z=[]
 for p,t,i in infos[topic]:
  c=sqlite3.connect('file:'+p+'?mode=ro',uri=True);z += [(ts/1e9,d) for ts,d in c.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(i,int(t0*1e9),int(t1*1e9)))];c.close()
 return sorted(z)
def dec(topic,t0,t1): typ=get_message(infos[topic][0][1]);return [(t,deserialize_message(d,typ)) for t,d in rows(topic,t0,t1)]
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def dt(t):return datetime.datetime.fromtimestamp(t,datetime.timezone(datetime.timedelta(hours=8))).strftime('%H:%M:%S.%f')[:-3]
fb=dec('/navigate_to_pose/_action/feedback',1794626070,1794626227)
def pose(t):z=min(fb,key=lambda x:abs(x[0]-t));p=z[1].feedback.current_pose.pose;return p.position.x,p.position.y,yaw(p.orientation)
im=np.array(Image.open('/home/akun/workspace/Car_jetson/multi_map_navigation/src/map/3b2mus/3b2mus_clean.pgm'));H,W=im.shape
def pix(wx,wy):mx=int((wx+92.9)/.1);my=int((wy+532)/.1);return int(im[H-1-my,mx])
# exact bresenham
def line(x0,y0,x1,y1):
 x,y=x0,y0;dx=abs(x1-x0);dy=abs(y1-y0);cur=0;x1i=x2i=1 if x1>=x0 else -1;y1i=y2i=1 if y1>=y0 else -1
 if dx>=dy:x1i=0;y2i=0;den=dx;num=dx//2;add=dy;n=dx
 else:x2i=0;y1i=0;den=dy;num=dy//2;add=dx;n=dy
 while cur<=n:
  yield x,y;num+=add
  if num>=den:num-=den;x+=x1i;y+=y1i
  x+=x2i;y+=y2i;cur+=1
for target in [1794626098.0388377,1794626099.3740113,1794626110.2270985,1794626158.0,1794626199.0336354,1794626226.4165127]:
 ts,m=min(dec('/global_costmap/costmap_raw',target-2,target+2),key=lambda z:abs(z[0]-target));x,y,th=pose(target);md=m.metadata;r=md.resolution;ox=md.origin.position.x;oy=md.origin.position.y;a=np.frombuffer(m.data,dtype=np.uint8).reshape(md.size_y,md.size_x);mx=int((x-ox)/r);my=int((y-oy)/r);cx=ox+(mx+.5)*r;cy=oy+(my+.5)*r;abin=round(th/(2*math.pi/64))%64;at=abin*2*math.pi/64;fp=[(-.45,-.30),(-.45,.30),(.45,.30),(.45,-.30)];corn=[]
 for rx,ry in fp:
  wx=cx+rx*math.cos(at)-ry*math.sin(at);wy=cy+rx*math.sin(at)+ry*math.cos(at);corn.append((int((wx-ox)/r),int((wy-oy)/r)))
 cells=[]
 for i in range(4):cells += list(line(*corn[i],*corn[(i+1)%4]))
 cells=list(dict.fromkeys(cells)); bad=[]
 for gx,gy in cells:
  c=int(a[gy,gx]);wx=ox+(gx+.5)*r;wy=oy+(gy+.5)*r
  if c>=253:bad.append((c,pix(wx,wy),round(wx,3),round(wy,3),gx,gy))
 from collections import Counter
 print('\n',dt(target),'cmap',dt(ts),'pose',round(x,3),round(y,3),round(math.degrees(th),1),'center',(int(a[my,mx]),pix(cx,cy)),'boundary bad count',len(bad),'pairs',Counter((z[0],z[1]) for z in bad))
 print('bad',bad)
