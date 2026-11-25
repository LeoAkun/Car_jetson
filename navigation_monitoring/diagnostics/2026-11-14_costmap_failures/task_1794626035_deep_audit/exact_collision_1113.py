import sqlite3,glob,math,sys,datetime
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ROOT=sys.argv[1]; infos={}
for p in sorted(glob.glob(ROOT+'/*.db3')):
 c=sqlite3.connect('file:'+p+'?mode=ro',uri=True)
 for n,t,i in c.execute('select name,type,id from topics'):infos.setdefault(n,[]).append((p,t,i))
 c.close()
def rows(topic,t0,t1):
 out=[]
 for p,t,i in infos[topic]:
  c=sqlite3.connect('file:'+p+'?mode=ro',uri=True);out += [(x/1e9,d) for x,d in c.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(i,int(t0*1e9),int(t1*1e9)))];c.close()
 return sorted(out)
def dec(topic,t0,t1):typ=get_message(infos[topic][0][1]);return [(t,deserialize_message(d,typ)) for t,d in rows(topic,t0,t1)]
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def dt(t):return datetime.datetime.fromtimestamp(t,datetime.timezone(datetime.timedelta(hours=8))).strftime('%H:%M:%S.%f')[:-3]
fb=dec('/navigate_to_pose/_action/feedback',1794626070,1794626227)
def pose(t):
 z=min(fb,key=lambda z:abs(z[0]-t));p=z[1].feedback.current_pose.pose;return p.position.x,p.position.y,yaw(p.orientation),z[0]
def line(x0,y0,x1,y1):
 x,y=x0,y0;dx=abs(x1-x0);dy=abs(y1-y0);cur=0
 if x1>=x0:xinc1=xinc2=1
 else:xinc1=xinc2=-1
 if y1>=y0:yinc1=yinc2=1
 else:yinc1=yinc2=-1
 if dx>=dy:xinc1=0;yinc2=0;den=dx;num=dx//2;numadd=dy;numpix=dx
 else:xinc2=0;yinc1=0;den=dy;num=dy//2;numadd=dx;numpix=dy
 while cur<=numpix:
  yield x,y
  num+=numadd
  if num>=den:num-=den;x+=xinc1;y+=yinc1
  x+=xinc2;y+=yinc2;cur+=1
for ts,m in dec('/global_costmap/costmap_raw',1794626095,1794626111):
 x,y,th,_=pose(ts);md=m.metadata;r=md.resolution;ox=md.origin.position.x;oy=md.origin.position.y;sx=md.size_x;sy=md.size_y;a=np.frombuffer(m.data,dtype=np.uint8).reshape(sy,sx)
 mx=int((x-ox)/r);my=int((y-oy)/r);cx=ox+(mx+.5)*r;cy=oy+(my+.5)*r;abin=round(th/(2*math.pi/64))%64;at=abin*2*math.pi/64
 fp=[(-.45,-.30),(-.45,.30),(.45,.30),(.45,-.30)]
 mc=[]
 for rx,ry in fp:
  wx=cx+rx*math.cos(at)-ry*math.sin(at);wy=cy+rx*math.sin(at)+ry*math.cos(at);mc.append((int((wx-ox)/r),int((wy-oy)/r)))
 cells=[]
 for i in range(4):cells += list(line(*mc[i],*mc[(i+1)%4]))
 seen=[]
 for gx,gy in cells:
  if (gx,gy) not in seen:seen.append((gx,gy))
 bad=[(gx,gy,int(a[gy,gx]),ox+(gx+.5)*r,oy+(gy+.5)*r) for gx,gy in seen if a[gy,gx]>=254]
 print(dt(ts),'pose',round(x,3),round(y,3),round(math.degrees(th),2),'cell',mx,my,'center',int(a[my,mx]),'bin',abin,'corners',mc,'maxBoundary',max(int(a[gy,gx]) for gx,gy in seen),'collision',bool(bad),'bad',bad)
