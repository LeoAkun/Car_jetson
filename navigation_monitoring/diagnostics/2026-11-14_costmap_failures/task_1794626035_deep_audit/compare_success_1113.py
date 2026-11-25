import sqlite3,glob,math,sys,datetime,re,collections
import numpy as np
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ROOT=sys.argv[1];infos={}
for p in glob.glob(ROOT+'/*.db3'):
 c=sqlite3.connect('file:'+p+'?mode=ro',uri=True)
 for n,t,i in c.execute('select name,type,id from topics'):infos.setdefault(n,[]).append((p,t,i))
 c.close()
def dec(topic,t0,t1):
 typ=get_message(infos[topic][0][1]);z=[]
 for p,t,i in infos[topic]:
  c=sqlite3.connect('file:'+p+'?mode=ro',uri=True);z += [(ts/1e9,deserialize_message(d,typ)) for ts,d in c.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(i,int(t0*1e9),int(t1*1e9)))];c.close()
 return sorted(z)
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def dt(t):return datetime.datetime.fromtimestamp(t,datetime.timezone(datetime.timedelta(hours=8))).strftime('%H:%M:%S.%f')[:-3]
fb=dec('/navigate_to_pose/_action/feedback',1794626669,1794626933)
def pose(t):z=min(fb,key=lambda x:abs(x[0]-t));p=z[1].feedback.current_pose.pose;return p.position.x,p.position.y,yaw(p.orientation)
cm=dec('/global_costmap/costmap_raw',1794626669,1794626933)
# scan candidate exact old condition and actual ranges <1 merged
front=dec('/laser/scan_front',1794626669,1794626933);mr=dec('/laser/scan_merge_laser',1794626669,1794626933);mf=dec('/laser/scan_merge_laser_filtered',1794626669,1794626933)
cand=[]
for ts,m in front:
 for k,r in enumerate(m.ranges):
  a=m.angle_min+k*m.angle_increment
  if not math.isfinite(r) or r<=0:continue
  x=r*math.cos(a);y=r*math.sin(a);px=x*.70710678-y*.70710678+.43;py=x*.70710678+y*.70710678+.28;d=math.degrees(a)
  if 132<=d<=135 and .60<=r<=.83 and -.40<=px<=-.18 and .28<=py<=.33:cand.append((ts,d,r,px,py))
print('candidate messages',len(cand),'frames',len(set(round(z[0],6) for z in cand)),'firstlast',cand[:5],cand[-5:])
for t in [1794626670,1794626680,1794626690,1794626700,1794626710,1794626720,1794626728.624,1794626734.186,1794626743.085,1794626750,1794626760,1794626770,1794626780,1794626787]:
 x,y,th=pose(t);z=min(cm,key=lambda z:abs(z[0]-t));ts,m=z;md=m.metadata;a=np.frombuffer(m.data,dtype=np.uint8).reshape(md.size_y,md.size_x);mx=int((x-md.origin.position.x)/md.resolution);my=int((y-md.origin.position.y)/md.resolution)
 print(dt(t),'pose',round(x,2),round(y,2),round(math.degrees(th),1),'cmapdt',round(ts-t,2),'center',int(a[my,mx]))
