import sqlite3,glob,math,sys,datetime
import numpy as np
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=sys.argv[1];infos={}
for p in glob.glob(root+'/*.db3'):
 c=sqlite3.connect('file:'+p+'?mode=ro',uri=True)
 for n,t,i in c.execute('select name,type,id from topics'):infos.setdefault(n,[]).append((p,t,i))
 c.close()
def dec(topic,t0,t1):
 typ=get_message(infos[topic][0][1]);z=[]
 for p,t,i in infos[topic]:
  c=sqlite3.connect('file:'+p+'?mode=ro',uri=True);z += [(ts/1e9,deserialize_message(d,typ)) for ts,d in c.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(i,int(t0*1e9),int(t1*1e9)))];c.close()
 return sorted(z)
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def dt(t):return datetime.datetime.fromtimestamp(t,datetime.timezone(datetime.timedelta(hours=8))).strftime('%H:%M:%S')
fb=dec('/navigate_to_pose/_action/feedback',1794626070,1794626227)
def pose(t):z=min(fb,key=lambda x:abs(x[0]-t));p=z[1].feedback.current_pose.pose;return p.position.x,p.position.y,yaw(p.orientation)
im=np.array(Image.open('/home/akun/workspace/Car_jetson/multi_map_navigation/src/map/3b2mus/3b2mus_clean.pgm'));H,W=im.shape
def occ(wx,wy):mx=int((wx+92.9)/.1);my=int((wy+532)/.1);p=int(im[H-1-my,mx]);return (255-p)/255>.65,p
# sample footprint interior at 2cm and compute occupied fraction and max penetration from edge
for t in [1794626070,1794626080,1794626090,1794626098.812,1794626110,1794626130,1794626150,1794626170,1794626199,1794626210,1794626226]:
 x,y,th=pose(t);bad=[];n=0
 for rx in np.arange(-.45,.451,.02):
  for ry in np.arange(-.30,.301,.02):
   wx=x+math.cos(th)*rx-math.sin(th)*ry;wy=y+math.sin(th)*rx+math.cos(th)*ry;o,p=occ(wx,wy);n+=1
   if o: bad.append((min(.45-abs(rx),.30-abs(ry)),rx,ry,p))
 print(dt(t),round(x,2),round(y,2),round(math.degrees(th),1),'interior occupied samples',len(bad),'/',n,'max penetration',round(max([z[0] for z in bad] or [0]),3),'deepest',sorted(bad,reverse=True)[:5])
