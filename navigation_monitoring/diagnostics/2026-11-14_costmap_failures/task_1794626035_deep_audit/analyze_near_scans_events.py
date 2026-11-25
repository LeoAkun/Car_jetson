import sqlite3,glob,math,sys,datetime,collections
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
def dt(t):return datetime.datetime.fromtimestamp(t,datetime.timezone(datetime.timedelta(hours=8))).strftime('%H:%M:%S.%f')[:-3]
def stats(m):
 pts=[]
 for k,r in enumerate(m.ranges):
  if math.isfinite(r):
   a=m.angle_min+k*m.angle_increment;x=r*math.cos(a);y=r*math.sin(a);gap=max(abs(x)-.45,abs(y)-.30,0);pts.append((r,a*180/math.pi,x,y,gap,k))
 pts.sort();near=[z for z in pts if z[0]<1.0];edge=sorted(near,key=lambda z:z[4]);return pts[:10],near,edge[:15]
for ev in [1794626098.812,1794626109.794,1794626158,1794626199.073,1794626226.390]:
 print('\nEVENT',dt(ev))
 for topic in ['/laser/scan_front','/laser/scan_back','/laser/scan_merge_laser','/laser/scan_merge_laser_filtered']:
  z=min(dec(topic,ev-.2,ev+.2),key=lambda q:abs(q[0]-ev));low,near,edge=stats(z[1]);print(topic,'dt',round(z[0]-ev,4),'finite<1',len(near),'min10',[(round(a,3),round(b,1),round(c,3),round(d,3),round(e,3)) for a,b,c,d,e,k in low],'edge',[(round(a,3),round(b,1),round(c,3),round(d,3),round(e,3)) for a,b,c,d,e,k in edge])
