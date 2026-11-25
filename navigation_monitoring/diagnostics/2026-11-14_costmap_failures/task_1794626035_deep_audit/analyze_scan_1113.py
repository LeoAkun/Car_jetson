import sqlite3,glob,os,math,datetime,sys,statistics,json
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
ROOT=sys.argv[1]
infos={}
for p in sorted(glob.glob(ROOT+'/*.db3')):
 con=sqlite3.connect('file:'+p+'?mode=ro',uri=True)
 for name,typ,tid in con.execute('select name,type,id from topics'): infos.setdefault(name,[]).append((p,typ,tid))
 con.close()
def msgs(topic,t0,t1):
 typ=get_message(infos[topic][0][1]); out=[]
 for p,_,tid in infos[topic]:
  con=sqlite3.connect('file:'+p+'?mode=ro',uri=True)
  for ts,data in con.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(tid,int(t0*1e9),int(t1*1e9))):
   out.append((ts/1e9,deserialize_message(data,typ)))
  con.close()
 return sorted(out)
def dt(ts): return datetime.datetime.fromtimestamp(ts,datetime.timezone(datetime.timedelta(hours=8))).strftime('%H:%M:%S.%f')[:-3]
def candidate_front(m):
 # exact C++ point transform, count and points
 out=[]; a=m.angle_min
 for k,r in enumerate(m.ranges):
  angle=a+k*m.angle_increment
  # C++ loops from -pi to pi and count, same
  if not math.isfinite(r) or r<=0: continue
  x=r*math.cos(angle); y=r*math.sin(angle)
  # alpha=45, offsets .43,.28
  px=x*math.cos(math.pi/4)-y*math.sin(math.pi/4)+.43
  py=x*math.sin(math.pi/4)+y*math.cos(math.pi/4)+.28
  deg=angle*180/math.pi
  if 132<=deg<=135 and .60<=r<=.83 and -.40<=px<=-.18 and .28<=py<=.33:
   out.append((k,deg,r,px,py))
 return out
front=msgs('/laser/scan_front',1794625927,1794626247)
print('front n',len(front))
counts=[]
for ts,m in front:
 c=candidate_front(m)
 finite=[r for r in m.ranges if math.isfinite(r) and r>0]
 counts.append((ts,len(c),min(finite) if finite else None,max(finite) if finite else None))
print('candidate frames',sum(n>0 for _,n,_,_ in counts),'max',max(n for _,n,_,_ in counts), 'total',sum(n for _,n,_,_ in counts))
for x in counts:
 if x[1]>0: print('CAND',dt(x[0]),x[1],x[2],x[3])
# summarize min per minute 10sec bins
for lo in [1794625927+i*10 for i in range(32)]:
 z=[x for x in counts if lo<=x[0]<lo+10]
 if z: print('BIN',dt(lo),len(z),'candframes',sum(x[1]>0 for x in z),'maxcand',max(x[1] for x in z),'minmed',statistics.median([x[2] for x in z if x[2] is not None]))
# compare merged raw/filtered nearest same stamp (they should align)
raw=msgs('/laser/scan_merge_laser',1794625927,1794626247); fil=msgs('/laser/scan_merge_laser_filtered',1794625927,1794626247)
print('merged n',len(raw),len(fil))
# pair by timestamp nearest <.005
j=0; diffs=[]
for ts,m in raw:
 while j+1<len(fil) and abs(fil[j+1][0]-ts)<abs(fil[j][0]-ts):j+=1
 if j<len(fil) and abs(fil[j][0]-ts)<.005:
  f=fil[j][1]; d=[]
  for k,(a,b) in enumerate(zip(m.ranges,f.ranges)):
   # filtered only removes points => raw finite and filtered inf/NaN or larger
   if (math.isfinite(a) and (not math.isfinite(b) or b>a+1e-4)):
    ang=m.angle_min+k*m.angle_increment
    d.append((k,ang*180/math.pi,a,b))
  diffs.append((ts,d))
print('paired',len(diffs),'frames with diff',sum(bool(d) for _,d in diffs),'removed bins total',sum(len(d) for _,d in diffs),'max/frame',max([len(d) for _,d in diffs] or [0]))
for ts,d in diffs:
 if d: print('DIFF',dt(ts),len(d),d[:20])
