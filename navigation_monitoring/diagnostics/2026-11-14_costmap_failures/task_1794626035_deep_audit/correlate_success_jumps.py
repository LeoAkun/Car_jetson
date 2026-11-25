import bisect,datetime,math,sqlite3,glob
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
B=Path('/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-14_11-02-27_manual_publish_monitor/navigation_core_bag')

def rows(topic):
 out=[];typ=None
 for db in sorted(B.glob('*.db3')):
  c=sqlite3.connect(f'file:{db}?mode=ro&immutable=1',uri=True); q=c.execute('select id,type from topics where name=?',(topic,)).fetchone()
  if q:
   i,typ=q;out += c.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp,id',(i,1794626640000000000,1794626940000000000)).fetchall()
  c.close()
 out.sort();C=get_message(typ);return [(b,deserialize_message(raw,C)) for b,raw in out]
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def ts(st):return st.sec+st.nanosec*1e-9
def pose(m):p=m.pose.pose;return p.position.x,p.position.y,yaw(p.orientation)
def iso(s):return datetime.datetime.fromtimestamp(s,datetime.timezone(datetime.timedelta(hours=8))).strftime('%H:%M:%S.%f')
O=rows('/odom'); M=rows('/lio_sam/mapping/odometry');MI=rows('/lio_sam/mapping/odometry_incremental'); V=rows('/vehicle_status')
mb=[b for b,_ in M]; vb=[b for b,_ in V]
def nearest_before(arr,b):
 i=bisect.bisect_right(arr,b)-1;return i
j=[]
for (b0,a),(b1,c) in zip(O,O[1:]):
 t0,t1=ts(a.header.stamp),ts(c.header.stamp);dt=t1-t0
 if not 0<dt<.1:continue
 p0,p1=pose(a),pose(c);ds=math.hypot(p1[0]-p0[0],p1[1]-p0[1])
 if ds<.2:continue
 i=nearest_before(mb,b1); bm,mm=M[i]; prev=M[i-1] if i>0 else None
 iv=nearest_before(vb,b1);bv,v=V[iv]
 item={'odom_bag':b1/1e9,'odom_header':t1,'dt':dt,'ds':ds,'before':p0,'after':p1,'mapping_bag':bm/1e9,'mapping_age_bag':(b1-bm)/1e9,'mapping_header':ts(mm.header.stamp),'mapping_pose':pose(mm),'vehicle_bag':bv/1e9,'vehicle_speed_kmh':v.cur_speed,'wheel_left':v.left_wheel_speed,'wheel_right':v.right_wheel_speed}
 if prev:
  bp,pm=prev;item['prev_mapping_bag']=bp/1e9;item['mapping_step']=math.hypot(pose(mm)[0]-pose(pm)[0],pose(mm)[1]-pose(pm)[1]);item['prev_mapping_pose']=pose(pm)
 j.append(item)
for z in j:
 print('\nODOM jump bag',iso(z['odom_bag']),'header',iso(z['odom_header']),'ds/dt',z['ds'],z['dt'],'before',z['before'],'after',z['after'])
 print(' latest mapping bag',iso(z['mapping_bag']),'age',z['mapping_age_bag'],'header',iso(z['mapping_header']),'pose',z['mapping_pose'],'mapstep',z.get('mapping_step'),'prev',z.get('prev_mapping_pose'))
 print(' vehicle receipt',iso(z['vehicle_bag']),'speed kmh',z['vehicle_speed_kmh'],'wheels',z['wheel_left'],z['wheel_right'])
# Mapping maxima
for label,A in [('mapping',M),('mapping_incremental',MI)]:
 arr=[]
 for (_,a),(_,b) in zip(A,A[1:]):
  d=math.hypot(pose(b)[0]-pose(a)[0],pose(b)[1]-pose(a)[1]);arr.append((d,ts(a.header.stamp),ts(b.header.stamp),pose(a),pose(b)))
 print('\n',label,'top')
 for x in sorted(arr,reverse=True)[:10]:print(x)
