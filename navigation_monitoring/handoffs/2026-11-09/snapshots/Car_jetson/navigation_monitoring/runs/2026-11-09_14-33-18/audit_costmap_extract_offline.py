from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from pathlib import Path
from scipy.spatial.transform import Rotation
import sqlite3,numpy as np,math,json,datetime,bisect,time
ROOT=Path('/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-09_14-33-18')
BAG=ROOT/'navigation_core_bag'; paths=sorted(BAG.glob('*.db3'),key=lambda p:int(p.stem.rsplit('_',1)[1]))
TZ=datetime.timezone(datetime.timedelta(hours=8))
def ts(s): return datetime.datetime.fromtimestamp(s,TZ).isoformat(timespec='milliseconds')
def htime(h): return h.stamp.sec+h.stamp.nanosec/1e9
def qvec(q): return [q.x,q.y,q.z,q.w]
def pv(p): return [p.position.x,p.position.y,p.position.z,*qvec(p.orientation)]
def tfv(tr):
 t=tr.transform.translation;r=tr.transform.rotation
 return [t.x,t.y,t.z,r.x,r.y,r.z,r.w]
classes={}
def msg(typ,data):
 if typ not in classes: classes[typ]=get_message(typ)
 return deserialize_message(data,classes[typ])
feedback=[]; tfs={}; static={}; plans=[]; scans=[]
# all small messages first
names=['/navigate_to_pose/_action/feedback','/tf','/tf_static','/plan','/laser/scan_merge_laser']
for p in paths:
 c=sqlite3.connect('file:'+str(p)+'?mode=ro',uri=True)
 marks=','.join('?' for n in names)
 for name,typ,t,d in c.execute('select t.name,t.type,m.timestamp,m.data from messages m join topics t on t.id=m.topic_id where t.name in ('+marks+') order by m.timestamp',names):
  m=msg(typ,d);rec=t/1e9
  if name.endswith('feedback'):
   f=m.feedback;feedback.append([rec,htime(f.current_pose.header),*pv(f.current_pose.pose),f.distance_remaining,f.number_of_recoveries])
  elif name in ['/tf','/tf_static']:
   for tr in m.transforms:
    key=tr.header.frame_id+'->'+tr.child_frame_id
    if name=='/tf_static': static[key]=tfv(tr)
    else: tfs.setdefault(key,[]).append([htime(tr.header),rec,*tfv(tr)])
  elif name=='/plan': plans.append({'t':rec,'header':htime(m.header),'frame':m.header.frame_id,'poses':[pv(q.pose) for q in m.poses]})
  else:
   r=np.array(m.ranges);mask=np.isfinite(r)&(r>=m.range_min)&(r<=m.range_max)
   scans.append({'t':rec,'header':htime(m.header),'frame':m.header.frame_id,'range_min':m.range_min,'range_max':m.range_max,'angle_min':m.angle_min,'angle_increment':m.angle_increment,'ranges':r,'valid':mask})
 c.close()
 print('loaded small',p.name,flush=True)
feedback.sort();fb=np.array(feedback)
for key in tfs:tfs[key]=np.array(sorted(tfs[key]))
plant=np.array([p['t'] for p in plans]);scant=np.array([p['t'] for p in scans])
def nearest(rows,t,col=0):
 a=rows[:,col];i=int(np.searchsorted(a,t));i=min(i,len(a)-1)
 if i>0 and abs(a[i-1]-t)<abs(a[i]-t):i-=1
 return rows[i]
def trnear(key,t):return nearest(tfs[key],t)[2:]
def pose_near(t,frame):
 if frame=='map': return nearest(fb,t)[2:9]
 return trnear('odom->base_link',t)
def transformpts(points,pose):return Rotation.from_quat(pose[3:]).apply(points)+pose[:3]
def map_to_local(points,t):
 tr=trnear('map->odom',t)
 return Rotation.from_quat(tr[3:]).inv().apply(points-tr[:3])

def metrics(arr,origin,res,pose):
 xy=pose[:2];yaw=Rotation.from_quat(pose[3:]).as_euler('xyz')[2]
 mx,my=np.floor((xy-origin)/res).astype(int); sy,sx=arr.shape
 if not(0<=mx<sx and 0<=my<sy):return {'outside':True}
 r=int(math.ceil(2/res));x0=max(0,mx-r);x1=min(sx,mx+r+1);y0=max(0,my-r);y1=min(sy,my+r+1)
 patch=arr[y0:y1,x0:x1];yy,xx=np.indices(patch.shape)
 dx=origin[0]+(xx+x0+.5)*res-xy[0];dy=origin[1]+(yy+y0+.5)*res-xy[1]
 rr=np.hypot(dx,dy)
 lethal=patch==254; inscribed=patch==253; unknown=patch==255
 bx=np.cos(yaw)*dx+np.sin(yaw)*dy;by=-np.sin(yaw)*dx+np.cos(yaw)*dy
 footprint=(abs(bx)<=.28)&(abs(by)<=.44)
 result={'center_cost':int(arr[my,mx]),'lethal_min_distance':float(rr[lethal].min()) if lethal.any() else None,'lethal_1m':int(np.sum(lethal&(rr<=1))),'lethal_2m':int(np.sum(lethal&(rr<=2))),'unknown_1m':int(np.sum(unknown&(rr<=1))),'footprint_max_cost':int(patch[footprint].max()) if footprint.any() else None,'footprint_lethal_cells':int(np.sum(lethal&footprint)), 'inscribed_min_distance':float(rr[inscribed].min()) if inscribed.any() else None}
 return result
rows=[]; snaps={}; nextsnap=0
cutoff=datetime.datetime(2026,11,9,14,40,18,tzinfo=TZ).timestamp()
start107=datetime.datetime(2026,11,9,14,39,3,tzinfo=TZ).timestamp()
snap_times=[datetime.datetime(2026,11,9,14,m,s,tzinfo=TZ).timestamp() for m,s in [(39,3),(39,10),(39,20),(39,30),(39,40),(39,50),(40,0),(40,10),(40,17)]]
for p in paths:
 c=sqlite3.connect('file:'+str(p)+'?mode=ro',uri=True)
 for name,typ,t,d in c.execute("select t.name,t.type,m.timestamp,m.data from messages m join topics t on t.id=m.topic_id where t.name in ('/global_costmap/costmap_raw','/local_costmap/costmap_raw') and m.timestamp<? order by m.timestamp",(int(cutoff*1e9),)):
  m=msg(typ,d);rt=t/1e9;ht=htime(m.header)
  if name.startswith('/local') and int(rt*5)%5: continue # sample~5Hz local, global all
  arr=np.asarray(m.data,dtype=np.uint8).reshape(m.metadata.size_y,m.metadata.size_x)
  origin=np.array([m.metadata.origin.position.x,m.metadata.origin.position.y]);res=m.metadata.resolution
  pose=pose_near(ht,m.header.frame_id);mt=metrics(arr,origin,res,pose)
  f=nearest(fb,ht)
  row={'t':rt,'time':ts(rt),'header':ht,'frame':m.header.frame_id,'resolution':res,'size':[m.metadata.size_x,m.metadata.size_y],'origin':origin.tolist(),'pose':pose.tolist(),'map_pose':f[2:9].tolist(),'pose_time_delta':float(f[1]-ht),**mt}
  # nearest plan point costs limited near robot 0-3m
  pi=max(0,int(np.searchsorted(plant,ht))-1);plan=plans[pi];pp=np.array(plan['poses'])[:,:3]
  if m.header.frame_id=='odom':pp=map_to_local(pp,ht)
  near=np.linalg.norm(pp[:,:2]-pose[:2],axis=1)<=3
  ij=np.floor((pp[:,:2]-origin)/res).astype(int);ok=(ij[:,0]>=0)&(ij[:,0]<arr.shape[1])&(ij[:,1]>=0)&(ij[:,1]<arr.shape[0])&near
  vals=arr[ij[ok,1],ij[ok,0]]
  row['plan_age']=ht-plan['t'];row['plan_near_count']=int(len(vals));row['plan_near_max_cost']=int(vals.max()) if len(vals) else None;row['plan_near_lethal_count']=int(np.sum(vals>=254))
  rows.append(row)
  if rt>=start107:
   for target in snap_times:
    key=m.header.frame_id+'_'+str(target)
    if key not in snaps or abs(rt-target)<abs(snaps[key]['row']['t']-target):
     # crop radius8m around bot in each nativeframe
     cen=np.floor((pose[:2]-origin)/res).astype(int);radius=int(8/res);x0=max(0,cen[0]-radius);x1=min(arr.shape[1],cen[0]+radius+1);y0=max(0,cen[1]-radius);y1=min(arr.shape[0],cen[1]+radius+1)
     snaps[key]={'row':row,'data':arr[y0:y1,x0:x1].copy(),'origin':origin+np.array([x0,y0])*res,'plan':pp,'target':target}
 c.close();print('costmaps',p.name,len(rows),flush=True)
 if min([x['t'] for x in rows[-5:]])>cutoff:break
# merged scan point z and mark height eligibility, with nearest TF to stamp
scanmetrics=[]
for sc in scans:
 if sc['t']<start107 or sc['t']>cutoff: continue
 if int(sc['t']*5)%5:continue
 angles=sc['angle_min']+np.arange(len(sc['ranges']))*sc['angle_increment'];valid=sc['valid'];r=sc['ranges'][valid]
 points=np.c_[r*np.cos(angles[valid]),r*np.sin(angles[valid]),np.zeros(len(r))]
 # laser_link->base_link constant translation(.0,.0,.6)
 s=static['base_link->laser_link'];points=transformpts(points,np.array(s))
 od=trnear('odom->base_link',sc['header']);mp=trnear('map->odom',sc['header'])
 op=transformpts(points,od);gp=transformpts(op,mp)
 om=transformpts(np.array([[0,0,.6]]),od)[0];gm=transformpts(np.array([om]),mp)[0]
 dist=np.linalg.norm(op-om,axis=1)
 local=(dist>=.1)&(dist<7);global_=(dist>=.1)&(dist<2.5)
 scanmetrics.append({'time':ts(sc['t']),'t':sc['t'],'header_age':sc['t']-sc['header'],'finite_count':int(len(r)),'range_min':float(r.min()) if len(r) else None,'local_inrange':int(local.sum()),'local_height_pass':int(np.sum(local&(op[:,2]>=0)&(op[:,2]<=2))),'global_inrange':int(global_.sum()),'global_height_pass':int(np.sum(global_&(gp[:,2]>=0)&(gp[:,2]<=2))),'local_origin_z':float(om[2]),'global_origin_z':float(gm[2]),'global_point_z_min':float(gp[:,2].min()),'global_point_z_max':float(gp[:,2].max())})
output={'scope':'offline only','cutoff':ts(cutoff),'tf_keys':list(tfs),'static':static,'costmap_samples':rows,'scan_samples':scanmetrics,'feedback_samples_1hz':fb[::30].tolist(),'plan_summary':[{'t':p['t'],'time':ts(p['t']),'start':p['poses'][0],'end':p['poses'][-1],'count':len(p['poses'])} for p in plans]}
(ROOT/'costmap_deviation_evidence.json').write_text(json.dumps(output,ensure_ascii=False,indent=2))
np.savez_compressed(ROOT/'costmap_deviation_snapshots.npz',**{k:np.array([v],dtype=object) for k,v in snaps.items()})
print('DONE',len(rows),len(scanmetrics),len(snaps))
print('107 samples')
for row in rows:
 if row['frame']=='map' and row['t']>=start107:print(row['time'],np.round(row['map_pose'][:3],2),row['center_cost'],row['lethal_min_distance'],row['footprint_lethal_cells'],row['plan_near_max_cost'])
print('scan',scanmetrics[::20])
