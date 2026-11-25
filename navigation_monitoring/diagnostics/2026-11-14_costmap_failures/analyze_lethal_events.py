#!/usr/bin/env python3
"""Read-only offline audit of all recorded lethal-start events on 2026-11-14."""
from __future__ import annotations
import collections, datetime as dt, importlib.util, json, math, re, sys
from pathlib import Path
import numpy as np

ROOT=Path('/home/akun/workspace/Car_jetson')
OUT=ROOT/'navigation_monitoring/diagnostics/2026-11-14_costmap_failures'
HELPER=OUT/'analyze_first_lethal_raw.py'
spec=importlib.util.spec_from_file_location('lethal_helper',HELPER)
h=importlib.util.module_from_spec(spec);sys.modules[spec.name]=h;spec.loader.exec_module(h)
TZ=dt.timezone(dt.timedelta(hours=8))

def events_from_log(path):
 out=[]
 for line in path.read_text(errors='replace').splitlines():
  if 'Starting point in lethal space' not in line: continue
  m=re.search(r'\[(\d+\.\d+)\]',line)
  if m: out.append(int(round(float(m.group(1))*1e9)))
 return out

def nearest_frame(bag,topic,event_ns,window_ns):
 frames=h.load_costmap_frames(bag,topic,event_ns,window_ns)
 if not frames:return None,[]
 return min(frames,key=lambda g:abs(g.header_ns-event_ns)),frames

def scan_details(bag,topic,event_ns):
 typ,rows=h.topic_rows(bag,topic,event_ns-80_000_000,event_ns+80_000_000)
 if not rows:return None
 bag_ns,blob=min(rows,key=lambda z:abs(z[0]-event_ns));msg=h.decode(typ,blob)
 pts=[]
 for i,v in enumerate(msg.ranges):
  r=float(v)
  if not math.isfinite(r):continue
  a=float(msg.angle_min+i*msg.angle_increment);x=r*math.cos(a);y=r*math.sin(a)
  dx=max(abs(x)-.45,0.0);dy=max(abs(y)-.30,0.0);gap=math.hypot(dx,dy)
  if r<1.2:
   pts.append({'i':i,'r':r,'a':a,'x':x,'y':y,'inside':gap==0,'footprint_gap':gap})
 return {'topic':topic,'bag_ns':bag_ns,'header_ns':h.ns(msg.header.stamp),'event_offset_s':(bag_ns-event_ns)/1e9,
         'frame':msg.header.frame_id,'beams':len(msg.ranges),'finite':sum(math.isfinite(float(v)) for v in msg.ranges),
         'min_range':min((float(v) for v in msg.ranges if math.isfinite(float(v))),default=None),
         'lt_1m':sum(p['r']<1 for p in pts),'lt_1_2m':len(pts),'close_points':pts}

def front_mask_details(bag,event_ns):
 typ,rows=h.topic_rows(bag,'/laser/scan_front',event_ns-80_000_000,event_ns+80_000_000)
 if not rows:return None
 bag_ns,blob=min(rows,key=lambda z:abs(z[0]-event_ns));msg=h.decode(typ,blob)
 candidates=[]
 for i,v in enumerate(msg.ranges):
  r=float(v)
  if not math.isfinite(r) or r<=0:continue
  a=float(msg.angle_min+i*msg.angle_increment)
  deg=math.degrees(a)
  # merger front-lidar transform: +45deg and (+0.43,+0.28)
  x=r*math.cos(a+math.radians(45))+.43
  y=r*math.sin(a+math.radians(45))+.28
  if 132<=deg<=135 and .60<=r<=.83 and -.40<=x<=-.18 and .28<=y<=.33:
   candidates.append({'i':i,'deg':deg,'r':r,'x':x,'y':y})
 return {'bag_ns':bag_ns,'header_ns':h.ns(msg.header.stamp),'event_offset_s':(bag_ns-event_ns)/1e9,
         'frame':msg.header.frame_id,'candidate_count':len(candidates),'candidates':candidates}

def event_analysis(bag,event_ns,static):
 tf,map_pose=h.nearest_tf(bag,event_ns);odom_pose=tf[('odom','base_link')]['pose']
 gg,gframes=nearest_frame(bag,'/global_costmap/costmap_raw',event_ns,1_500_000_000)
 lg,lframes=nearest_frame(bag,'/local_costmap/costmap_raw',event_ns,250_000_000)
 if gg is None:return {'event_ns':event_ns,'event':h.iso(event_ns),'error':'no global costmap frame'}
 smac,abin,cell=h.quantized_smac_pose(gg,map_pose)
 poly=h.transform_polygon(h.FOOTPRINT,smac);bcells,outside=gg.polygon_boundary(poly)
 pairs=collections.Counter();lethal=[]
 for mx,my in bcells:
  gv=int(gg.data[my,mx]);wx,wy=gg.map_to_world(mx,my);sv=static.cost(wx,wy);pairs[(gv,sv)]+=1
  if gv==254:
   dx,dy=wx-smac[0],wy-smac[1]
   f=math.cos(smac[2])*dx+math.sin(smac[2])*dy
   l=-math.sin(smac[2])*dx+math.cos(smac[2])*dy
   lethal.append({'world':[wx,wy],'static':sv,'forward':f,'left':l})
 gs=h.frame_summary(gg,event_ns,smac);ss=h.static_footprint_stats(static,smac)
 ls=h.frame_summary(lg,event_ns,odom_pose) if lg else None
 raw=scan_details(bag,'/laser/scan_merge_laser',event_ns)
 filtered=scan_details(bag,'/laser/scan_merge_laser_filtered',event_ns)
 front=front_mask_details(bag,event_ns)
 plan,_=h.plan_analysis(bag,event_ns,map_pose,smac)
 out={'event_ns':event_ns,'event':h.iso(event_ns),
      'tf_map_pose':list(map_pose),'tf_odom_pose':list(odom_pose),
      'smac_pose':list(smac),'angle_bin':abin,'map_cell':list(cell),
      'global_frame_offset_s':(gg.header_ns-event_ns)/1e9,
      'global_bag_offset_s':(gg.bag_ns-event_ns)/1e9,
      'global_center_cost':gs['center_cost_display'],
      'global_boundary_cost_counts':gs['boundary_cost_counts_display'],
      'global_nearest_lethal':gs['nearest_lethal'],
      'global_boundary_pairs':{f'raw={a},static={b}':n for (a,b),n in sorted(pairs.items())},
      'global_boundary_lethal_cells':lethal,
      'static_center':ss['center_occupancy'],'static_boundary_counts':ss['boundary_occupancy_counts'],
      'static_geometric_collision':ss['geometric_collision_if_unknown_disallowed'],
      'local_frame_offset_s':None if not lg else (lg.header_ns-event_ns)/1e9,
      'local_center_cost':None if not ls else ls['center_cost_display'],
      'local_boundary_cost_counts':None if not ls else ls['boundary_cost_counts_display'],
      'local_nearest_lethal':None if not ls else ls['nearest_lethal'],
      'raw_scan':raw,'filtered_scan':filtered,'front_mask':front,'last_plan':plan}
 # concise cause from the nearest published snapshot
 if gs['center_cost_display'] in (253,254): out['snapshot_cause']='global center inscribed/lethal'
 elif lethal and all(x['static']==100 for x in lethal): out['snapshot_cause']='static occupied cells intersect footprint boundary'
 elif lethal: out['snapshot_cause']='dynamic/non-static lethal cells intersect footprint boundary'
 elif gs['smac_collision_reconstructed']: out['snapshot_cause']='collision reconstructed without boundary lethal classification'
 else: out['snapshot_cause']='nearest 1 Hz snapshot does not reproduce in-memory collision'
 del gframes,lframes
 return out

def analyze(label,run,logpath):
 bag=run/'navigation_core_bag';log=Path(logpath);events=events_from_log(log);static=h.load_static_grid()
 rows=[]
 for i,e in enumerate(events,1):
  print(label,i,len(events),h.iso(e),file=sys.stderr,flush=True)
  rows.append(event_analysis(bag,e,static))
 return {'label':label,'run':str(run),'bag':str(bag),'nav_log':str(log),'event_count':len(events),'events':rows}

def main():
 results=[]
 results.append(analyze('11:13 failed',ROOT/'navigation_monitoring/runs/2026-11-14_11-12-06_manual_publish_monitor',ROOT/'navigation_monitoring/runs/2026-11-14_11-12-06_manual_publish_monitor/nav2_2026-11-14_11-14-11.log'))
 results.append(analyze('11:22 succeeded',ROOT/'navigation_monitoring/runs/2026-11-14_11-02-27_manual_publish_monitor',ROOT/'navigation_monitoring/runs/2026-11-14_11-12-06_manual_publish_monitor/nav2_2026-11-14_11-24-10.log'))
 out={'method':'read-only sqlite rosbag2 deserialization; nearest published global raw snapshot is 1 Hz while planner memory updates at 5 Hz','runs':results}
 (OUT/'lethal_events_analysis.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
 for run in results:
  print('\n###',run['label'])
  for e in run['events']:
   print(e['event'],e['snapshot_cause'],'pose',tuple(round(v,3) for v in e.get('tf_map_pose',[])),
     'gdt',round(e.get('global_frame_offset_s',999),3),'center',e.get('global_center_cost'),
     'b254',len(e.get('global_boundary_lethal_cells',[])),
     'static',e.get('static_boundary_counts'),
     'rawmin',None if not e.get('raw_scan') else round(e['raw_scan']['min_range'],3),
     'filtmin',None if not e.get('filtered_scan') else round(e['filtered_scan']['min_range'],3),
     'mask',None if not e.get('front_mask') else e['front_mask']['candidate_count'])
if __name__=='__main__':main()
