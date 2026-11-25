import json,numpy as np
from pathlib import Path
R=Path('/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-09_14-33-18');D=json.load(open(R/'costmap_deviation_evidence.json'));S=np.load(R/'costmap_deviation_snapshots.npz',allow_pickle=True)
proof=[]
for rec in D['snapshot_nearest_obstacles']:
 key='map_'+str(min([1794206343.,1794206350.,1794206360.,1794206370.,1794206380.,1794206390.,1794206400.,1794206410.,1794206417.],key=lambda x:abs(x-rec['t'])))
 s=S[key][0];xy=np.array(rec['nearest_local_lethal_map_xy']);ij=np.floor((xy-s['origin'])/s['row']['resolution']).astype(int);a=s['data'];val=int(a[ij[1],ij[0]])
 proof.append({'time_local':rec['time'],'time_global':s['row']['time'],'map_xy':xy.tolist(),'local_cost':254,'global_cost':val})
D['same_point_global_local_costs']=proof
ss=D['scan_samples']; cut=1794206413.831
def summary(samples):
 return {'scan_sample_count':len(samples),'global_inrange_sum':sum(x['global_inrange'] for x in samples),'global_height_pass_sum':sum(x['global_height_pass'] for x in samples),'local_inrange_sum':sum(x['local_inrange'] for x in samples),'local_height_pass_sum':sum(x['local_height_pass'] for x in samples),'first_time':samples[0]['time'] if samples else None,'last_time':samples[-1]['time'] if samples else None,'all_scan_samples_max_age':max([x['header_age'] for x in samples],default=None)}
D.pop('mark_filter_summary',None)
D['mark_filter_summary_all_samples']=summary(ss)
D['mark_filter_summary_auto']=summary([x for x in ss if x['t']<cut])
D['mark_filter_summary_manual_early']=summary([x for x in ss if x['t']>=cut])
D['manual_takeover_exact']={'epoch':cut,'time':'2026-11-09T14:40:13.831+08:00','source':'trajectory/VehicleStatus audit; vehicle_status first is_autodrive=false'}
for row in D['costmap_samples']: row['is_before_exact_manual_takeover']=row['t']<cut
(R/'costmap_deviation_evidence.json').write_text(json.dumps(D,ensure_ascii=False,indent=2))
print(json.dumps(proof,indent=2))
