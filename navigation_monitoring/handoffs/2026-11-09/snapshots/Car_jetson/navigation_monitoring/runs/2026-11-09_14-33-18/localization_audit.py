#!/usr/bin/python3
"""Offline localization consistency audit; no ROS initialization or publishing."""
import datetime as dt
import json
import math
from pathlib import Path
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix

ROOT = Path(__file__).resolve().parent
CSV = ROOT.parents[2] / 'multi_map_navigation/src/map/3b2mus/3b2mus.csv'
cal = np.loadtxt(CSV, delimiter=',')
anchor = np.radians(cal[0, :2])
def enu(latlon):
    delta = np.radians(np.asarray(latlon)) - anchor
    return np.column_stack([delta[:, 1] * 6378137 * np.cos(anchor[0]), delta[:, 0] * 6378137])
src = np.column_stack([enu(cal[:, :2]), np.ones(len(cal))])
affine, _, rank, singular = np.linalg.lstsq(src, cal[:, 2:4], rcond=None)
residual = np.linalg.norm(src @ affine - cal[:, 2:4], axis=1)
gnss = []
for path in sorted((ROOT/'navigation_core_bag').glob('*.db3')):
    con = sqlite3.connect('file:'+str(path)+'?mode=ro&immutable=1', uri=True)
    topic = con.execute("select id from topics where name='/sensing/gnss/pose_with_covariance'").fetchone()
    if topic:
        for t, raw in con.execute('select timestamp,data from messages where topic_id=? order by timestamp',topic):
            msg = deserialize_message(raw, NavSatFix)
            gnss.append([t*1e-9,msg.latitude,msg.longitude,msg.status.status,msg.position_covariance_type,msg.position_covariance[0],msg.position_covariance[4]])
    con.close()
gnss = np.asarray(sorted(gnss))
xy = np.column_stack([enu(gnss[:,1:3]),np.ones(len(gnss))]) @ affine
arrays = np.load(ROOT/'trajectory_arrays.npz')
fb = arrays['feedback']
idx = np.searchsorted(fb[:,0],gnss[:,0]).clip(0,len(fb)-1)
previous = np.maximum(idx-1,0)
idx = np.where(abs(fb[previous,0]-gnss[:,0])<abs(fb[idx,0]-gnss[:,0]),previous,idx)
delta = xy - fb[idx,2:4]
close = abs(fb[idx,0]-gnss[:,0])<.2
auto107 = close & (gnss[:,0]>=1794206343.990) & (gnss[:,0]<1794206413.831)
tz = dt.timezone(dt.timedelta(hours=8))
def local(t): return dt.datetime.fromtimestamp(float(t),tz).isoformat(timespec='milliseconds')
result = {
 'calibration_rows':len(cal),'fit_rank':int(rank),'affine_enu_to_xy':affine.tolist(),
 'calibration_residual_m':{'median':float(np.median(residual)),'p95':float(np.percentile(residual,95)),'max':float(residual.max())},
 'gnss_count':len(gnss),'gnss_status_counts':{str(int(k)):int(np.sum(gnss[:,3]==k)) for k in np.unique(gnss[:,3])},
 'gnss_covariance_type_counts':{str(int(k)):int(np.sum(gnss[:,4]==k)) for k in np.unique(gnss[:,4])},
 'gnss_107_vs_navigation':{'count':int(auto107.sum()),'median_delta_xy_m':np.median(delta[auto107],axis=0).tolist(),'median_distance_m':float(np.median(np.linalg.norm(delta[auto107],axis=1)))},
 'samples':[]}
for target in [1794206344,1794206350,1794206360,1794206380,1794206400,1794206413]:
 i=int(np.argmin(abs(gnss[:,0]-target)))
 result['samples'].append({'time':local(gnss[i,0]),'gps_map_xy':xy[i].tolist(),'navigation_xy':fb[idx[i],2:4].tolist(),'delta_xy_m':delta[i].tolist(),'status':int(gnss[i,3]),'covariance_xy':gnss[i,5:7].tolist()})
tr = arrays['tf_map__odom'][0]
x,y,z,w = tr[5:9]
rpy = [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(2*(w*y-z*x)),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]
rotation = np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)], [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)], [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
poses = arrays['tf_odom__base_link']
position = poses[np.argmin(abs(poses[:,0]-1794206344)),2:5]
result['constant_map_odom_tilt'] = {
 'rpy_deg':np.degrees(rpy).tolist(), 'translation':tr[2:5].tolist(),
 'odom_base_at_107_start':position.tolist(),
 'z_from_xy_rotation_m':float(rotation[2,:2]@position[:2]),
 'transformed_base_z_m':float((rotation@position)[2]),
 'explanation':'The initial full roll/pitch quaternion is held constant in map->odom. Its tilt projects 133m of odometry travel into about -1.07m of map z. Negative global-frame laser z is not evidence of 1.5m physical descent.'}
result['limits']='GNSS affine calibration has residual error and receiver covariance is only approximate. This comparison cannot establish true vehicle position or prove a LiDAR localization error.'
(ROOT/'localization_evidence.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
