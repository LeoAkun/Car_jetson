import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, NavSatFix, PointCloud2
from tf2_msgs.msg import TFMessage

base = Path('/home/akun/car-host-step9.MgehLL')
out = Path(__file__).resolve().parent
os.environ['ROS_DOMAIN_ID'] = '88'
os.environ['ROS_LOG_DIR'] = str(out/'ros-log')
duration = 40
records = defaultdict(list)
node = None
processes = []
camera_processes = []
logs = []

def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec/1e9

def callback(topic, msg):
    if isinstance(msg,TFMessage):
        for t in msg.transforms:
            records[topic].append({'received':time.monotonic(),
                'stamp':t.header.stamp.sec+t.header.stamp.nanosec/1e9,
                'parent':t.header.frame_id,'child':t.child_frame_id,
                'translation':[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]})
        return
    row = {'received':time.monotonic(),'stamp':stamp(msg),
           'age_seconds':time.time()-stamp(msg)}
    if hasattr(msg,'header'):
        row['frame'] = msg.header.frame_id
    if isinstance(msg, LaserScan):
        valid = [v for v in msg.ranges if math.isfinite(v) and msg.range_min < v < msg.range_max]
        row.update(points=len(msg.ranges), valid=len(valid), scan_time=msg.scan_time)
    elif isinstance(msg, PointCloud2):
        row.update(points=msg.width*msg.height, fields=[f.name for f in msg.fields])
        fieldmap={f.name:f for f in msg.fields}
        if 'ring' in fieldmap and msg.width*msg.height:
            f=fieldmap['ring']
            dtype={1:'i1',2:'u1',3:'<i2',4:'<u2',5:'<i4',6:'<u4',7:'<f4',8:'<f8'}[f.datatype]
            rings=np.ndarray((msg.width*msg.height,),dtype=dtype,buffer=msg.data,
                             offset=f.offset,strides=(msg.point_step,))
            row['ring_range']=[int(rings.min()),int(rings.max())]
    elif isinstance(msg, Image):
        row.update(width=msg.width,height=msg.height,encoding=msg.encoding)
        if msg.encoding == '16UC1' and len(msg.data):
            values=np.frombuffer(msg.data,dtype='<u2' if not msg.is_bigendian else '>u2')
            row['valid_fraction']=float(np.count_nonzero(values)/values.size)
    elif isinstance(msg, CameraInfo):
        row.update(width=msg.width,height=msg.height,fx=msg.k[0],fy=msg.k[4])
    elif isinstance(msg, Imu):
        row['accel_norm']=math.sqrt(msg.linear_acceleration.x**2+msg.linear_acceleration.y**2+msg.linear_acceleration.z**2)
        values=[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w,
                msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,
                msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
        row['finite']=all(math.isfinite(v) for v in values)
        row['quaternion_norm']=math.sqrt(sum(v*v for v in values[:4]))
    elif isinstance(msg, NavSatFix):
        row.update(status=msg.status.status,latitude=msg.latitude,longitude=msg.longitude)
    elif isinstance(msg, TFMessage):
        row['children']=[t.child_frame_id for t in msg.transforms]
    records[topic].append(row)

def start(name, command):
    log=(out/f'{name}.log').open('w')
    logs.append(log)
    p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append((name,p))
    return p

def stop(name,p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM)
            try:
                p.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL)
                p.wait(timeout=4)
    try:
        os.killpg(p.pid,signal.SIGKILL)
    except ProcessLookupError:
        pass
    return p.returncode

rclpy.init()
node=rclpy.create_node('integrated_sensor_health_observer')
types={
    '/imu':Imu,
    '/rslidar_points':PointCloud2,
    '/rslidar_Pointcloud2':PointCloud2,
    '/laser/scan_front':LaserScan,
    '/laser/scan_back':LaserScan,
    '/laser/scan_merge':PointCloud2,
    '/laser/scan_merge_laser':LaserScan,
    '/camera/front_depth_camera/depth/image_rect_raw':Image,
    '/camera/front_depth_camera/depth/camera_info':CameraInfo,
    '/rear_camera/image_raw':Image,
    '/rear_camera/camera_info':CameraInfo,
    '/rear_camera/image_rect':Image,
    '/sensing/gnss/pose_with_covariance':NavSatFix,
    '/tf_static':TFMessage,
}
tf_qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
subscriptions=[node.create_subscription(t,topic,lambda msg,topic=topic:callback(topic,msg),
                                       tf_qos if topic=='/tf_static' else qos_profile_sensor_data)
               for topic,t in types.items()]
try:
    start('integrated-lidar',[
        'ros2','launch',str(base/'sensor_only.launch.py'),
        'confirm_sensor_only:=true','start_imu:=true','imu_reset_on_open:=false',
        'start_rslidar:=true','start_scan_lasers:=true',
        'scan_host_ip:=192.168.1.140','scan_front_port:="2368"','scan_rear_port:="2369"'])
    start('integrated-realsense',['ros2','launch','launch_real','realsense.launch.py'])
    start('integrated-rear-camera',['ros2','launch','charging_dock','rear_camera.launch.py'])
    start('integrated-scan-merge',['ros2','launch','ros2_laser_scan_merger','merge_2_scan.launch.py'])
    start('integrated-gps',['ros2','launch','gnss_converter','gnss_converter.launch.py'])
    for device in ['/dev/video8','/dev/video10','/dev/video12','/dev/video14']:
        log=(out/f'concurrent-{Path(device).name}.log').open('w')
        logs.append(log)
        camera_processes.append(subprocess.Popen(
            ['python3',str(out/'check-concurrent-uvc.py'),device],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    deadline=time.monotonic()+duration
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=0.1)
finally:
    for name,p in reversed(processes):
        rc=stop(name,p)
        print(json.dumps({'stopped':name,'exit':rc}),flush=True)
    for p in camera_processes:
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGKILL)
            p.wait(timeout=2)
    if node is not None:
        node.destroy_node()
        rclpy.shutdown()
    for log in logs:
        log.close()

summary={}
for topic in types:
    rows=records[topic]
    item={'message_count':len(rows)}
    if rows:
        item.update(received_hz=(len(rows)-1)/(rows[-1]['received']-rows[0]['received']) if len(rows)>1 else 0,
                    zero_stamps=sum(r['stamp']<=0 for r in rows),
                    nonmonotonic_stamps=sum(b['stamp']<=a['stamp'] for a,b in zip(rows,rows[1:])),
                    first=rows[0],last=rows[-1])
        if 'points' in rows[0]:
            item['point_count_range']=[min(r['points'] for r in rows),max(r['points'] for r in rows)]
        if 'valid_fraction' in rows[0]:
            item['valid_fraction_min']=min(r['valid_fraction'] for r in rows)
        if 'age_seconds' in rows[0]:
            ages=np.array([r['age_seconds'] for r in rows])
            item['age_seconds_percentiles']=dict(zip(['min','median','p95','max'],
                                                   np.percentile(ages,[0,50,95,100]).tolist()))
            item['max_gap_seconds']=max((b['received']-a['received'] for a,b in zip(rows,rows[1:])),default=0)
        if topic=='/imu':
            item['all_finite']=all(r['finite'] for r in rows)
            item['accel_norm_range']=[min(r['accel_norm'] for r in rows),max(r['accel_norm'] for r in rows)]
            item['quaternion_norm_range']=[min(r['quaternion_norm'] for r in rows),max(r['quaternion_norm'] for r in rows)]
        if topic=='/tf_static':
            item['transforms']={r['child']:{k:v for k,v in r.items() if k not in ['received','stamp']} for r in rows}
            item.pop('nonmonotonic_stamps',None)
    summary[topic]=item
result={'duration_seconds':duration,'topics':summary,
        'launches':{name:{'exit':p.returncode} for name,p in processes},
        'extra_uvc_cameras':[{'device':d,'exit':p.returncode} for d,p in zip(
            ['/dev/video8','/dev/video10','/dev/video12','/dev/video14'],camera_processes)]}
(out/'integrated-result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
