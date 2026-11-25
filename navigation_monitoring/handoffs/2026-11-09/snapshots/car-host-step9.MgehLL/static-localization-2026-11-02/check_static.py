"""Bounded real-sensor/SLAM check. No vehicle commands or control nodes."""
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
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformBroadcaster

ROOT = Path('/home/akun/workspace/Car_jetson')
OUT = Path(__file__).resolve().parent
assert os.environ['ROS_DOMAIN_ID'] == '89'
assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
ALLOWED = {('rslidar_sdk', 'rslidar_sdk_node'), ('rslidar_sdk', 'rslidar_convert_node'),
           ('fdilink_ahrs', 'ahrs_driver_node'), ('gnss_converter', 'gnss_converter_node')}
ALLOWED |= {('lio_sam', 'lio_sam_' + s) for s in
            ['imuPreintegration', 'imageProjection', 'featureExtraction', 'mapOptimization']}
processes, files = [], []
records, tf_records = defaultdict(list), defaultdict(list)
clouds_saved = set()
first_cloud_time = None
last_cloud = None
map_transform = None
start = time.monotonic()
rclpy.init(args=[])
node = rclpy.create_node('stationary_real_sensor_observer')
buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=120.))
listener = TransformListener(buffer, node)
static = StaticTransformBroadcaster(node)
dynamic = TransformBroadcaster(node)
transforms = []
for parent, child, xyz in [('base_link', 'rslidar', [.35, 0., .51]),
                           ('rslidar', 'imu_link', [0., 0., -.11])]:
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id, transform.child_frame_id = parent, child
    transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = xyz
    transform.transform.rotation.w = 1.
    transforms.append(transform)
static.sendTransform(transforms)


def atomic_json(name, value):
    target = OUT / name
    temporary = target.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(target)


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


def pose_values(pose):
    return [pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


def callback(topic, message):
    global last_cloud, first_cloud_time
    row = {'elapsed': time.monotonic() - start, 'stamp': stamp(message),
           'age': node.get_clock().now().nanoseconds / 1e9 - stamp(message),
           'frame': message.header.frame_id}
    if isinstance(message, Imu):
        row.update(q=[message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w],
                   accel=[message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z],
                   gyro=[message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z])
    elif isinstance(message, PointCloud2):
        row['points'] = message.width * message.height
        if topic == '/rslidar_Pointcloud2':
            last_cloud = message
            if first_cloud_time is None:
                first_cloud_time = time.monotonic()
    elif isinstance(message, Odometry):
        row['pose'] = pose_values(message.pose.pose)
        row['child'] = message.child_frame_id
    elif isinstance(message, NavSatFix):
        row.update(status=message.status.status, lat=message.latitude, lon=message.longitude,
                   altitude=message.altitude, covariance=list(message.position_covariance))
    records[topic].append(row)


for topic, cls in [('/imu', Imu), ('/rslidar_Pointcloud2', PointCloud2),
                   ('/lio_sam/mapping/odometry', Odometry), ('/odom', Odometry),
                   ('/lio_sam/deskew/cloud_deskewed', PointCloud2),
                   ('/sensing/gnss/pose_with_covariance', NavSatFix)]:
    node.create_subscription(cls, topic, lambda msg, t=topic: callback(t, msg), qos_profile_sensor_data)


def accept_map_transform(message):
    global map_transform
    assert message.header.frame_id == 'map' and message.child_frame_id == 'odom'
    values = [message.transform.translation.x, message.transform.translation.y,
              message.transform.translation.z, message.transform.rotation.x,
              message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w]
    assert all(math.isfinite(v) for v in values)
    map_transform = message


node.create_subscription(TransformStamped, '/static_check/accepted_map_to_odom', accept_map_transform,
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))


def start_node(package, executable, arguments):
    assert (package, executable) in ALLOWED
    path = Path(get_package_prefix(package)) / 'lib' / package / executable
    log = (OUT / (executable + '.log')).open('w')
    files.append(log)
    process = subprocess.Popen([str(path), '--ros-args', *arguments],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append((executable, process))


def tf_pose(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    return [t.x, t.y, t.z, q.x, q.y, q.z, q.w]


def summary():
    topics = {}
    for topic, rows in records.items():
        elapsed = rows[-1]['elapsed'] - rows[0]['elapsed']
        topics[topic] = {'count': len(rows), 'received_hz': (len(rows) - 1) / elapsed if elapsed else 0.,
                         'zero_stamps': sum(r['stamp'] <= 0. for r in rows),
                         'stamp_regressions': sum(b['stamp'] <= a['stamp'] for a, b in zip(rows, rows[1:])),
                         'max_received_gap': max((b['elapsed'] - a['elapsed'] for a, b in zip(rows, rows[1:])), default=0.),
                         'age_p50_p95_max': np.percentile([r['age'] for r in rows], [50, 95, 100]).tolist(),
                         'first': rows[0], 'last': rows[-1]}
    tf_stats = {}
    for frame, rows in tf_records.items():
        stable = [r for r in rows if r['elapsed'] >= rows[0]['elapsed'] + 15.]
        if len(stable) < 2:
            stable = rows
        values = np.asarray([r['pose'] for r in stable])
        if not len(values): continue
        position = values[:, :3]
        q = values[:, 3:]
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        angle = 2. * np.arccos(np.clip(np.abs(q @ q[0]), 0., 1.))
        steps = np.linalg.norm(np.diff(position, axis=0), axis=1)
        tf_stats[frame] = {'samples': len(rows), 'stable_samples': len(stable),
                           'stable_duration': stable[-1]['elapsed'] - stable[0]['elapsed'],
                           'xyz_peak_to_peak_m': np.ptp(position, axis=0).tolist(),
                           'max_displacement_from_stable_start_m': float(np.max(np.linalg.norm(position-position[0], axis=1))),
                           'end_displacement_m': float(np.linalg.norm(position[-1]-position[0])),
                           'max_orientation_change_deg': float(np.rad2deg(np.max(angle))),
                           'max_position_step_m': float(np.max(steps)) if len(steps) else 0.,
                           'first': stable[0], 'last': stable[-1]}
    imu_rows = records['/imu']
    imu = {}
    if imu_rows:
        acceleration = np.asarray([r['accel'] for r in imu_rows])
        gyro = np.asarray([r['gyro'] for r in imu_rows])
        quat = np.asarray([r['q'] for r in imu_rows])
        imu = {'accel_mean': acceleration.mean(axis=0).tolist(), 'accel_std': acceleration.std(axis=0).tolist(),
               'gyro_mean_rad_s': gyro.mean(axis=0).tolist(), 'gyro_std_rad_s': gyro.std(axis=0).tolist(),
               'accel_norm_min_median_max': np.percentile(np.linalg.norm(acceleration, axis=1), [0, 50, 100]).tolist(),
               'all_finite': bool(np.isfinite(acceleration).all() and np.isfinite(gyro).all() and np.isfinite(quat).all())}
    return {'elapsed': time.monotonic()-start, 'topics': topics, 'tf': tf_stats, 'imu': imu,
            'map_transform_received': map_transform is not None,
            'processes': {name: p.poll() for name, p in processes}}


failure = None
try:
    start_node('rslidar_sdk', 'rslidar_sdk_node', ['-p', 'config_path:=' + str(Path(get_package_share_directory('rslidar_sdk'))/'config/config.yaml')])
    start_node('rslidar_sdk', 'rslidar_convert_node', [])
    start_node('fdilink_ahrs', 'ahrs_driver_node', ['-r', '__node:=static_check_imu',
        '-p', 'serial_port_:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        '-p', 'serial_baud_:=921600', '-p', 'reset_on_open_:=false', '-p', 'no_data_reconnect_ms_:=5000',
        '-p', 'imu_topic:=/imu', '-p', 'imu_frame_id_:=imu_link', '-p', 'device_type_:=1'])
    start_node('gnss_converter', 'gnss_converter_node', ['-p', 'serial_port:=/dev/ttyS8', '-p', 'baudrate:=115200'])
    params = ROOT/'slam/src/LIO-SAM/config/params_rslidar.yaml'
    for suffix in ['imuPreintegration', 'imageProjection', 'featureExtraction', 'mapOptimization']:
        start_node('lio_sam', 'lio_sam_'+suffix, ['--params-file', str(params),
            '-p', 'savePCD:=false', '-p', 'loopClosureEnableFlag:=false',
            '-p', 'gpsTopic:=/static_check/no_gps', '-p', 'numberOfCores:=2'])
    last_report = last_tf = last_map = 0.
    while time.monotonic() - start < 420. and not (OUT/'STOP').exists():
        rclpy.spin_once(node, timeout_sec=.01)
        now = time.monotonic()
        for name, process in processes:
            if process.poll() is not None:
                raise RuntimeError(f'{name} exited {process.returncode}')
        if map_transform is not None and now - last_map > .1:
            map_transform.header.stamp = node.get_clock().now().to_msg()
            dynamic.sendTransform(map_transform)
            last_map = now
        if now - last_tf > .05:
            last_tf = now
            for parent in ['odom', 'map']:
                try:
                    transform = buffer.lookup_transform(parent, 'base_link', rclpy.time.Time())
                except Exception:
                    continue
                tf_records[parent+'->base_link'].append({'elapsed': now-start, 'stamp': stamp(transform), 'pose': tf_pose(transform)})
        if last_cloud is not None and first_cloud_time is not None:
            for seconds in [10, 30, 60]:
                if seconds in clouds_saved or now-first_cloud_time < seconds: continue
                (OUT/f'cloud-{seconds}s.cdr').write_bytes(serialize_message(last_cloud))
                metadata = {'frame': last_cloud.header.frame_id, 'stamp': stamp(last_cloud),
                            'point_count': last_cloud.width*last_cloud.height}
                try:
                    transform = buffer.lookup_transform('odom', 'rslidar', rclpy.time.Time.from_msg(last_cloud.header.stamp))
                    metadata['odom_to_lidar'] = tf_pose(transform)
                except Exception as exc:
                    metadata['tf_error'] = str(exc)
                atomic_json(f'cloud-{seconds}s.json', metadata)
                clouds_saved.add(seconds)
        if now-last_report > 5.:
            last_report = now
            temperatures = [int(p.read_text()) for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')]
            if max(temperatures, default=0) > 85000:
                raise RuntimeError('Temperature exceeded 85C')
            state = summary()
            state['temperatures_mC'] = temperatures
            atomic_json('live-summary.json', state)
            print(json.dumps({'elapsed': round(now-start, 1), 'counts': {t: len(r) for t, r in records.items()},
                              'tf': state['tf'], 'gps': records['/sensing/gnss/pose_with_covariance'][-1:]}, ensure_ascii=False), flush=True)
except Exception as exc:
    failure = repr(exc)
    print('CHECK FAILED:', failure, flush=True)
finally:
    for name, process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for name, process in reversed(processes):
        try: process.wait(timeout=6.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try: process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.)
    final = summary()
    final['failure'] = failure
    atomic_json('result.json', final)
    atomic_json('tf-records.json', dict(tf_records))
    atomic_json('sensor-records.json', dict(records))
    for handle in files: handle.close()
    node.destroy_node()
    rclpy.shutdown()
    print('STOPPED', json.dumps(final['processes']), flush=True)
if failure:
    raise SystemExit(1)
