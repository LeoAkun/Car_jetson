"""Match captured real scan to 3b2mus using the installed NDT service.

Only an independently checked transform is sent to the isolated observer.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_init_pose.nav2_init_pose import GpsToMapConverter
from re_localization.srv import ReLocalization
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

ROOT = Path('/home/akun/workspace/Car_jetson')
OUT = Path(__file__).resolve().parent
MAP = ROOT/'multi_map_navigation/src/map/3b2mus/3b2mus_raw.pcd'
assert os.environ['ROS_DOMAIN_ID'] == '89' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
rclpy.init(args=[])
node = rclpy.create_node('static_check_map_match')
cloud = deserialize_message((OUT/'cloud-10s.cdr').read_bytes(), PointCloud2)
metadata = json.loads((OUT/'cloud-10s.json').read_text())
assert 'odom_to_lidar' in metadata, metadata
live = json.loads((OUT/'live-summary.json').read_text())
gps = live['topics']['/sensing/gnss/pose_with_covariance']['last']
assert gps['status'] >= 0
converter = GpsToMapConverter(str(MAP.with_name('3b2mus.csv')))
gps_x, gps_y = converter.gps_to_map(gps['lat'], gps['lon'])
print('GPS map seed', gps_x, gps_y, flush=True)


def xyz(message):
    fields = {f.name: f for f in message.fields}
    values = np.column_stack([np.ndarray((message.width * message.height,), dtype='<f4',
        buffer=message.data, offset=fields[name].offset, strides=(message.point_step,)) for name in ['x', 'y', 'z']])
    radius = np.linalg.norm(values[:, :2], axis=1)
    return values[np.isfinite(values).all(axis=1) & (radius > .5) & (radius < 60.)].astype(float)


def matrix(values):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    result[:3, 3] = values[:3]
    return result


print('Loading bounded map neighborhood for independent overlap check', flush=True)
with MAP.open('rb') as stream:
    while not stream.readline().startswith(b'DATA '):
        pass
    offset = stream.tell()
points = np.memmap(MAP, dtype='<f4', mode='r', offset=offset).reshape(-1, 4)
mask = (np.abs(points[:, 0]-gps_x) < 85.) & (np.abs(points[:, 1]-gps_y) < 85.)
local = np.asarray(points[mask, :3], dtype=float)
local = local[np.isfinite(local).all(axis=1)]
# Deduplicate voxels once; nearest-neighbor distances remain meaningful at .2m.
_, indices = np.unique(np.floor(local / .2).astype(np.int32), axis=0, return_index=True)
local = local[indices]
tree = cKDTree(local)
np.save(OUT/'map-neighborhood.npy', local.astype(np.float32))
source = xyz(cloud)
_, indices = np.unique(np.floor(source / .3).astype(np.int32), axis=0, return_index=True)
source = source[indices]
print('Independent map points', len(local), 'scan points', len(source), flush=True)


def overlap(transform, source_points=source):
    transformed = source_points @ transform[:3, :3].T + transform[:3, 3]
    distances, _ = tree.query(transformed, workers=2)
    return {'points': len(source_points), 'inlier_0_3m': float(np.mean(distances < .3)),
            'inlier_0_5m': float(np.mean(distances < .5)),
            'inlier_1_2m': float(np.mean(distances < 1.2)),
            'distance_p50_p90_m': np.percentile(distances, [50, 90]).tolist(),
            'rmse_in_0_5m': float(np.sqrt(np.mean(distances[distances < .5]**2))) if np.any(distances < .5) else None}


log = (OUT/'ndt-map-match.log').open('w')
executable = Path(get_package_prefix('re_localization'))/'lib/re_localization/pcl_localization_node'
process = subprocess.Popen([str(executable), '--ros-args', '--params-file',
    str(ROOT/'utils/src/re_localization/param/localization_real.yaml'),
    '-p', f'map_path:={MAP}', '-p', 'lidar_topic:=/static_check/frozen_real_scan',
    '-p', 'ndt_num_threads:=4'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
publisher = node.create_publisher(PointCloud2, '/static_check/frozen_real_scan', 10)
accepted_pub = node.create_publisher(TransformStamped, '/static_check/accepted_map_to_odom',
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
last_pub = 0.
start = time.monotonic()


def wait(predicate, timeout):
    global last_pub
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        assert process.poll() is None, process.returncode
        assert time.monotonic()-start < 280., 'Map matching exceeded bounded runtime'
        if time.monotonic()-last_pub > .15:
            publisher.publish(cloud)
            last_pub = time.monotonic()
        rclpy.spin_once(node, timeout_sec=.02)
        if predicate(): return
    raise TimeoutError('NDT service wait timed out')


def call(client, request, timeout=25.):
    wait(client.service_is_ready, 10.)
    future = client.call_async(request)
    wait(future.done, timeout)
    return future.result()


candidates = []
report = {'map': str(MAP), 'gps': gps, 'gps_map_seed': [gps_x, gps_y],
          'scan_metadata': metadata, 'accepted': False, 'candidates': candidates}
try:
    lifecycle = node.create_client(ChangeState, '/pcl_localization/change_state')
    for transition in [1, 3]:
        request = ChangeState.Request()
        request.transition.id = transition
        assert call(lifecycle, request, 65.).success
    service = node.create_client(ReLocalization, '/re_localization')
    # Twelve independent heading seeds around the GPS position. The vehicle is stationary.
    for yaw in range(0, 360, 30):
        request = ReLocalization.Request()
        request.initial_pose.header.frame_id = 'map'
        request.initial_pose.header.stamp = node.get_clock().now().to_msg()
        request.initial_pose.pose.pose.position.x, request.initial_pose.pose.pose.position.y = float(gps_x), float(gps_y)
        q = Rotation.from_euler('z', yaw, degrees=True).as_quat()
        orientation = request.initial_pose.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = q.tolist()
        response = call(service, request, 35.)
        candidate = {'seed_yaw_deg': yaw, 'success': response.success,
                     'service_score': response.fitness_score}
        if response.success:
            p, q = response.pose.pose.pose.position, response.pose.pose.pose.orientation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            transform = matrix(values)
            candidate.update(pose=values, overlap=overlap(transform),
                             rpy_deg=Rotation.from_matrix(transform[:3, :3]).as_euler('xyz', degrees=True).tolist())
        candidates.append(candidate)
        (OUT/'map-match-progress.json').write_text(json.dumps(report, indent=2)+'\n')
        print('Candidate', json.dumps(candidate), flush=True)
    plausible = [c for c in candidates if c['success'] and max(abs(v) for v in c['rpy_deg'][:2]) < 15.]
    best = max(plausible, key=lambda c: (c['overlap']['inlier_0_5m'], c['overlap']['inlier_0_3m']))
    report['best'] = best
    # Do not equate the existing service success flag with localization quality.
    accepted = best['overlap']['inlier_0_5m'] > .80 and best['overlap']['inlier_1_2m'] > .93
    report['accepted'] = accepted
    if accepted:
        map_lidar = matrix(best['pose'])
        map_odom = map_lidar @ np.linalg.inv(matrix(metadata['odom_to_lidar']))
        report['map_to_odom_matrix'] = map_odom.tolist()
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map_odom[:3, 3].tolist()
        q = Rotation.from_matrix(map_odom[:3, :3]).as_quat()
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = q.tolist()
        for _ in range(10):
            transform.header.stamp = node.get_clock().now().to_msg()
            accepted_pub.publish(transform)
            rclpy.spin_once(node, timeout_sec=.1)
        for seconds in [30, 60]:
            sample = OUT/f'cloud-{seconds}s.cdr'
            if not sample.exists(): continue
            message = deserialize_message(sample.read_bytes(), PointCloud2)
            detail = json.loads(sample.with_suffix('.json').read_text())
            if 'odom_to_lidar' in detail:
                report[f'held_out_{seconds}s'] = overlap(map_odom @ matrix(detail['odom_to_lidar']), xyz(message))
        report['accepted_transform_note'] = 'Published full map-lidar * inverse(odom-lidar), without identity placeholder.'
    print('BEST', json.dumps(best), 'ACCEPTED', accepted, flush=True)
except Exception as exc:
    report['error'] = repr(exc)
    print('MAP MATCH ERROR', repr(exc), flush=True)
finally:
    (OUT/'map-match-result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=6.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=4.)
    log.close()
    node.destroy_node()
    rclpy.shutdown()
if not report['accepted']:
    raise SystemExit(1)
