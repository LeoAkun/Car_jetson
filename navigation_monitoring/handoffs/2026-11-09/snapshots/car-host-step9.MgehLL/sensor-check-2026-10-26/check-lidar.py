import argparse
import json
import math
import os
import signal
import subprocess
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

parser = argparse.ArgumentParser()
parser.add_argument('interface')
parser.add_argument('kind', choices=['rslidar', 'scan'])
parser.add_argument('--output-dir', type=Path, default=Path(__file__).parent)
parser.add_argument('--duration', type=float, default=35)
args = parser.parse_args()
out = args.output_dir
out.mkdir(parents=True, exist_ok=True)
if not (out / f'ready-{args.interface}').exists():
    raise SystemExit('Temporary network stage is not ready.')
addresses = subprocess.check_output(['ip', '-4', '-o', 'address', 'show', 'dev', args.interface], text=True)
if '192.168.1.140/24' not in addresses:
    raise SystemExit('Expected temporary IPv4 address is not present.')
rclpy.init()
node = rclpy.create_node('sensor_lidar_health_observer')
samples = defaultdict(list)


def callback(topic, msg):
    sample = {'received': time.monotonic(), 'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
              'frame': msg.header.frame_id}
    if isinstance(msg, LaserScan):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min < r < msg.range_max]
        sample.update(points=len(msg.ranges), valid=len(valid),
                      range_min=min(valid) if valid else None, range_max=max(valid) if valid else None,
                      scan_time=msg.scan_time, angle_increment=msg.angle_increment)
    else:
        sample.update(points=msg.width * msg.height, fields=[f.name for f in msg.fields])
        types = {1: 'i1', 2: 'u1', 3: '<i2', 4: '<u2', 5: '<i4', 6: '<u4', 7: '<f4', 8: '<f8'}
        dtype = np.dtype({'names': [f.name for f in msg.fields],
                          'formats': [types[f.datatype] for f in msg.fields],
                          'offsets': [f.offset for f in msg.fields], 'itemsize': msg.point_step})
        points = np.frombuffer(msg.data, dtype=dtype)
        finite = np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])
        sample['finite_points'] = int(finite.sum())
        if 'ring' in dtype.names and len(points):
            sample['ring_range'] = [int(points['ring'].min()), int(points['ring'].max())]
        if 'time' in dtype.names and len(points):
            sample['point_time_range'] = [float(points['time'].min()), float(points['time'].max())]
    samples[topic].append(sample)


kind = args.kind
topics = ['/rslidar_points', '/rslidar_Pointcloud2'] if kind == 'rslidar' else ['/laser/scan_front', '/laser/scan_back']
subscriptions = [node.create_subscription(PointCloud2 if kind == 'rslidar' else LaserScan, topic,
                                         lambda msg, topic=topic: callback(topic, msg), qos_profile_sensor_data)
                 for topic in topics]
command = ['ros2', 'launch', str(Path(__file__).parent.parent / 'sensor_only.launch.py'), 'confirm_sensor_only:=true',
           'start_rslidar:=true' if kind == 'rslidar' else 'start_scan_lasers:=true']
if kind == 'scan':
    command.extend(['scan_host_ip:=192.168.1.140', 'scan_front_port:="2368"', 'scan_rear_port:="2369"'])
log = (out / f'{args.interface}-{kind}-driver.log').open('w')
p = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    end = time.monotonic() + args.duration
    while time.monotonic() < end and p.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=7)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            try:
                p.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait(timeout=4)
    # A blocked UDP receiver can outlive the launch process after SIGTERM.
    # This process group was created exclusively for this single-sensor test.
    try:
        os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    log.close()
    node.destroy_node()
    rclpy.shutdown()
results = {}
for topic in topics:
    records = samples[topic]
    result = {'message_count': len(records)}
    if records:
        result.update(received_hz=(len(records) - 1) / (records[-1]['received'] - records[0]['received']) if len(records) > 1 else 0,
                      nonmonotonic_stamps=sum(b['stamp'] <= a['stamp'] for a, b in zip(records, records[1:])),
                      zero_stamps=sum(record['stamp'] <= 0 for record in records),
                      point_count_range=[min(record['points'] for record in records), max(record['points'] for record in records)],
                      first=records[0], last=records[-1])
    results[topic] = result
(out / f'{args.interface}-{kind}-result.json').write_text(json.dumps(results, indent=2))
print(json.dumps(results, indent=2))
print((out / f'{args.interface}-{kind}-driver.log').read_text()[-8000:])
