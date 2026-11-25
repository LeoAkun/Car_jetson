import json
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

out = Path(__file__).parent
rclpy.init()
node = rclpy.create_node('sensor_depth_health_observer')
received = []
info = []


def callback(msg):
    pixels = np.frombuffer(msg.data, dtype='<u2' if not msg.is_bigendian else '>u2')
    valid = pixels[pixels > 0]
    received.append({
        'received': time.monotonic(),
        'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        'frame': msg.header.frame_id,
        'width': msg.width, 'height': msg.height, 'encoding': msg.encoding,
        'valid_fraction': len(valid) / len(pixels),
        'median_depth_raw': float(np.median(valid)) if len(valid) else None,
    })


sub = node.create_subscription(
    Image, '/camera/front_depth_camera/depth/image_rect_raw', callback,
    qos_profile_sensor_data)
camera_info = node.create_subscription(
    CameraInfo, '/camera/front_depth_camera/depth/camera_info',
    lambda msg: info.append({'width': msg.width, 'height': msg.height, 'fx': msg.k[0], 'fy': msg.k[4]}),
    qos_profile_sensor_data)
log = (out / 'realsense-driver.log').open('w')
process = subprocess.Popen(
    ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
     'camera_namespace:=camera', 'camera_name:=front_depth_camera',
     "serial_no:='117322074703'", 'enable_color:=false', 'enable_depth:=true',
     'enable_infra1:=false', 'enable_infra2:=false',
     'enable_gyro:=false', 'enable_accel:=false',
     'depth_module.depth_profile:=848x480x30', 'depth_module.depth_format:=Z16',
     'pointcloud.enable:=false', 'align_depth.enable:=false', 'initial_reset:=true'],
    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline and process.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=7)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
    log.close()
    node.destroy_node()
    rclpy.shutdown()
result = {'frame_count': len(received), 'camera_info_count': len(info), 'exit_code': process.returncode}
if received:
    result.update(
        received_hz=(len(received) - 1) / (received[-1]['received'] - received[0]['received']) if len(received) > 1 else 0,
        nonmonotonic_stamps=sum(b['stamp'] <= a['stamp'] for a, b in zip(received, received[1:])),
        valid_fraction_min=min(s['valid_fraction'] for s in received),
        first=received[0], last=received[-1],
        camera_info=info[-1] if info else None)
(out / 'realsense-result.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
print((out / 'realsense-driver.log').read_text()[-8000:])
