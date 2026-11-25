import json
import os
import signal
import subprocess
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

out = Path(__file__).parent
rclpy.init()
node = rclpy.create_node('rear_camera_health_observer')
samples = defaultdict(list)
subscriptions = []
for topic, msg_type in [('/rear_camera/image_raw', Image), ('/rear_camera/image_rect', Image), ('/rear_camera/camera_info', CameraInfo)]:
    def callback(msg, topic=topic):
        samples[topic].append({'received': time.monotonic(), 'width': msg.width, 'height': msg.height,
                               'frame': msg.header.frame_id})
    subscriptions.append(node.create_subscription(msg_type, topic, callback, qos_profile_sensor_data))
log = (out / 'rear-camera-driver.log').open('w')
p = subprocess.Popen(['ros2', 'launch', 'charging_dock', 'rear_camera.launch.py', 'video_device:=/dev/video0'],
                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    end = time.monotonic() + 15
    while time.monotonic() < end and p.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=7)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            p.wait(timeout=4)
    log.close()
    node.destroy_node()
    rclpy.shutdown()
result = {}
for topic, messages in samples.items():
    result[topic] = {'message_count': len(messages), 'last': messages[-1],
                     'received_hz': (len(messages) - 1) / (messages[-1]['received'] - messages[0]['received']) if len(messages) > 1 else 0}
(out / 'rear-camera-result.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
print((out / 'rear-camera-driver.log').read_text()[-7000:])
