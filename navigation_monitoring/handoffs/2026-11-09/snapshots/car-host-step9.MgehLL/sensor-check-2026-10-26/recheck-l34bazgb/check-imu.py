import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

out = Path(__file__).parent
rclpy.init()
node = rclpy.create_node('sensor_recheck_imu_observer')
samples = []


def callback(msg):
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    a = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
    g = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
    samples.append({'received': time.monotonic(),
                    'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                    'frame': msg.header.frame_id,
                    'all_finite': all(math.isfinite(x) for x in q + a + g),
                    'quaternion_norm': math.sqrt(sum(x*x for x in q)),
                    'acceleration_norm': math.sqrt(sum(x*x for x in a))})


sub = node.create_subscription(Imu, '/imu', callback, qos_profile_sensor_data)
log = (out / 'imu-driver.log').open('w')
p = subprocess.Popen([
    'ros2', 'launch', '/home/akun/car-host-step9.MgehLL/sensor_only.launch.py',
    'confirm_sensor_only:=true', 'start_imu:=true', 'imu_reset_on_open:=false'],
    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline and p.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=7)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait(timeout=3)
    node.destroy_node()
    rclpy.shutdown()
    log.close()
result = {'sample_count': len(samples), 'launch_exit': p.returncode}
if samples:
    result.update(received_hz=(len(samples)-1)/(samples[-1]['received']-samples[0]['received']),
                  frames=sorted(set(s['frame'] for s in samples)),
                  all_finite=all(s['all_finite'] for s in samples),
                  zero_stamps=sum(s['stamp'] <= 0 for s in samples),
                  nonmonotonic_stamps=sum(b['stamp'] <= a['stamp'] for a, b in zip(samples, samples[1:])),
                  quaternion_norm_range=[min(s['quaternion_norm'] for s in samples), max(s['quaternion_norm'] for s in samples)],
                  acceleration_norm_range=[min(s['acceleration_norm'] for s in samples), max(s['acceleration_norm'] for s in samples)])
(out / 'imu-result.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
