import collections
import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import termios
import time

import serial
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

out = Path(__file__).parent
port = '/dev/ttyS8'
result = {'started': datetime.datetime.now().astimezone().isoformat(),
          'serial_port': port, 'baudrate': 115200, 'raw_seconds': 20,
          'driver_seconds': 20, 'nmea_types': {}, 'gga': [], 'navsatfix': []}
fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
old = termios.tcgetattr(fd)
os.close(fd)
raw = bytearray()
try:
    with serial.Serial(port, 115200, timeout=0.2, exclusive=True) as receiver:
        end = time.monotonic() + result['raw_seconds']
        while time.monotonic() < end:
            raw.extend(receiver.read(max(1, receiver.in_waiting)))
finally:
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    termios.tcsetattr(fd, termios.TCSANOW, old)
    os.close(fd)
(out/'gps-nmea.bin').write_bytes(raw)
os.chmod(out/'gps-nmea.bin', 0o600)
types = collections.Counter()
valid = invalid = 0
for line in bytes(raw).splitlines():
    line = line.strip()
    if not line.startswith(b'$') or b'*' not in line:
        continue
    body, checksum = line[1:].rsplit(b'*', 1)
    actual = 0
    for b in body:
        actual ^= b
    try:
        good = actual == int(checksum[:2], 16)
    except ValueError:
        good = False
    if not good:
        invalid += 1
        continue
    valid += 1
    fields = body.decode('ascii', errors='replace').split(',')
    types[fields[0]] += 1
    if fields[0] in ('GNGGA', 'GPGGA') and len(fields) >= 10:
        result['gga'].append({'quality': fields[6], 'satellites': fields[7],
                              'hdop': fields[8],
                              'latitude_present': bool(fields[2]),
                              'longitude_present': bool(fields[4])})
result.update(raw_bytes=len(raw), valid_nmea=valid, invalid_nmea=invalid,
              nmea_types=dict(types))
print(json.dumps({k:v for k,v in result.items() if k!='navsatfix'}, ensure_ascii=False), flush=True)

rclpy.init()
node = rclpy.create_node('gps_sensor_check_observer')
def callback(msg):
    result['navsatfix'].append({'status': msg.status.status,
        'frame': msg.header.frame_id,
        'stamp': msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
        'all_finite': all(math.isfinite(x) for x in [msg.latitude,msg.longitude,msg.altitude])})
sub = node.create_subscription(NavSatFix, '/sensing/gnss/pose_with_covariance', callback, qos_profile_sensor_data)
log = (out/'gps-driver.log').open('w')
p = subprocess.Popen(['ros2', 'launch', 'gnss_converter', 'gnss_converter.launch.py',
                      'serial_port:=/dev/ttyS8', 'baudrate:=115200'],
                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    end = time.monotonic()+result['driver_seconds']
    while time.monotonic() < end and p.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=7)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait(timeout=3)
    result['driver_exit'] = p.returncode
    log.close()
    node.destroy_node()
    rclpy.shutdown()
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    termios.tcsetattr(fd, termios.TCSANOW, old)
    os.close(fd)
(out/'gps-result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
print(json.dumps({'driver_exit': result['driver_exit'], 'navsatfix_count':len(result['navsatfix'])}), flush=True)
