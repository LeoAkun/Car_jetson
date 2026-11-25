import fcntl
import json
import os
from pathlib import Path
import select
import termios
import time

out = Path(__file__).resolve().parent
fd = os.open('/dev/ttyS8', os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
old = termios.tcgetattr(fd)
data = bytearray()
try:
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    end = time.monotonic() + 10
    while time.monotonic() < end:
        ready, _, _ = select.select([fd], [], [], min(.2, max(0, end-time.monotonic())))
        if ready:
            data.extend(os.read(fd, 65536))
finally:
    termios.tcsetattr(fd, termios.TCSANOW, old)
    os.close(fd)

valid, invalid, gga, rmc = 0, 0, [], []
for line in bytes(data).splitlines():
    if not line.startswith(b'$') or b'*' not in line:
        continue
    body, checksum = line[1:].rsplit(b'*', 1)
    actual = 0
    for value in body:
        actual ^= value
    try:
        correct = actual == int(checksum[:2], 16)
    except ValueError:
        correct = False
    if not correct:
        invalid += 1
        continue
    valid += 1
    fields = body.decode('ascii').split(',')
    if fields[0].endswith('GGA'):
        gga.append({'quality':fields[6], 'satellites':fields[7], 'hdop':fields[8]})
    if fields[0].endswith('RMC'):
        rmc.append({'status':fields[2]})
result = {'device':'/dev/ttyS8', 'baud':115200, 'seconds':10, 'bytes':len(data),
          'valid_nmea':valid, 'invalid_nmea':invalid, 'gga':gga, 'rmc':rmc}
(out/'gps-nmea-result.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
