import datetime
import fcntl
import json
import os
from pathlib import Path
import select
import termios
import time

out = Path(__file__).resolve().parent
ports = [f'/dev/ttyS{i}' for i in [0, 1, 2, 3, 4, 5, 6, 7, 8, 11]]
bauds = [115200, 9600, 38400, 57600, 19200, 4800]
results = {'started': datetime.datetime.now().astimezone().isoformat(),
           'mode': 'receive only; no serial write; restore original termios',
           'attempts': []}

def nmea_valid(line):
    line = line.strip()
    if not line.startswith(b'$') or b'*' not in line:
        return False
    body, checksum = line[1:].rsplit(b'*', 1)
    try:
        expected = int(checksum[:2], 16)
    except ValueError:
        return False
    actual = 0
    for value in body:
        actual ^= value
    return actual == expected

for baud in bauds:
    opened = {}
    data = {}
    attempt = {'baud': baud, 'seconds': 5, 'ports': {}}
    try:
        for port in ports:
            fd = None
            old = None
            try:
                fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                old = termios.tcgetattr(fd)
                attrs = termios.tcgetattr(fd)
                attrs[0] = 0
                attrs[1] = 0
                attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
                attrs[3] = 0
                attrs[4] = attrs[5] = getattr(termios, f'B{baud}')
                attrs[6][termios.VMIN] = 0
                attrs[6][termios.VTIME] = 0
                termios.tcsetattr(fd, termios.TCSANOW, attrs)
                termios.tcflush(fd, termios.TCIFLUSH)
                opened[fd] = (port, old)
                data[port] = bytearray()
            except Exception as exc:
                attempt['ports'][port] = {'error': str(exc)}
                if fd is not None:
                    if old is not None:
                        termios.tcsetattr(fd, termios.TCSANOW, old)
                    os.close(fd)
        end = time.monotonic() + attempt['seconds']
        while opened and time.monotonic() < end:
            ready, _, _ = select.select(list(opened), [], [], min(0.2, max(0, end-time.monotonic())))
            for fd in ready:
                try:
                    chunk = os.read(fd, 65536)
                    if chunk:
                        data[opened[fd][0]].extend(chunk)
                except BlockingIOError:
                    pass
    finally:
        for fd, (port, old) in opened.items():
            try:
                termios.tcsetattr(fd, termios.TCSANOW, old)
            finally:
                os.close(fd)
    for port, buf in data.items():
        raw = bytes(buf)
        valid = [line.strip() for line in raw.splitlines() if nmea_valid(line)]
        types = {}
        for line in valid:
            name = line.split(b',', 1)[0].decode('ascii', errors='replace')
            types[name] = types.get(name, 0) + 1
        entry = {'bytes': len(raw), 'valid_nmea': len(valid), 'nmea_types': types,
                 'printable_fraction': sum(32 <= b < 127 or b in (10,13) for b in raw)/len(raw) if raw else None}
        if raw:
            filename = f'{Path(port).name}-{baud}.bin'
            (out/filename).write_bytes(raw)
            os.chmod(out/filename, 0o600)
            entry['capture'] = filename
        attempt['ports'][port] = entry
    results['attempts'].append(attempt)
    (out/'rs232-result.json').write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(attempt, ensure_ascii=False), flush=True)
    if any(x.get('valid_nmea', 0) >= 3 for x in attempt['ports'].values()):
        break
