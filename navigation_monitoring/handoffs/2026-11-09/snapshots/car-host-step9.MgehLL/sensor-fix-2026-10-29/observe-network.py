import collections
import json
from pathlib import Path
import select
import socket
import time

interfaces = ['eno1', 'enp4s0', 'enp2s0']
sockets = {}
counts = {name: collections.Counter() for name in interfaces}
for name in interfaces:
    sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
    sock.bind((name, 0))
    sock.setblocking(False)
    sockets[sock] = name
try:
    end = time.monotonic() + 6
    while time.monotonic() < end:
        ready, _, _ = select.select(list(sockets), [], [], 0.2)
        for sock in ready:
            packet = sock.recv(65536)
            if len(packet) >= 14:
                mac = ':'.join(f'{x:02x}' for x in packet[6:12])
                counts[sockets[sock]][mac] += 1
finally:
    for sock in sockets:
        sock.close()
result = {name:dict(values) for name,values in counts.items()}
Path('/home/akun/car-host-step9.MgehLL/sensor-fix-2026-10-29/passive-network.json').write_text(
    json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2), flush=True)
expected = {'enp4s0':['08:48:57:08:84:f4'],
            'eno1':['88:c9:b3:90:8d:2d','88:c9:b3:90:f6:c9']}
if not all(counts[name][mac] for name,macs in expected.items() for mac in macs):
    raise SystemExit('Sensor topology differs or a sensor is silent; no network settings changed.')
