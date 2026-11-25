"""Exercise the installed driver with synthetic CAN over loopback only."""

import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from yunle_msgs.msg import VehicleStatus


def main():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert 100 <= int(os.environ['ROS_DOMAIN_ID']) <= 232
    root = Path(__file__).resolve().parents[3]
    binary = root / 'real_ws/install/chassis_driver/lib/chassis_driver/chassis_driver_node'
    rclpy.init()
    node = Node('loopback_wheel_feedback_probe')
    received = []
    subscription = node.create_subscription(VehicleStatus, '/vehicle_status', received.append, 10)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    checks = {}

    def send(identifier, payload):
        sock.sendto(b'\x08' + identifier.to_bytes(4, 'big') + payload, ('127.0.0.1', 18002))

    def poll(condition, packets, timeout=2.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for identifier, payload in packets:
                send(identifier, payload)
            rclpy.spin_once(node, timeout_sec=.01)
            if received and condition(received[-1]):
                return received[-1]
        raise AssertionError('loopback driver condition timed out')

    status = (0x51, b'\x23' + b'\0' * 7)
    front = (0x168, struct.pack('<hhhh', -20, -22, -20, -22))
    diagnostic = [(0x281, struct.pack('<i', 3000) + b'\0' * 4),
                  (0x282, struct.pack('<i', -3000) + b'\0' * 4)]
    scale = .1213 * 2 * math.pi / 60 / 10

    try:
        for strict in (True, False):
            received.clear()
            command = [str(binary), '--ros-args', '-r', '__node:=loopback_chassis_probe']
            for key, value in {
                'canbus_type': '1', 'can_eth_card': 'lo', 'local_ip': '127.0.0.1',
                'local_port': '18002', 'can1_remote_ip': '127.0.0.1',
                'can2_remote_ip': '127.0.0.1', 'can1_remote_port': '14001',
                'can2_remote_port': '14002', 'car_type': 'JD03',
                'use_front_wheel_feedback': str(strict).lower(),
                'front_wheel_feedback_timeout': '0.05',
            }.items():
                command.extend(['-p', key + ':=' + value])
            log = Path(__file__).with_name('loopback_driver_' + str(strict).lower() + '.log')
            with log.open('w') as output:
                process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
                try:
                    if strict:
                        poll(lambda m: math.isnan(m.left_wheel_speed), [status])
                        checks['missing_wheels_are_invalid'] = True
                        poll(lambda m: abs(m.left_wheel_speed + 20 * scale) < 1e-6
                             and abs(m.right_wheel_speed + 22 * scale) < 1e-6,
                             [front, status])
                        checks['front_wheels_decode_signed_mps'] = True
                        # Every cycle includes conflicting diagnostic frames.
                        front_second = (0x168, struct.pack('<hhhh', -40, -42, -40, -42))
                        poll(lambda m: abs(m.left_wheel_speed + 40 * scale) < 1e-6
                             and abs(m.right_wheel_speed + 42 * scale) < 1e-6,
                             [front_second, *diagnostic, status])
                        checks['diagnostics_cannot_overwrite_front_source'] = True
                        poll(lambda m: math.isnan(m.left_wheel_speed)
                             and math.isnan(m.right_wheel_speed), [status])
                        checks['fresh_status_cannot_refresh_stale_wheels'] = True
                    else:
                        poll(lambda m: math.isfinite(m.left_wheel_speed)
                             and abs(m.left_wheel_speed + 3000 * scale) < 1e-5,
                             [*diagnostic, status])
                        checks['default_legacy_source_unchanged'] = True
                finally:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
        Path(__file__).with_name('driver_feedback_checks.json').write_text(
            json.dumps(checks, indent=2))
        print(json.dumps(checks, indent=2))
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
