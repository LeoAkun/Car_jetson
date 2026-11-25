#!/usr/bin/env python3
"""Open a door lock and recover its dedicated USB serial device on failure."""

import argparse
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from lock_controller.srv import QueryLockStatus, UnlockCommand


DEFAULT_DEVICE = (
    '/dev/serial/by-path/'
    'pci-0000:00:14.0-usb-0:1.4:1.0-port0'
)
DEFAULT_USB_DEVICE = '1-1.4'
DEFAULT_USB_INTERFACE = '1-1.4:1.0'


def log(message):
    print(f'[door-recovery] {message}', flush=True)


def wait_for_path(path, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if Path(path).exists():
            return True
        time.sleep(0.2)
    return Path(path).exists()


def lock_controller_pids():
    pids = []
    own_pid = os.getpid()
    for proc_dir in Path('/proc').glob('[0-9]*'):
        try:
            pid = int(proc_dir.name)
            command = (proc_dir / 'cmdline').read_bytes().replace(b'\0', b' ')
        except (OSError, ValueError):
            continue
        if pid != own_pid and b'lock_controller_node' in command:
            pids.append(pid)
    return pids


def stop_lock_controller():
    pids = lock_controller_pids()
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        remaining = [pid for pid in pids if Path(f'/proc/{pid}').exists()]
        if not remaining:
            return
        time.sleep(0.1)

    for pid in remaining:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def start_lock_controller(device, board_address):
    ros2 = shutil.which('ros2')
    if not ros2:
        raise RuntimeError('ros2 was not found in PATH')

    log_path = Path.home() / '.ros' / 'log' / 'lock_controller_recovery.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        ros2,
        'run',
        'lock_controller',
        'lock_controller_node',
        '--ros-args',
        '-p',
        f'serial_port:={device}',
        '-p',
        'baud_rate:=9600',
        '-p',
        f'board_address:={board_address}',
    ]

    with log_path.open('ab') as output:
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            close_fds=True,
        )
    log(f'Started lock controller PID {process.pid} on {device}')


def privileged_write(path, value):
    path = Path(path)
    if os.geteuid() == 0:
        path.write_text(value, encoding='ascii')
        return True

    sudo = shutil.which('sudo')
    if not sudo:
        log(f'Root permission is required to write {path}')
        return False

    command = [sudo]
    if not sys.stdin.isatty():
        command.append('-n')
    command.extend([
        'sh', '-c', 'printf %s "$1" > "$2"',
        'door-recovery', value, str(path),
    ])
    result = subprocess.run(command, check=False)
    if result.returncode != 0:
        log('USB recovery needs sudo permission; run this command in a terminal')
        return False
    return True


def rebind_usb(usb_device, usb_interface):
    device_path = Path('/sys/bus/usb/devices') / usb_device
    interface_path = Path('/sys/bus/usb/devices') / usb_interface
    driver_path = Path('/sys/bus/usb/drivers/ch341')

    if not device_path.exists():
        log(
            f'USB device {usb_device} is absent from sysfs; '
            'physical reconnection or per-port hub power control is required'
        )
        return False

    stop_lock_controller()
    if interface_path.exists() and (interface_path / 'driver').exists():
        log(f'Rebinding only the door-lock USB interface {usb_interface}')
        if not privileged_write(driver_path / 'unbind', usb_interface):
            return False
        time.sleep(0.5)
        return privileged_write(driver_path / 'bind', usb_interface)

    if interface_path.exists():
        log(f'Binding door-lock USB interface {usb_interface}')
        return privileged_write(driver_path / 'bind', usb_interface)

    log(f'Reauthorizing only the door-lock USB device {usb_device}')
    if not privileged_write(device_path / 'authorized', '0'):
        return False
    time.sleep(0.5)
    return privileged_write(device_path / 'authorized', '1')


class DoorClient(Node):
    def __init__(self, timeout_sec):
        super().__init__('open_door_recovery')
        self.timeout_sec = timeout_sec
        self.unlock_client = self.create_client(UnlockCommand, '/unlock')
        self.query_client = self.create_client(QueryLockStatus, '/query_status')

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=self.timeout_sec):
            return None

        future = client.call_async(request)
        deadline = time.monotonic() + self.timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            future.cancel()
            return None
        try:
            return future.result()
        except Exception as error:  # rclpy transports service errors here
            log(f'Service call failed: {error}')
            return None

    def unlock_and_confirm(self, board_address, lock_address):
        unlock_request = UnlockCommand.Request()
        unlock_request.board_address = board_address
        unlock_request.lock_address = lock_address
        unlock_response = self.call(self.unlock_client, unlock_request)
        if unlock_response is None or not unlock_response.success:
            message = getattr(unlock_response, 'message', 'service unavailable')
            log(f'Unlock attempt failed: {message}')
            return False

        time.sleep(0.2)
        query_request = QueryLockStatus.Request()
        query_request.board_address = board_address
        query_request.lock_address = lock_address
        query_response = self.call(self.query_client, query_request)
        if query_response is None or not query_response.success:
            message = getattr(query_response, 'message', 'service unavailable')
            log(f'Unlock was not confirmed: {message}')
            return False
        if query_response.is_locked:
            log('Controller replied, but the lock is still closed')
            return False

        log('Door lock is open')
        return True


def restart_and_unlock(client, args):
    if not wait_for_path(args.device, 1.0):
        return False
    stop_lock_controller()
    start_lock_controller(args.device, args.board)
    time.sleep(1.5)
    return client.unlock_and_confirm(args.board, args.lock)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Open the door and recover its USB serial controller on failure.'
    )
    parser.add_argument('--board', type=int, default=1)
    parser.add_argument('--lock', type=int, default=1)
    parser.add_argument('--device', default=DEFAULT_DEVICE)
    parser.add_argument('--usb-device', default=DEFAULT_USB_DEVICE)
    parser.add_argument('--usb-interface', default=DEFAULT_USB_INTERFACE)
    parser.add_argument('--timeout', type=float, default=2.0)
    parser.add_argument(
        '--no-usb-reset', action='store_true',
        help='Do not rebind the USB interface if restarting the node fails.',
    )
    args = parser.parse_args()
    if not 1 <= args.board <= 32:
        parser.error('--board must be between 1 and 32')
    if not 1 <= args.lock <= 24:
        parser.error('--lock must be between 1 and 24')
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    return args


def main():
    args = parse_args()
    rclpy.init()
    client = DoorClient(args.timeout)
    try:
        log('Trying the existing door-lock service')
        if client.unlock_and_confirm(args.board, args.lock):
            return 0

        log('Restarting the door-lock node on its stable USB path')
        if restart_and_unlock(client, args):
            return 0

        if args.no_usb_reset:
            log('USB reset is disabled; door remains closed')
            return 1

        log('The node restart did not recover the lock; checking USB binding')
        if not rebind_usb(args.usb_device, args.usb_interface):
            return 1
        if not wait_for_path(args.device, 8.0):
            log(f'Door-lock serial path did not return: {args.device}')
            return 1
        if restart_and_unlock(client, args):
            return 0

        log('Recovery completed, but the lock still did not open')
        return 1
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
