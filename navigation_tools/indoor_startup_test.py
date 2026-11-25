#!/usr/bin/env python3
"""Bounded live-sensor startup test. Never sends a goal or chassis command."""
import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from zoneinfo import ZoneInfo

import psutil
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster
import yaml

from indoor_startup_observer import IndoorStartupObserver

ROOT = Path(__file__).resolve().parents[1]
LIFECYCLE_NODES = (
    'map_server', 'controller_server', 'smoother_server', 'planner_server',
    'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother',
)
ALLOWED = {
    ('fdilink_ahrs', 'ahrs_driver_node'),
    ('rslidar_sdk', 'rslidar_sdk_node'), ('rslidar_sdk', 'rslidar_convert_node'),
    ('pointcloud_to_laserscan', 'pointcloud_to_laserscan_node'),
    *(('lio_sam', 'lio_sam_' + name) for name in (
        'imuPreintegration', 'imageProjection', 'featureExtraction', 'mapOptimization')),
}


def stamp():
    return datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()


class StartupTest:
    def __init__(self, args):
        if os.environ.get('ROS_DOMAIN_ID') != '88' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Use run_indoor_startup_test.sh (localhost-only domain 88).')
        self.args = args
        run_id = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y-%m-%d_%H-%M-%S')
        self.output = ROOT / 'navigation_monitoring' / 'indoor_startup' / run_id
        self.output.mkdir(parents=True, exist_ok=False)
        os.environ['ROS_LOG_DIR'] = str(self.output / 'ros_logs')
        self.children = []
        self.handles = []
        self.events = open(self.output / 'events.jsonl', 'w', buffering=1)
        self.samples = open(self.output / 'observations.jsonl', 'w', buffering=1)
        self.imu_raw = open(self.output / 'imu_samples.jsonl', 'w', buffering=65536)
        self.phase = 'initializing'
        self.states = {}
        self.pending_states = {}
        self.last_snapshot = 0.0
        self.last_state_query = 0.0
        self.latest_snapshot = None
        self.started_monotonic = time.monotonic()
        self.result = {'started': stamp(), 'output': str(self.output), 'passed': False,
                       'pass_scope': 'startup and final data freshness; sensor timing quality is reported separately',
                       'mode': 'live_imu_live_lidar_no_goal', 'domain': 88,
                       'limitations': ['artificial empty map and map->odom',
                           '3D LiDAR projected to test scan, not front/rear laser chain',
                           'no GPS/relocalization/task/vehicle motion validation',
                           'small map and no RViz differ from production startup load']}
        rclpy.init(args=[])
        self.node = IndoorStartupObserver()
        self.static = StaticTransformBroadcaster(self.node)
        self.state_clients = {name: self.node.create_client(GetState, f'/{name}/get_state')
                              for name in LIFECYCLE_NODES}
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Imu, '/imu', self.record_imu, qos)
        self.emit('test_created', output=str(self.output), domain=88)

    def emit(self, event, **data):
        record = {'time': stamp(), 'event': event, 'phase': self.phase, **data}
        self.events.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
        print(json.dumps(record, ensure_ascii=False), flush=True)

    def set_phase(self, phase):
        self.phase = phase
        self.node.set_phase(phase)
        self.emit('phase_started')

    def record_imu(self, msg):
        record = {'received_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(),
                  'phase': self.phase,
                  'header_ns': msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                  'frame': msg.header.frame_id,
                  'q': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
                  'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                  'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]}
        self.imu_raw.write(json.dumps(record) + '\n')

    def start(self, label, package, executable, ros_args=()):
        if (package, executable) not in ALLOWED:
            raise RuntimeError(f'Executable not allowed: {package}/{executable}')
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
        return self.spawn(label, [str(path), '--ros-args', *ros_args])

    def spawn(self, label, command):
        handle = open(self.output / f'{label}.log', 'w')
        proc = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                start_new_session=True)
        self.handles.append(handle)
        self.children.append((label, proc))
        self.emit('process_started', label=label, pid=proc.pid, command=command)
        return proc

    def pump(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)
        for label, proc in self.children:
            if proc.poll() is not None:
                raise RuntimeError(f'{label} exited with code {proc.returncode}; see {label}.log')
        now = time.monotonic()
        for name, item in list(self.pending_states.items()):
            future, started = item
            if future.done():
                try:
                    state = future.result().current_state
                    self.states[name] = {'id': state.id, 'label': state.label, 'queried_at': now}
                except Exception as exc:
                    self.states[name] = {'id': None, 'error': str(exc), 'queried_at': now}
                del self.pending_states[name]
            elif now - started > 3.0:
                future.cancel()
                self.states[name] = {'id': None, 'label': 'query_timeout', 'queried_at': now}
                del self.pending_states[name]
        if self.phase in ('nav2_start', 'hold') and now - self.last_state_query >= 2.0:
            self.last_state_query = now
            for name, client in self.state_clients.items():
                if name not in self.pending_states and client.service_is_ready():
                    self.pending_states[name] = (client.call_async(GetState.Request()), now)
        if now - self.last_snapshot >= 1.0:
            self.last_snapshot = now
            snap = self.node.snapshot()
            snap['nav2_lifecycle'] = {name: {**state, 'query_age_sec': now-state['queried_at']}
                                      for name, state in self.states.items()}
            snap['time'] = stamp()
            self.latest_snapshot = snap
            self.samples.write(json.dumps(snap, ensure_ascii=False, allow_nan=False) + '\n')

    def wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if predicate():
                self.emit('condition_met', description=description)
                return
        raise TimeoutError(f'{description}: timed out after {timeout}s')

    def duration(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.pump()

    def preflight(self):
        # Do not contend with existing hardware users, even in another ROS domain.
        conflicts = []
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            if proc.pid == os.getpid():
                continue
            try:
                argv = proc.info['cmdline'] or []
                exe = Path(argv[0]).name if argv else ''
                if exe in ('ahrs_driver_node', 'rslidar_sdk_node', 'rslidar_convert_node',
                           'component_container_isolated') or exe.startswith('lio_sam_'):
                    conflicts.append({'pid': proc.pid, 'argv': argv})
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        if conflicts:
            raise RuntimeError(f'Existing sensor/navigation processes must be preserved: {conflicts}')
        self.duration(2.0)
        others = [(name, namespace) for name, namespace in self.node.get_node_names_and_namespaces()
                  if name != self.node.get_name() and not name.startswith('_ros2cli_daemon_')]
        if others:
            raise RuntimeError(f'Test domain already occupied: {others}')
        self.emit('preflight_passed', boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())

    def prepare(self):
        nav_source = ROOT / 'nav2/src/nav2/config/nav2_real_params.yaml'
        lio_source = ROOT / 'slam/src/LIO-SAM/config/params_rslidar.yaml'
        nav = yaml.safe_load(nav_source.read_text())
        nav_dir = Path(get_package_share_directory('nav2'))
        nav['map_server']['ros__parameters']['yaml_filename'] = str(self.output / 'test_map.yaml')
        bt = nav['bt_navigator']['ros__parameters']
        bt['default_nav_to_pose_bt_xml'] = str(nav_dir / 'config/general_nav_to_pose.xml')
        bt['default_nav_through_poses_bt_xml'] = str(nav_dir / 'config/general_nav_through_poses.xml')
        (self.output / 'nav2_params.yaml').write_text(yaml.safe_dump(nav, sort_keys=False))
        lio = yaml.safe_load(lio_source.read_text())
        params = lio['/**']['ros__parameters']
        params.update(savePCD=False, loopClosureEnableFlag=False, use_sim_time=False)
        (self.output / 'lio_params.yaml').write_text(yaml.safe_dump(lio, sort_keys=False))
        # 40 m square artificial free-space map, not a map of the room.
        size = 400
        (self.output / 'test_map.pgm').write_bytes(f'P5\n{size} {size}\n255\n'.encode() + bytes([254]) * size**2)
        (self.output / 'test_map.yaml').write_text(yaml.safe_dump({
            'image': 'test_map.pgm', 'resolution': 0.1, 'origin': [-20., -20., 0.],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}))
        self.result['source_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                        for p in (nav_source, lio_source)}
        transforms = []
        for parent, child, xyz in (
                ('map', 'odom', (0., 0., 0.)),
                ('base_link', 'rslidar', (.35, 0., .51)),
                ('base_link', 'imu_link', (.35, 0., .40))):
            transform = TransformStamped()
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = xyz
            transform.transform.rotation.w = 1.
            transforms.append(transform)
        self.static.sendTransform(transforms)
        self.emit('test_inputs_prepared', artificial=['free_space_map', 'map_to_odom'],
                  live=['imu', '3d_lidar', 'lio_odometry'], lio_overrides=params)

    def run(self):
        self.preflight()
        self.prepare()
        self.set_phase('sensor_baseline')
        imu_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        self.start('imu', 'fdilink_ahrs', 'ahrs_driver_node', [
            '-p', f'serial_port_:={imu_port}', '-p', 'serial_baud_:=921600',
            '-p', 'if_debug_:=false', '-p', 'no_data_reconnect_ms_:=5000',
            '-p', 'reset_on_open_:=true', '-p', 'use_device_timestamp_:=true',
            '-p', 'imu_topic:=/imu', '-p', 'imu_frame_id_:=imu_link', '-p', 'device_type_:=1'])
        lidar_config = Path(get_package_share_directory('rslidar_sdk')) / 'config/config.yaml'
        self.start('lidar', 'rslidar_sdk', 'rslidar_sdk_node', [
            '-r', '__ns:=/rslidar_sdk', '-p', f'config_path:={lidar_config}'])
        self.start('lidar_converter', 'rslidar_sdk', 'rslidar_convert_node')
        self.start('pointcloud_scan', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node', [
            '-r', 'cloud_in:=/rslidar_Pointcloud2', '-r', 'scan:=/laser/scan_merge_laser',
            '-p', 'target_frame:=base_link', '-p', 'transform_tolerance:=0.1',
            '-p', 'min_height:=0.1', '-p', 'max_height:=2.0',
            '-p', 'angle_min:=-3.141592653589793', '-p', 'angle_max:=3.141592653589793',
            '-p', 'angle_increment:=0.008726646259971648', '-p', 'scan_time:=0.1',
            '-p', 'range_min:=0.3', '-p', 'range_max:=30.0', '-p', 'use_inf:=true'])
        self.duration(self.args.baseline)
        self.emit('baseline_complete', snapshot=self.node.snapshot())
        self.require_sensor_inputs()
        self.set_phase('lio_start')
        for suffix in ('imuPreintegration', 'imageProjection', 'featureExtraction', 'mapOptimization'):
            self.start('lio_' + suffix, 'lio_sam', 'lio_sam_' + suffix,
                       ['--params-file', str(self.output / 'lio_params.yaml')])
        self.wait(self.lio_ready, self.args.lio_timeout, 'real LIO ready and odom->base_link fresh')
        self.set_phase('nav2_start')
        self.spawn('nav2', ['/opt/ros/humble/bin/ros2', 'launch',
                   str(ROOT / 'navigation_tools/indoor_nav2.launch.py'),
                   f'params_file:={self.output / "nav2_params.yaml"}'])
        self.wait(self.nav2_ready, self.args.nav2_timeout, 'all eight Nav2 lifecycle nodes active')
        self.emit('nav2_active', states=self.states)
        self.set_phase('hold')
        self.duration(self.args.hold)
        if not self.lio_ready() or not self.nav2_ready():
            raise RuntimeError('LIO/Nav2 did not remain ready through the hold interval')
        self.require_final_observations()
        self.result['passed'] = True

    # Observer schema access is kept here so the recorder remains reusable.
    def require_sensor_inputs(self):
        snap = self.node.snapshot()
        self.emit('sensor_input_check', snapshot=snap)
        for topic in ('/imu', '/rslidar_Pointcloud2'):
            info = snap['topics'][topic]
            if info['count'] < 10 or info['last_arrival_age_sec'] is None or info['last_arrival_age_sec'] > 2.:
                raise RuntimeError(f'Real sensor input missing or stale: {topic}: {info}')

    def lio_ready(self):
        snap = self.latest_snapshot
        if snap is None:
            return False
        age = snap['topics']['/odom']['last_arrival_age_sec']
        return (snap['imu_preintegration']['ready'] is True and
                snap['topics']['/odom']['count'] > 10 and
                age is not None and age < 2. and
                snap['tf']['odom->base_link'].get('fresh') is True)

    def nav2_ready(self):
        now = time.monotonic()
        return all(self.states.get(name, {}).get('id') == 3 and
                   now-self.states[name]['queried_at'] < 5. for name in LIFECYCLE_NODES)

    def require_final_observations(self):
        snap = self.node.snapshot()
        for topic in ('/imu', '/rslidar_Pointcloud2', '/laser/scan_merge_laser',
                      '/lio_sam/mapping/odometry', '/odom', '/map',
                      '/global_costmap/costmap', '/local_costmap/costmap'):
            info = snap['topics'][topic]
            # A static map is intentionally latched and only published once.
            if info['count'] == 0 or (topic != '/map' and
                    (info['last_arrival_age_sec'] is None or info['last_arrival_age_sec'] > 3.)):
                raise RuntimeError(f'Missing/stale final output: {topic}: {info}')
        for topic, info in snap['topics'].items():
            counters = info['counters']
            if counters.get('nonzero_count', 0) or counters.get('active_entries', 0):
                raise RuntimeError(f'Unexpected velocity or active action during idle test: {topic}')
        self.emit('final_observations', snapshot=snap)

    def close(self):
        self.result['ended'] = stamp()
        self.result['duration_sec'] = time.monotonic() - self.started_monotonic
        try:
            self.result['observations'] = self.node.snapshot()
        except Exception as exc:
            self.result['observations_error'] = f'{type(exc).__name__}: {exc}'
        self.result['nav2_lifecycle'] = self.states
        # SIGINT only our process groups; never stop preexisting services.
        cleanup = []
        groups = []
        for label, proc in reversed(self.children):
            try:
                group = os.getpgid(proc.pid)
            except ProcessLookupError:
                group = proc.pid
            groups.append((label, proc, group))
        for label, proc, group in groups:
            def group_alive():
                proc.poll()  # Reap the direct child before checking its group.
                for candidate in psutil.process_iter(['pid', 'status']):
                    try:
                        if candidate.info['status'] == psutil.STATUS_ZOMBIE:
                            continue
                        if os.getpgid(candidate.info['pid']) == group:
                            return True
                    except (ProcessLookupError, PermissionError, psutil.Error):
                        continue
                return False
            for sig, timeout in ((signal.SIGINT, 5.), (signal.SIGTERM, 2.), (signal.SIGKILL, 2.)):
                try:
                    os.killpg(group, sig)
                except ProcessLookupError:
                    continue
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    alive = group_alive()
                    if not alive:
                        break
                    time.sleep(0.05)
                try:
                    proc.wait(timeout=0.1)
                except subprocess.TimeoutExpired:
                    pass
                alive = group_alive()
                if not alive:
                    break
            cleanup.append({'label': label, 'pid': proc.pid, 'returncode': proc.poll()})
        self.result['cleanup'] = cleanup
        self.result['finished_cleanup'] = stamp()
        self.imu_raw.close()
        for handle in self.handles:
            handle.close()
        (self.output / 'result.json').write_text(json.dumps(self.result, ensure_ascii=False, indent=2, allow_nan=False))
        self.emit('test_finished', passed=self.result['passed'], output=str(self.output),
                  error=self.result.get('error'))
        self.samples.close()
        self.events.close()
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=float, default=20.)
    parser.add_argument('--lio-timeout', type=float, default=90.)
    parser.add_argument('--nav2-timeout', type=float, default=180.)
    parser.add_argument('--hold', type=float, default=120.)
    args = parser.parse_args()
    for key, value in vars(args).items():
        if not math.isfinite(value) or value <= 0:
            parser.error(f'{key} must be finite and positive')
    test = StartupTest(args)
    try:
        test.run()
    except (Exception, KeyboardInterrupt) as exc:
        test.result['error'] = f'{type(exc).__name__}: {exc}'
        test.emit('test_error', error=test.result['error'])
    finally:
        test.close()
    return 0 if test.result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
