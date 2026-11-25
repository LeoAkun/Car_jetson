"""Bounded, localhost-only runtime harness; never launches hardware nodes."""
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

HERE = Path(__file__).resolve().parent
ROOT = Path('/home/akun/workspace/Car_jetson')
PARAMS = ROOT / 'nav2/src/nav2/config/nav2_real_params.yaml'
ALLOWED = {
    ('multi_map_navigation', 'route_planner'),
    ('multi_map_navigation', 'navigation_manager'),
    ('convert_laser', 'convert'),
    ('pointcloud_to_laserscan', 'pointcloud_to_laserscan_node'),
    ('nav2_map_server', 'map_server'), ('nav2_planner', 'planner_server'),
    ('nav2_controller', 'controller_server'), ('nav2_smoother', 'smoother_server'),
    ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_velocity_smoother', 'velocity_smoother'),
    ('nav2_waypoint_follower', 'waypoint_follower'),
    ('re_localization', 'pcl_localization_node'),
    ('lio_sam', 'lio_sam_imuPreintegration'), ('lio_sam', 'lio_sam_imageProjection'),
    ('lio_sam', 'lio_sam_featureExtraction'), ('lio_sam', 'lio_sam_mapOptimization'),
}


class Runtime:
    def __init__(self, label):
        assert os.environ.get('ROS_DOMAIN_ID') == '87'
        assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
        assert os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE') == str(HERE / 'localhost-dds.xml')
        assert 'ROBOT_HARDWARE_COMMISSIONED' not in os.environ
        self.label = label
        self.processes = []
        self.handles = []
        rclpy.init(args=[])
        try:
            self.node = Node('offline_acceptance_' + label.replace('-', '_'))
        except Exception:
            rclpy.shutdown()
            raise

    def start(self, package, executable, args=()):
        assert (package, executable) in ALLOWED, 'Executable is not on the software-only allowlist'
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
        handle = open(HERE / f'{self.label}-{executable}.log', 'w')
        process = subprocess.Popen([str(path), '--ros-args', *args],
                                   stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.handles.append(handle)
        self.processes.append(process)
        return process

    def healthy(self):
        for process in self.processes:
            assert process.poll() is None, f'Child exited {process.args}: {process.returncode}'

    def wait(self, predicate, timeout=10., tick=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.healthy()
            if tick:
                tick()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(f'{self.label}: condition timed out after {timeout}s')

    def pump(self, seconds, tick=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.healthy()
            if tick:
                tick()
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def call(self, client, request, timeout=10., tick=None):
        self.wait(client.service_is_ready, timeout, tick)
        future = client.call_async(request)
        self.wait(future.done, timeout, tick)
        assert future.exception() is None
        return future.result()

    def lifecycle(self, name, transition, tick=None):
        client = self.node.create_client(ChangeState, f'/{name}/change_state')
        request = ChangeState.Request()
        request.transition.id = transition
        result = self.call(client, request, 25., tick)
        assert result.success, f'{name}: transition {transition} failed'
        self.node.destroy_client(client)
        state_client = self.node.create_client(GetState, f'/{name}/get_state')
        state = self.call(state_client, GetState.Request(), tick=tick).current_state
        expected = {1: 2, 3: 3, 4: 2}[transition]
        assert state.id == expected, f'{name}: unexpected lifecycle state {state.label}'
        self.node.destroy_client(state_client)

    def close(self):
        # Signal only process groups created by this harness, never pkill/killall.
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in reversed(self.processes):
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.)
        for handle in self.handles:
            handle.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
