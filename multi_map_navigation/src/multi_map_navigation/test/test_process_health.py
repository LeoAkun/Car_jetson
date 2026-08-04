from types import SimpleNamespace
import threading

import pytest
from std_msgs.msg import String

from multi_map_navigation.navigation_manager import NavigationManager
from multi_map_navigation.process_manager import ProcessManagerNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class FakeLogger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)

    def warning(self, message):
        self.errors.append(message)


class FakeGoalHandle:
    def __init__(self):
        self.cancel_count = 0

    def cancel_goal_async(self):
        self.cancel_count += 1


class FakeTwistPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeFuture:
    def __init__(self, state_id, label):
        self._result = SimpleNamespace(
            current_state=SimpleNamespace(id=state_id, label=label))

    def done(self):
        return True

    def result(self):
        return self._result


class FakeLifecycleClient:
    def __init__(self, state_id, label):
        self.state_id = state_id
        self.label = label

    def service_is_ready(self):
        return True

    def call_async(self, request):
        return FakeFuture(self.state_id, self.label)


@pytest.fixture
def manager():
    instance = ProcessManagerNode.__new__(ProcessManagerNode)
    instance.process_states = {'navigation2': 'HEALTHY'}
    instance.health_failures = {'navigation2': 0}
    instance.fault_reasons = {}
    instance.health_failure_threshold = 3
    instance.fault_pub = FakePublisher()
    instance.get_logger = lambda: FakeLogger()
    return instance


def test_transient_health_failures_are_degraded(manager):
    manager.update_process_health('navigation2', False, 'graph discovery delay')
    manager.update_process_health('navigation2', False, 'graph discovery delay')

    assert manager.process_states['navigation2'] == 'DEGRADED'
    assert manager.fault_pub.messages == []

    manager.update_process_health('navigation2', True, '')
    assert manager.process_states['navigation2'] == 'HEALTHY'
    assert manager.health_failures['navigation2'] == 0


def test_repeated_health_failure_is_published_once(manager):
    for _ in range(3):
        manager.update_process_health('navigation2', False, 'planner_server missing')

    assert manager.process_states['navigation2'] == 'FAILED'
    assert manager.fault_reasons['navigation2'] == 'planner_server missing'
    assert manager.fault_pub.messages == ['navigation2: planner_server missing']

    manager.update_process_health('navigation2', False, 'planner_server missing')
    assert manager.fault_pub.messages == ['navigation2: planner_server missing']


def test_launch_exit_fails_immediately(manager):
    manager.update_process_health(
        'navigation2', False, 'launch process exited with code -6', True)

    assert manager.process_states['navigation2'] == 'FAILED'
    assert manager.fault_pub.messages == [
        'navigation2: launch process exited with code -6']


def test_probe_requires_child_process_and_ros_nodes(manager):
    manager.processes = {
        'navigation2': SimpleNamespace(poll=lambda: None, returncode=None)}
    manager.required_process_patterns = {
        'navigation2': ('component_container_isolated',)}
    manager.required_nodes = {
        'navigation2': ('/planner_server', '/controller_server')}
    manager._running_process_commands = lambda process: [
        '/opt/ros/humble/lib/rclcpp_components/component_container_isolated']
    manager._current_node_names = lambda: {'/planner_server'}

    healthy, reason, immediate = manager.probe_process_health('navigation2')

    assert not healthy
    assert reason == 'missing ROS nodes: /controller_server'
    assert not immediate


def test_lifecycle_nodes_must_be_active(manager):
    manager.lifecycle_query_timeout = 0.1
    manager.lifecycle_clients = {
        '/planner_server': FakeLifecycleClient(3, 'active'),
        '/controller_server': FakeLifecycleClient(2, 'inactive'),
    }

    active, reason = manager._lifecycle_nodes_active()

    assert not active
    assert reason == 'lifecycle nodes not active: /controller_server=inactive'


def test_probe_accepts_complete_healthy_nav2_stack(manager):
    manager.processes = {
        'navigation2': SimpleNamespace(poll=lambda: None, returncode=None)}
    manager.required_process_patterns = {
        'navigation2': ('component_container_isolated',)}
    manager.required_nodes = {
        'navigation2': ('/planner_server', '/controller_server')}
    manager.required_services = {
        'navigation2': ('/navigate_to_pose/_action/send_goal',)}
    manager._running_process_commands = lambda process: [
        '/opt/ros/humble/lib/rclcpp_components/component_container_isolated']
    manager._current_node_names = lambda: {
        '/planner_server', '/controller_server'}
    manager._current_service_names = lambda: {
        '/navigate_to_pose/_action/send_goal'}
    manager._lifecycle_nodes_active = lambda: (True, '')

    healthy, reason, immediate = manager.probe_process_health('navigation2')

    assert healthy
    assert reason == ''
    assert not immediate


def test_confirmed_failure_closes_failed_process_group(manager):
    manager.processes = {'navigation2': object()}
    manager.health_failure_threshold = 1
    manager.probe_process_health = lambda name: (
        False, 'component container missing', False)
    shutdown_calls = []
    manager.shutdown_process = lambda name, preserve_fault=False: shutdown_calls.append(
        (name, preserve_fault)) or True

    manager.health_check_callback()

    assert shutdown_calls == [('navigation2', True)]
    assert manager.fault_pub.messages == [
        'navigation2: component container missing']


def test_navigation_fault_cancels_goal_stops_robot_and_starts_cleanup(monkeypatch):
    navigation = NavigationManager.__new__(NavigationManager)
    navigation.state_lock = threading.RLock()
    navigation.is_navigating = True
    navigation.stack_fault_active = False
    navigation.current_goal_handle = FakeGoalHandle()
    navigation.emergency_cmd_vel_pub = FakeTwistPublisher()
    navigation.get_logger = lambda: FakeLogger()
    robot_states = []
    navigation.publish_robot_state = robot_states.append
    started_threads = []

    class FakeThread:
        def __init__(self, target, args, daemon):
            started_threads.append((target, args, daemon))

        def start(self):
            return None

    monkeypatch.setattr(
        'multi_map_navigation.navigation_manager.threading.Thread', FakeThread)

    fault = String()
    fault.data = 'navigation2: planner_server missing'
    navigation.process_fault_callback(fault)

    assert navigation.stack_fault_active
    assert navigation.current_goal_handle.cancel_count == 1
    assert robot_states == ['fault']
    assert len(navigation.emergency_cmd_vel_pub.messages) == 3
    for message in navigation.emergency_cmd_vel_pub.messages:
        assert message.linear.x == 0.0
        assert message.angular.z == 0.0
    assert started_threads == [(
        navigation._cleanup_failed_stack,
        ('navigation2: planner_server missing',),
        True,
    )]

    navigation.process_fault_callback(fault)
    assert navigation.current_goal_handle.cancel_count == 1
    assert len(navigation.emergency_cmd_vel_pub.messages) == 3
