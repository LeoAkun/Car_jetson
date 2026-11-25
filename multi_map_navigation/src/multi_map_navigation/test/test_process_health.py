from types import SimpleNamespace
import threading

import pytest
from std_msgs.msg import String
from multi_map_navigation_msgs.msg import LioCorrectionEvent

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

    def info(self, _message):
        return None


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
    instance.confirmed_healthy_processes = {'navigation2'}
    instance.health_failure_threshold = 3
    instance.liosam_ready = False
    instance.liosam_ever_ready = False
    instance.liosam_not_ready_since = None
    instance.liosam_recovery_grace_sec = 5.0
    instance.liosam_stability_ready = True
    instance.liosam_stable_correction_count = 10
    instance.liosam_stable_correction_count_required = 10
    instance.liosam_stable_translation_m = 0.05
    instance.liosam_stable_yaw_rad = 0.017453292519943295
    instance.liosam_last_correction_epoch = 0
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


def test_nav2_waits_for_liosam_scan_stability(manager):
    manager.processes = {
        'navigation2': SimpleNamespace(poll=lambda: None, returncode=None),
        'liosam': SimpleNamespace(poll=lambda: None, returncode=None),
    }
    manager.required_process_patterns = {'navigation2': ()}
    manager.required_nodes = {'navigation2': ()}
    manager.required_services = {}
    manager.lifecycle_clients = {}
    manager.process_dependencies = {}
    manager._running_process_commands = lambda _process: []
    manager.liosam_ready = True
    manager.liosam_stability_ready = False

    healthy, reason, immediate = manager.probe_process_health('navigation2')

    assert not healthy
    assert reason == 'waiting for LIO-SAM stable scan corrections'
    assert not immediate


def test_liosam_requires_preintegration_readiness(manager):
    manager.processes = {
        'liosam': SimpleNamespace(poll=lambda: None, returncode=None)}
    manager.required_process_patterns = {
        'liosam': ('lio_sam_imuPreintegration',)}
    manager.required_nodes = {
        'liosam': ('/lio_sam_imuPreintegration',)}
    manager.required_services = {}
    manager._running_process_commands = lambda _process: [
        '/opt/lio_sam/lib/lio_sam/lio_sam_imuPreintegration']
    manager._current_node_names = lambda: {'/lio_sam_imuPreintegration'}
    manager.liosam_ready = False

    healthy, reason, immediate = manager.probe_process_health('liosam')

    assert not healthy
    assert reason == 'IMU preintegration is not ready'
    assert not immediate

    manager.liosam_stability_ready = False
    manager.liosam_stable_correction_count = 9
    manager.liosam_ready = True
    healthy, reason, immediate = manager.probe_process_health('liosam')
    assert not healthy
    assert reason == 'waiting for LIO-SAM stable scan corrections (9/10)'
    assert not immediate

    manager.liosam_stability_ready = True
    manager.liosam_ready = True
    healthy, reason, immediate = manager.probe_process_health('liosam')
    assert healthy
    assert reason == ''
    assert not immediate


def test_liosam_stability_requires_ten_consecutive_small_scan_corrections(
        manager):
    manager.processes = {'liosam': object()}
    manager.process_states = {'liosam': 'STARTING'}
    manager.process_instance_ids = {'liosam': 'lio-current'}
    manager.process_generations = {'liosam': 12}
    manager.liosam_stability_ready = False
    manager.liosam_stable_correction_count = 0
    manager.liosam_last_correction_epoch = 0

    def correction(epoch, translation=0.01, accepted=True, degenerate=False):
        message = LioCorrectionEvent()
        message.instance_id = 'lio-current'
        message.generation = 12
        message.correction_epoch = epoch
        message.source = message.SOURCE_SCAN_TO_MAP
        message.translation_residual = translation
        message.yaw_residual = 0.0
        message.accepted = accepted
        message.degenerate = degenerate
        return message

    for epoch in range(1, 10):
        manager.liosam_correction_callback(correction(epoch))
    assert manager.liosam_stable_correction_count == 9
    assert not manager.liosam_stability_ready

    manager.liosam_correction_callback(correction(10))
    assert manager.liosam_stable_correction_count == 10
    assert manager.liosam_stability_ready


def test_unreliable_scan_correction_resets_startup_stability_count(manager):
    manager.processes = {'liosam': object()}
    manager.process_states = {'liosam': 'STARTING'}
    manager.process_instance_ids = {'liosam': 'lio-current'}
    manager.process_generations = {'liosam': 12}
    manager.liosam_stability_ready = False
    manager.liosam_stable_correction_count = 4
    manager.liosam_last_correction_epoch = 4

    message = LioCorrectionEvent()
    message.instance_id = 'lio-current'
    message.generation = 12
    message.correction_epoch = 5
    message.source = message.SOURCE_SCAN_TO_MAP
    message.translation_residual = 0.08
    message.accepted = True
    manager.liosam_correction_callback(message)

    assert manager.liosam_stable_correction_count == 0
    assert not manager.liosam_stability_ready


def test_liosam_recovery_grace_does_not_accumulate_health_failures(
        manager, monkeypatch):
    manager.processes = {
        'liosam': SimpleNamespace(poll=lambda: None, returncode=None)}
    manager.process_states = {'liosam': 'HEALTHY'}
    manager.health_failures = {'liosam': 0}
    manager.confirmed_healthy_processes = {'liosam'}
    manager.required_process_patterns = {
        'liosam': ('lio_sam_imuPreintegration',)}
    manager.required_nodes = {
        'liosam': ('/lio_sam_imuPreintegration',)}
    manager.required_services = {}
    manager._running_process_commands = lambda _process: [
        '/opt/lio_sam/lib/lio_sam/lio_sam_imuPreintegration']
    manager._current_node_names = lambda: {'/lio_sam_imuPreintegration'}
    manager.liosam_ever_ready = True
    manager.liosam_ready = False
    manager.liosam_not_ready_since = 100.0
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.monotonic', lambda: 101.0)

    for _ in range(4):
        healthy, reason, immediate = manager.probe_process_health('liosam')
        assert not healthy
        assert reason == 'IMU preintegration is recovering (1.0/5.0 s)'
        assert not immediate
        manager.update_process_health('liosam', healthy, reason, immediate)

    assert manager.process_states['liosam'] == 'DEGRADED'
    assert manager.health_failures['liosam'] == 0
    assert manager.fault_pub.messages == []


def test_liosam_recovery_timeout_fails_immediately(manager, monkeypatch):
    manager.processes = {
        'liosam': SimpleNamespace(poll=lambda: None, returncode=None)}
    manager.process_states = {'liosam': 'DEGRADED'}
    manager.health_failures = {'liosam': 0}
    manager.confirmed_healthy_processes = {'liosam'}
    manager.required_process_patterns = {
        'liosam': ('lio_sam_imuPreintegration',)}
    manager.required_nodes = {
        'liosam': ('/lio_sam_imuPreintegration',)}
    manager.required_services = {}
    manager._running_process_commands = lambda _process: [
        '/opt/lio_sam/lib/lio_sam/lio_sam_imuPreintegration']
    manager._current_node_names = lambda: {'/lio_sam_imuPreintegration'}
    manager.liosam_ever_ready = True
    manager.liosam_ready = False
    manager.liosam_not_ready_since = 100.0
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.monotonic', lambda: 105.1)

    healthy, reason, immediate = manager.probe_process_health('liosam')
    manager.update_process_health('liosam', healthy, reason, immediate)

    assert not healthy
    assert immediate
    assert reason == 'IMU preintegration recovery timed out after 5.1 s'
    assert manager.process_states['liosam'] == 'FAILED'
    assert manager.fault_pub.messages == [
        'liosam: IMU preintegration recovery timed out after 5.1 s']


def test_ready_callback_tracks_recovery_from_first_false_message(
        manager, monkeypatch):
    manager.processes = {'liosam': object()}
    manager.process_generations = {'liosam': 12}
    manager.process_instance_ids = {'liosam': 'lio-current'}
    manager.process_states = {'liosam': 'HEALTHY'}
    monotonic_values = iter([10.0, 11.0])
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.monotonic',
        lambda: next(monotonic_values))

    def readiness(ready):
        return SimpleNamespace(
            ready=ready,
            instance_id='lio-current',
            generation=12,
        )

    manager.liosam_ready_callback(readiness(True))
    manager.liosam_ready_callback(readiness(False))
    manager.liosam_ready_callback(readiness(False))

    assert not manager.liosam_ready
    assert manager.liosam_ever_ready
    assert manager.liosam_not_ready_since == 10.0

    manager.liosam_ready_callback(readiness(True))
    assert manager.liosam_ready
    assert manager.liosam_not_ready_since is None


def test_ready_callback_rejects_previous_liosam_instance(manager):
    manager.processes = {'liosam': object()}
    manager.process_generations = {'liosam': 22}
    manager.process_instance_ids = {'liosam': 'lio-new'}
    manager.process_states = {'liosam': 'STARTING'}

    manager.liosam_ready_callback(SimpleNamespace(
        ready=True,
        instance_id='lio-old',
        generation=21,
    ))
    manager.liosam_ready_callback(SimpleNamespace(
        ready=True,
        instance_id='lio-new',
        generation=21,
    ))
    manager.liosam_ready_callback(SimpleNamespace(
        ready=True,
        instance_id='lio-old',
        generation=22,
    ))

    assert not manager.liosam_ready
    assert not manager.liosam_ever_ready

    manager.liosam_ready_callback(SimpleNamespace(
        ready=True,
        instance_id='lio-new',
        generation=22,
    ))

    assert manager.liosam_ready
    assert manager.liosam_ever_ready


def test_healthy_process_missing_required_child_fails_immediately(manager):
    manager.processes = {
        'navigation2': SimpleNamespace(poll=lambda: None, returncode=None)}
    manager.required_process_patterns = {
        'navigation2': ('component_container_isolated',)}
    manager.required_nodes = {}
    manager.required_services = {}
    manager._running_process_commands = lambda _process: []

    healthy, reason, immediate = manager.probe_process_health('navigation2')

    assert not healthy
    assert reason == 'missing child processes: component_container_isolated'
    assert immediate


def test_starting_process_missing_required_child_uses_failure_threshold(manager):
    manager._mark_starting('navigation2')
    manager.processes = {
        'navigation2': SimpleNamespace(poll=lambda: None, returncode=None)}
    manager.required_process_patterns = {
        'navigation2': ('component_container_isolated',)}
    manager.required_nodes = {}
    manager.required_services = {}
    manager._running_process_commands = lambda _process: []

    healthy, reason, immediate = manager.probe_process_health('navigation2')

    assert not healthy
    assert reason == 'missing child processes: component_container_isolated'
    assert not immediate


def test_required_child_loss_stays_immediate_after_graph_degradation(manager):
    manager.update_process_health(
        'navigation2', False, 'missing ROS nodes: /planner_server')
    assert manager.process_states['navigation2'] == 'DEGRADED'

    manager.processes = {
        'navigation2': SimpleNamespace(poll=lambda: None, returncode=None)}
    manager.required_process_patterns = {
        'navigation2': ('component_container_isolated',)}
    manager.required_nodes = {}
    manager.required_services = {}
    manager._running_process_commands = lambda _process: []

    healthy, reason, immediate = manager.probe_process_health('navigation2')

    assert not healthy
    assert reason == 'missing child processes: component_container_isolated'
    assert immediate


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
        'navigation2': SimpleNamespace(poll=lambda: None, returncode=None),
        'liosam': SimpleNamespace(poll=lambda: None, returncode=None),
    }
    manager.process_states['liosam'] = 'HEALTHY'
    manager.liosam_ready = True
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


def test_healthy_nav2_fails_immediately_when_liosam_is_not_tracked(manager):
    manager.processes = {
        'navigation2': SimpleNamespace(poll=lambda: None, returncode=None)}
    manager.required_process_patterns = {'navigation2': ()}
    manager.required_nodes = {'navigation2': ()}
    manager.required_services = {'navigation2': ()}
    manager._running_process_commands = lambda _process: []
    manager._lifecycle_nodes_active = lambda: (True, '')

    healthy, reason, immediate = manager.probe_process_health('navigation2')

    assert not healthy
    assert reason == 'required LIO-SAM process is not healthy and tracked'
    assert immediate


def test_nav2_waits_without_failure_count_while_liosam_recovers(manager):
    manager.processes = {
        'navigation2': SimpleNamespace(poll=lambda: None, returncode=None),
        'liosam': SimpleNamespace(poll=lambda: None, returncode=None),
    }
    manager.required_process_patterns = {
        'navigation2': ('component_container_isolated',)}
    manager.required_nodes = {'navigation2': ('/planner_server',)}
    manager.required_services = {
        'navigation2': ('/navigate_to_pose/_action/send_goal',)}
    manager._running_process_commands = lambda _process: [
        '/opt/ros/humble/lib/rclcpp_components/component_container_isolated']
    manager._current_node_names = lambda: {'/planner_server'}
    manager._current_service_names = lambda: {
        '/navigate_to_pose/_action/send_goal'}
    manager._lifecycle_nodes_active = lambda: (True, '')
    manager.liosam_ready = False

    for _ in range(4):
        healthy, reason, immediate = manager.probe_process_health('navigation2')
        assert not healthy
        assert reason == 'waiting for LIO-SAM IMU preintegration recovery'
        assert not immediate
        manager.update_process_health('navigation2', healthy, reason, immediate)

    assert manager.process_states['navigation2'] == 'DEGRADED'
    assert manager.health_failures['navigation2'] == 0
    assert manager.fault_pub.messages == []


def test_nav2_startup_poll_does_not_multiply_non_immediate_liosam_failure(
        manager):
    manager.processes = {'liosam': object()}
    manager.process_states = {'liosam': 'HEALTHY'}
    manager.health_failures = {'liosam': 0}
    manager.probe_process_health = lambda _name: (
        False, 'missing ROS nodes: /lio_sam_mapOptimization', False)

    for _ in range(4):
        assert manager._tracked_liosam_failure_reason() == ''

    assert manager.process_states['liosam'] == 'HEALTHY'
    assert manager.health_failures['liosam'] == 0


def test_nav2_uses_independent_startup_timeout(manager):
    manager.max_startup_time = 60.0
    manager.nav2_max_startup_time = 150.0

    assert manager._startup_timeout_for('liosam') == 60.0
    assert manager._startup_timeout_for('navigation2') == 150.0


def test_nav2_wait_stops_immediately_when_tracked_liosam_exits(
        manager, monkeypatch):
    manager.max_startup_time = 60.0
    manager.nav2_max_startup_time = 150.0
    manager.processes = {'navigation2': object(), 'liosam': object()}
    manager.process_states = {'navigation2': 'STARTING', 'liosam': 'HEALTHY'}
    manager.health_failures = {'navigation2': 0, 'liosam': 0}
    manager.confirmed_healthy_processes.add('liosam')
    probes = []

    def probe(name):
        probes.append(name)
        if name == 'liosam':
            return False, 'launch process exited with code -6', True
        raise AssertionError('Nav2 must not be probed after LIO-SAM exits')

    manager.probe_process_health = probe
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.monotonic', lambda: 0.0)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.sleep',
        lambda _seconds: pytest.fail('must stop without sleeping'))

    assert not manager.wait_for_process_healthy('navigation2')
    assert probes == ['liosam']
    assert manager.process_states['liosam'] == 'FAILED'
    assert manager.process_states['navigation2'] == 'FAILED'
    assert manager.fault_reasons['navigation2'] == (
        'LIO-SAM failed while waiting for Navigation2 startup: '
        'launch process exited with code -6')


def test_nav2_wait_stops_when_liosam_is_already_confirmed_failed(
        manager, monkeypatch):
    manager.max_startup_time = 60.0
    manager.nav2_max_startup_time = 150.0
    manager.processes = {'navigation2': object()}
    manager.process_states = {'navigation2': 'STARTING', 'liosam': 'FAILED'}
    manager.health_failures = {'navigation2': 0, 'liosam': 3}
    manager.fault_reasons = {'liosam': 'IMU preintegration crashed'}
    manager.probe_process_health = lambda _name: pytest.fail(
        'confirmed LIO-SAM failure must stop before probing Nav2')
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.monotonic', lambda: 0.0)

    assert not manager.wait_for_process_healthy('navigation2')
    assert manager.process_states['navigation2'] == 'FAILED'
    assert manager.fault_reasons['navigation2'].endswith(
        'IMU preintegration crashed')


def test_nav2_wait_stops_immediately_after_healthy_liosam_loses_child(
        manager, monkeypatch):
    manager.max_startup_time = 60.0
    manager.nav2_max_startup_time = 150.0
    manager.health_failure_threshold = 3
    manager.processes = {
        'navigation2': object(),
        'liosam': SimpleNamespace(poll=lambda: None, returncode=None),
    }
    manager.process_states = {'navigation2': 'STARTING', 'liosam': 'HEALTHY'}
    manager.health_failures = {'navigation2': 0, 'liosam': 0}
    manager.confirmed_healthy_processes.add('liosam')
    manager.required_process_patterns = {
        'liosam': ('lio_sam_imuPreintegration',),
    }
    manager.required_nodes = {}
    manager.required_services = {}
    manager._running_process_commands = lambda _process: []
    probes = []
    real_probe = manager.probe_process_health

    def probe(name):
        probes.append(name)
        if name == 'liosam':
            return real_probe(name)
        return False, 'Nav2 still starting', False

    monotonic_values = iter([0.0, 0.0])
    manager.probe_process_health = probe
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.monotonic',
        lambda: next(monotonic_values))
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.sleep', lambda _seconds: None)

    assert not manager.wait_for_process_healthy('navigation2')
    assert probes == ['liosam']
    assert manager.process_states['liosam'] == 'FAILED'
    assert manager.process_states['navigation2'] == 'FAILED'
    assert manager.fault_reasons['liosam'] == (
        'missing child processes: lio_sam_imuPreintegration')
    assert manager.fault_reasons['navigation2'] == (
        'LIO-SAM failed while waiting for Navigation2 startup: '
        'missing child processes: lio_sam_imuPreintegration')


def test_nav2_wait_continues_while_tracked_liosam_is_healthy(
        manager, monkeypatch):
    manager.max_startup_time = 60.0
    manager.nav2_max_startup_time = 150.0
    manager.processes = {'navigation2': object(), 'liosam': object()}
    manager.process_states = {'navigation2': 'STARTING', 'liosam': 'HEALTHY'}
    manager.health_failures = {'navigation2': 0, 'liosam': 0}
    nav2_results = iter([
        (False, 'lifecycle nodes not active', False),
        (True, '', False),
    ])
    probes = []

    def probe(name):
        probes.append(name)
        if name == 'liosam':
            return True, '', False
        return next(nav2_results)

    monotonic_values = iter([0.0, 0.0, 0.25])
    manager.probe_process_health = probe
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.monotonic',
        lambda: next(monotonic_values))
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.sleep', lambda _seconds: None)

    assert manager.wait_for_process_healthy('navigation2')
    assert probes == ['liosam', 'navigation2', 'liosam', 'navigation2']
    assert manager.process_states['navigation2'] == 'HEALTHY'


def test_nav2_timeout_boundary_checks_liosam_before_final_nav2_probe(
        manager, monkeypatch):
    manager.max_startup_time = 60.0
    manager.nav2_max_startup_time = 150.0
    manager.processes = {'navigation2': object(), 'liosam': object()}
    manager.process_states = {'navigation2': 'STARTING', 'liosam': 'HEALTHY'}
    manager.health_failures = {'navigation2': 0, 'liosam': 0}
    probes = []

    def probe(name):
        probes.append(name)
        if name == 'liosam':
            return False, 'launch process exited with code -6', True
        raise AssertionError('final Nav2 probe must not run after LIO-SAM fails')

    monotonic_values = iter([0.0, 150.0])
    manager.probe_process_health = probe
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.monotonic',
        lambda: next(monotonic_values))

    assert not manager.wait_for_process_healthy('navigation2')
    assert probes == ['liosam']
    assert manager.process_states['navigation2'] == 'FAILED'


def test_nav2_launch_environment_fills_only_missing_gui_values(
        manager, monkeypatch):
    gui_values = {
        'DISPLAY': ':0',
        'WAYLAND_DISPLAY': 'wayland-0',
        'XAUTHORITY': '/run/user/1001/.mutter-Xwaylandauth.test',
        'XDG_RUNTIME_DIR': '/run/user/1001',
        'DBUS_SESSION_BUS_ADDRESS': 'unix:path=/run/user/1001/bus',
    }
    for key in gui_values:
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv('DISPLAY', ':9')
    monkeypatch.setenv('WAYLAND_DISPLAY', '')
    monkeypatch.setenv('NAV2_ENV_SENTINEL', 'preserved')
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        stdout = '\n'.join(
            f'{key}={value}' for key, value in gui_values.items())
        return SimpleNamespace(stdout=stdout)

    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.run', fake_run)

    launch_env = manager._nav2_launch_environment()

    assert launch_env['DISPLAY'] == ':9'
    for key, value in gui_values.items():
        if key != 'DISPLAY':
            assert launch_env[key] == value
    assert launch_env['NAV2_ENV_SENTINEL'] == 'preserved'
    assert calls == [(
        ['systemctl', '--user', 'show-environment'],
        {
            'check': True,
            'capture_output': True,
            'text': True,
            'timeout': 2.0,
        },
    )]


def test_nav2_launch_environment_skips_systemctl_when_gui_values_exist(
        manager, monkeypatch):
    for key in (
            'DISPLAY', 'WAYLAND_DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR',
            'DBUS_SESSION_BUS_ADDRESS'):
        monkeypatch.setenv(key, f'existing-{key}')

    def unexpected_run(*_args, **_kwargs):
        raise AssertionError('systemctl must not run for a complete environment')

    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.run', unexpected_run)

    launch_env = manager._nav2_launch_environment()

    assert launch_env['DISPLAY'] == 'existing-DISPLAY'
    assert launch_env['XAUTHORITY'] == 'existing-XAUTHORITY'


def test_launch_navigation2_passes_gui_environment_to_child(
        manager, monkeypatch, tmp_path):
    launch_env = {'DISPLAY': ':0', 'NAV2_ENV_SENTINEL': 'preserved'}
    popen_calls = []
    child_process = SimpleNamespace()
    manager.map_dir = '/maps'
    liosam_process = SimpleNamespace()
    manager.processes = {'liosam': liosam_process}
    manager.process_generations = {'liosam': 7}
    manager.process_cancel_events = {'liosam': threading.Event()}
    manager.process_states = {'liosam': 'HEALTHY'}
    manager._nav2_launch_environment = lambda: launch_env
    manager._mark_starting = lambda _name: None
    manager.wait_for_process_healthy = lambda _name, _generation, **_deps: True
    monkeypatch.setenv('CAR_JETSON_MONITOR_ROOT', str(tmp_path))

    def fake_popen(command, **kwargs):
        popen_calls.append((command, kwargs))
        return child_process

    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen', fake_popen)

    assert manager.launch_navigation2('3b2mus')
    assert manager.processes['navigation2'] is child_process
    assert popen_calls[0][0] == [
        'stdbuf', '-o0', '-e0',
        'ros2', 'launch', 'nav2', 'run.real_launch.py',
        'map:=/maps/3b2mus/3b2mus_clean.yaml',
    ]
    assert popen_calls[0][1]['env'] is launch_env
    manager.nav2_log_file.close()


def test_confirmed_failure_closes_failed_process_group(manager):
    manager.processes = {'navigation2': object()}
    manager.health_failure_threshold = 1
    manager.probe_process_health = lambda name: (
        False, 'component container missing', False)
    shutdown_calls = []
    manager.shutdown_process = (
        lambda name, preserve_fault=False, **_kwargs:
        shutdown_calls.append((name, preserve_fault)) or True)

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


def test_map_to_odom_wait_stops_immediately_after_stack_fault():
    navigation = NavigationManager.__new__(NavigationManager)
    navigation.state_lock = threading.RLock()
    navigation.is_navigating = False
    navigation.stack_fault_active = True
    navigation.get_logger = lambda: FakeLogger()

    assert not navigation.wait_for_map_to_odom_link_tf(
        timeout_sec=600.0,
        check_interval=0.5,
    )
