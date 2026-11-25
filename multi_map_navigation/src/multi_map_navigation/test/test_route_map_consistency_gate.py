from types import SimpleNamespace
import threading

from builtin_interfaces.msg import Time

from multi_map_navigation.navigation_manager import NavigationManager
from multi_map_navigation_msgs.msg import MapSwitchResult


class FakeLogger:
    def __init__(self):
        self.messages = []

    def debug(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)

    def info(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)

    warn = warning


class PendingFuture:
    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


class RecordingNav2Client:
    def __init__(self, on_wait=None):
        self.on_wait = on_wait
        self.wait_calls = 0
        self.goals = []

    def wait_for_server(self, timeout_sec):
        self.wait_calls += 1
        if self.on_wait is not None:
            self.on_wait()
        return True

    def send_goal_async(self, goal, feedback_callback):
        self.goals.append((goal, feedback_callback))
        return PendingFuture()


class InlineThread:
    def __init__(self, target, args=(), daemon=None):
        self.target = target
        self.args = args

    def start(self):
        self.target(*self.args)


def active_manager(waypoint_map='map_a', current_map='map_a'):
    manager = NavigationManager.__new__(NavigationManager)
    manager.state_lock = threading.RLock()
    manager._navigation_generation = 7
    manager._navigation_cleanup_active = False
    manager._goal_sequence = 0
    manager._active_goal_token = None
    manager.current_goal_handle = None
    manager.current_waypoint_index = 0
    waypoint = SimpleNamespace(
        map_name=waypoint_map,
        x=1.0,
        y=2.0,
        yaw=0.3,
        type=1,
    )
    manager.waypoint_list = [waypoint]
    manager.current_map = current_map
    manager.is_navigating = True
    manager.is_map_switching = False
    manager._active_map_switch_transaction_id = ''
    manager._active_map_switch_context = None
    manager.stack_fault_active = False
    manager.docking_hold = False
    manager.pending_replan_endpoints = None
    manager._stack_recovery_token = None
    manager._reset_stack_recovery = lambda: None
    manager.logger = FakeLogger()
    manager.get_logger = lambda: manager.logger
    manager.states = []
    manager.publish_robot_state = manager.states.append
    manager.cleanup_reasons = []
    manager._cleanup_failed_stack = manager.cleanup_reasons.append
    manager.nav2_client = RecordingNav2Client()
    manager.navigation_behavior_tree = lambda _waypoint: ''
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=Time))
    return manager, waypoint


def test_mismatched_waypoint_map_faults_and_never_contacts_nav2(monkeypatch):
    manager, waypoint = active_manager(
        waypoint_map='map_after_switch', current_map='map_before_switch')
    monkeypatch.setattr(threading, 'Thread', InlineThread)

    manager.send_nav2_goal(waypoint)

    assert manager.nav2_client.wait_calls == 0
    assert manager.nav2_client.goals == []
    assert manager.stack_fault_active
    assert manager._navigation_cleanup_active
    assert manager._navigation_generation == 8
    assert manager.states == ['fault']
    assert len(manager.cleanup_reasons) == 1
    assert 'map_before_switch' in manager.cleanup_reasons[0]
    assert 'map_after_switch' in manager.cleanup_reasons[0]
    assert any('ROUTE_MAP_MISMATCH' in item for item in manager.logger.messages)


def test_matching_waypoint_map_sends_one_nav2_goal():
    manager, waypoint = active_manager()

    manager.send_nav2_goal(waypoint)

    assert manager.nav2_client.wait_calls == 1
    assert len(manager.nav2_client.goals) == 1
    assert manager._active_goal_token == (7, 0, 1)
    assert manager.states == ['running']
    assert manager.cleanup_reasons == []


def test_map_change_while_waiting_for_action_server_blocks_goal(monkeypatch):
    manager, waypoint = active_manager()
    manager.nav2_client = RecordingNav2Client(
        on_wait=lambda: setattr(manager, 'current_map', 'unexpected_map'))
    monkeypatch.setattr(threading, 'Thread', InlineThread)

    manager.send_nav2_goal(waypoint)

    assert manager.nav2_client.wait_calls == 1
    assert manager.nav2_client.goals == []
    assert manager.stack_fault_active
    assert manager.states == ['fault']
    assert len(manager.cleanup_reasons) == 1


def test_successful_map_switch_updates_owner_before_next_goal_is_sent():
    manager, switch_waypoint = active_manager()
    switch_waypoint.next_map_name = 'map_b'
    next_waypoint = SimpleNamespace(
        map_name='map_b', x=3.0, y=4.0, yaw=0.5, type=1)
    manager.waypoint_list = [switch_waypoint, next_waypoint]
    manager.is_map_switching = True
    manager._active_map_switch_transaction_id = 'tx-map-b'
    manager._active_map_switch_context = (
        'tx-map-b', 7, 0, switch_waypoint)
    navigation_attempts = []
    manager.navigate_to_next_waypoint = lambda **kwargs: (
        navigation_attempts.append(kwargs))

    result = MapSwitchResult()
    result.transaction_id = 'tx-map-b'
    result.success = True
    manager.map_switch_complete_callback(result)

    assert manager.current_map == 'map_b'
    assert manager.current_waypoint_index == 1
    assert navigation_attempts == [{'navigation_context': (7, 1)}]

    manager.send_nav2_goal(next_waypoint)

    assert len(manager.nav2_client.goals) == 1


def test_map_switch_completion_for_different_transaction_is_ignored():
    manager, switch_waypoint = active_manager()
    switch_waypoint.next_map_name = 'map_b'
    manager.is_map_switching = True
    manager._active_map_switch_transaction_id = 'tx-current'
    manager._active_map_switch_context = (
        'tx-current', 7, 0, switch_waypoint)
    navigation_attempts = []
    manager.navigate_to_next_waypoint = lambda: navigation_attempts.append(True)
    result = MapSwitchResult()
    result.transaction_id = 'tx-stale'
    result.success = True

    manager.map_switch_complete_callback(result)

    assert manager.is_map_switching
    assert manager._active_map_switch_transaction_id == 'tx-current'
    assert manager.current_map == 'map_a'
    assert manager.current_waypoint_index == 0
    assert navigation_attempts == []
    assert manager.states == []


def test_completion_with_stale_task_context_cannot_mutate_replacement_task():
    manager, old_waypoint = active_manager()
    old_waypoint.next_map_name = 'old_next_map'
    replacement_waypoint = SimpleNamespace(
        map_name='replacement_map',
        next_map_name='replacement_next_map',
        x=8.0,
        y=9.0,
        yaw=0.1,
        type=1,
    )
    manager._navigation_generation = 8
    manager.waypoint_list = [replacement_waypoint]
    manager.current_waypoint_index = 0
    manager.current_map = 'replacement_map'
    manager.is_map_switching = True
    manager._active_map_switch_transaction_id = 'tx-old-task'
    manager._active_map_switch_context = (
        'tx-old-task', 7, 0, old_waypoint)
    navigation_attempts = []
    manager.navigate_to_next_waypoint = lambda **kwargs: (
        navigation_attempts.append(kwargs))
    result = MapSwitchResult()
    result.transaction_id = 'tx-old-task'
    result.success = True

    manager.map_switch_complete_callback(result)

    assert manager.waypoint_list == [replacement_waypoint]
    assert manager.current_map == 'replacement_map'
    assert manager.current_waypoint_index == 0
    assert manager._navigation_generation == 8
    assert navigation_attempts == []
