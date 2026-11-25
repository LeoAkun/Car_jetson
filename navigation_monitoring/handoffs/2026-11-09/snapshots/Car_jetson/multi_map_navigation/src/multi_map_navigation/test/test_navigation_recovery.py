from pathlib import Path
from types import SimpleNamespace
import threading
import time

import networkx as nx
import pytest
import yaml
from builtin_interfaces.msg import Time
from rclpy.task import Future

from multi_map_navigation.navigation_manager import NavigationManager
from multi_map_navigation.route_planner import RoutePlannerNode
from multi_map_navigation_msgs.msg import Waypoint, WaypointList


class FakeLogger:
    def __init__(self):
        self.messages = []

    def debug(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)

    def info(self, message):
        self.messages.append(message)

    def warn(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)


class CompletedFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class FakeClient:
    def __init__(self, response):
        self.response = response
        self.requests = []

    def call_async(self, request):
        self.requests.append(request)
        return CompletedFuture(self.response)


def waypoint(name):
    item = Waypoint()
    item.name = name
    item.map_name = "map1"
    item.type = 1
    return item


def repo_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "nav2/src/nav2/config/nav2_real_params.yaml").is_file():
            return parent
    raise AssertionError("repository root not found")


def test_real_nav2_config_keeps_dubins_workaround_and_valid_mppi_timing():
    config_path = repo_root() / "nav2/src/nav2/config/nav2_real_params.yaml"
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    controller = config["controller_server"]["ros__parameters"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    global_inflation = config["global_costmap"]["global_costmap"]["ros__parameters"][
        "inflation_layer"
    ]
    mppi = controller["FollowPath"]

    assert planner["smooth_path"] is False
    assert planner["minimum_turning_radius"] == pytest.approx(0.40)
    assert mppi["AckermannConstraints"]["min_turning_r"] == pytest.approx(0.40)
    assert global_inflation["inflation_radius"] == pytest.approx(2.0)
    assert controller["controller_frequency"] == pytest.approx(30.0)
    assert mppi["time_steps"] == 56
    assert mppi["model_dt"] == pytest.approx(0.05)
    assert mppi["model_dt"] >= 1.0 / controller["controller_frequency"]
    assert mppi["time_steps"] * mppi["model_dt"] == pytest.approx(2.8)


def test_aborted_goal_requests_alternate_route_from_last_completed_waypoint():
    manager = NavigationManager.__new__(NavigationManager)
    manager.state_lock = threading.RLock()
    manager.current_waypoint_index = 2
    manager.waypoint_list = [waypoint("A"), waypoint("B"), waypoint("C")]
    manager.pub_new_path_client = FakeClient(SimpleNamespace(success=True))
    manager.get_logger = lambda: FakeLogger()
    manager.publish_robot_state = lambda _state: None
    manager.complete_navigation = lambda: pytest.fail("navigation should continue")

    assert manager.abort_navigation(reason=1)
    request = manager.pub_new_path_client.requests[0]
    assert [item.name for item in request.points] == ["A", "B"]
    assert manager.pending_replan_endpoints == ("B", "C")


def test_aborted_first_goal_finishes_cleanly_without_empty_replan_request():
    manager = NavigationManager.__new__(NavigationManager)
    manager.state_lock = threading.RLock()
    manager.current_waypoint_index = 0
    manager.waypoint_list = [waypoint("A"), waypoint("B")]
    manager.pub_new_path_client = FakeClient(SimpleNamespace(success=True))
    manager.get_logger = lambda: FakeLogger()
    manager.publish_robot_state = lambda _state: None
    completed = []
    manager.complete_navigation = lambda: completed.append(True)

    assert not manager.abort_navigation(reason=1)
    assert completed == [True]
    assert manager.pub_new_path_client.requests == []


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.canceled = False

    def cancel(self):
        self.canceled = True


def recovery_manager(max_retries=3):
    manager = NavigationManager.__new__(NavigationManager)
    manager.state_lock = threading.RLock()
    manager.waypoint_list = [waypoint("A"), waypoint("B")]
    manager.current_waypoint_index = 0
    manager.current_map = "map1"
    manager.is_navigating = True
    manager.is_map_switching = False
    manager.stack_fault_active = False
    manager.docking_hold = False
    manager.max_retries = max_retries
    manager.retry_delay = 0.01
    manager.stack_recovery_clock = SimpleNamespace()
    manager._stack_recovery_generation = 0
    manager._stack_recovery_context = None
    manager._stack_recovery_attempts = 0
    manager._stack_recovery_token = None
    manager._stack_recovery_timer = None
    manager.scheduled_timers = []
    manager.faults = []
    manager.get_logger = lambda: FakeLogger()
    manager.publish_robot_state = lambda _state: None
    manager.create_timer = lambda _delay, callback, **_kwargs: (
        manager.scheduled_timers.append(FakeTimer(callback)) or manager.scheduled_timers[-1])
    manager.destroy_timer = lambda _timer: None
    manager.shutdown_all_processes_service = lambda: None
    manager.process_fault_callback = lambda message: manager.faults.append(message.data)
    return manager


def run_scheduled_recovery(manager):
    for _ in range(20):
        if not manager.scheduled_timers:
            return
        timer = manager.scheduled_timers.pop(0)
        if not timer.canceled:
            timer.callback()
    pytest.fail('recovery did not stop within the test execution budget')


def test_stack_restart_failures_share_retry_budget_without_recursion():
    manager = recovery_manager()
    launches = []
    manager.launch_new_stack = lambda _map: launches.append(True) or False

    assert manager.abort_navigation(reason=2)
    assert launches == []
    run_scheduled_recovery(manager)

    assert len(launches) == manager.max_retries
    assert len(manager.faults) == 1
    assert manager._stack_recovery_timer is None


def test_unavailable_action_service_cannot_reset_retry_budget_after_restart(monkeypatch):
    manager = recovery_manager()
    launches = []
    manager.launch_new_stack = lambda _map: launches.append(True) or True
    manager.get_process_status = lambda: {"navigation2": True}
    manager.nav2_client = SimpleNamespace(wait_for_server=lambda **_kwargs: False)
    monkeypatch.setattr('multi_map_navigation.navigation_manager.time.sleep', lambda _: None)

    manager.navigate_to_next_waypoint()
    assert launches == []
    run_scheduled_recovery(manager)

    assert len(launches) == manager.max_retries
    assert len(manager.faults) == 1
    assert manager._stack_recovery_attempts == manager.max_retries


def test_successful_restart_sends_goal_without_recursive_status_recheck():
    manager = recovery_manager()
    launches = []
    goals = []
    manager.get_process_status = lambda: {"navigation2": False}
    manager.launch_new_stack = lambda _map: launches.append(True) or len(launches) == 2
    manager.send_nav2_goal = lambda goal, *_context: goals.append(goal)

    manager.navigate_to_next_waypoint()
    assert len(launches) == 1
    run_scheduled_recovery(manager)

    assert len(launches) == 2
    assert goals == [manager.waypoint_list[0]]
    assert manager.faults == []


def test_repeated_map_switch_failures_stop_after_shared_retry_budget():
    manager = recovery_manager()
    manager.is_map_switching = True
    launches = []
    switches = []
    manager.launch_new_stack = lambda _map: launches.append(True) or True

    def retry_switch(**_kwargs):
        manager.is_map_switching = True
        switches.append(True)

    manager.on_goal_reached = retry_switch

    for _ in range(manager.max_retries + 1):
        manager.map_switch_complete_callback(SimpleNamespace(data=False))
        run_scheduled_recovery(manager)

    assert len(launches) == manager.max_retries
    assert len(switches) == manager.max_retries
    assert len(manager.faults) == 1


@pytest.mark.parametrize('change', ['reset', 'fault', 'waypoint'])
def test_queued_restart_cannot_act_on_expired_context(change):
    manager = recovery_manager()
    launches = []
    manager.launch_new_stack = lambda _map: launches.append(True) or True
    manager.abort_navigation(reason=2)
    queued_callback = manager.scheduled_timers[0].callback

    if change == 'reset':
        manager._reset_stack_recovery()
    elif change == 'fault':
        manager.stack_fault_active = True
    else:
        manager._advance_current_waypoint()
    queued_callback()

    assert launches == []


def test_duplicate_failures_schedule_only_one_recovery():
    manager = recovery_manager()

    for _ in range(5):
        assert manager.abort_navigation(reason=2)

    assert len(manager.scheduled_timers) == 1
    assert manager._stack_recovery_attempts == 1


def test_zero_retry_budget_faults_without_starting_stack():
    manager = recovery_manager(max_retries=0)

    assert not manager.abort_navigation(reason=2)

    assert manager.scheduled_timers == []
    assert len(manager.faults) == 1


def test_exhausted_recovery_stops_and_cleans_task_as_fault(monkeypatch):
    manager = recovery_manager()
    launches, stops, states, shutdowns = [], [], [], []
    manager.current_goal_handle = SimpleNamespace(cancel_goal_async=lambda: None)
    manager.emergency_cmd_vel_pub = SimpleNamespace(publish=stops.append)
    manager.publish_robot_state = states.append
    manager.process_fault_callback = NavigationManager.process_fault_callback.__get__(manager)
    manager.shutdown_all_processes_service = lambda: shutdowns.append(True)
    manager.launch_new_stack = lambda _map: launches.append(True) or False
    monkeypatch.setattr('multi_map_navigation.navigation_manager.threading.Thread',
                        lambda target, args, daemon: SimpleNamespace(
                            start=lambda: target(*args)))

    manager.abort_navigation(reason=2)
    run_scheduled_recovery(manager)

    assert len(launches) == manager.max_retries
    assert len(shutdowns) == manager.max_retries + 1
    assert len(stops) == 3
    assert all(msg.linear.x == 0.0 and msg.angular.z == 0.0 for msg in stops)
    assert states == ['fault', 'fault']
    assert manager.stack_fault_active
    assert not manager.is_navigating
    assert manager.waypoint_list is None


def test_retry_delay_is_used_and_budget_resets_on_waypoint_progress():
    manager = recovery_manager(max_retries=1)
    delays = []
    original_create_timer = manager.create_timer
    manager.create_timer = lambda delay, cb, **_kwargs: (
        delays.append(delay) or original_create_timer(delay, cb))
    manager.launch_new_stack = lambda _: True
    manager.send_nav2_goal = lambda *_args: None

    assert manager.abort_navigation(reason=2)
    run_scheduled_recovery(manager)
    manager._advance_current_waypoint()
    assert manager.abort_navigation(reason=2)
    run_scheduled_recovery(manager)

    assert delays == [manager.retry_delay, manager.retry_delay]
    assert manager._stack_recovery_attempts == 1
    assert manager.faults == []


def test_real_recovery_timers_finish_with_paused_ros_clock():
    from rclpy.clock import Clock, ClockType
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    context = Context()
    context.init(domain_id=225)
    manager = recovery_manager()
    manager.create_timer = Node.create_timer.__get__(manager)
    manager.destroy_timer = Node.destroy_timer.__get__(manager)
    Node.__init__(manager, 'navigation_recovery_timer_test', context=context,
                  parameter_overrides=[Parameter('use_sim_time', value=True)])
    manager.stack_recovery_clock = Clock(clock_type=ClockType.STEADY_TIME)
    manager.retry_delay = 0.001
    launches = []
    manager.launch_new_stack = lambda _: launches.append(True) or True
    manager.nav2_client = SimpleNamespace(wait_for_server=lambda **_kwargs: False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(manager)
    try:
        assert manager.get_clock().now().nanoseconds == 0
        manager.abort_navigation(reason=2)
        deadline = time.monotonic() + 2.0
        while not manager.faults and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(launches) == manager.max_retries
        assert len(manager.faults) == 1
        assert list(manager.timers) == []
    finally:
        executor.remove_node(manager)
        executor.shutdown()
        manager.destroy_node()
        context.shutdown()


def test_route_planner_removes_failed_edge_and_publishes_alternate_path():
    planner = RoutePlannerNode.__new__(RoutePlannerNode)
    planner.G = nx.Graph()
    for name in ("A", "B", "C", "D"):
        planner.G.add_node(name, type=1, map_name="map1")
    planner.G.add_weighted_edges_from([
        ("A", "B", 1.0),
        ("B", "D", 1.0),
        ("A", "C", 2.0),
        ("C", "D", 2.0),
    ])
    planner.path_nodes = {"path1": ["A", "B", "D"]}
    planner.end_node_name = "D"
    planner.get_logger = lambda: FakeLogger()
    published = []
    planner.pub_waypoint_list = lambda nodes, path_name: published.append(
        (list(nodes), path_name))

    request = SimpleNamespace(points=[waypoint("A")])
    response = SimpleNamespace(success=False, message="")
    result = planner.pub_new_path_callback(request, response)

    assert result.success
    assert not planner.G.has_edge("A", "B")
    assert planner.path_nodes["path1"] == ["A", "C", "D"]
    assert published == [(["A", "C", "D"], "path1")]


def test_fault_state_allows_route_planner_to_accept_a_new_task():
    planner = RoutePlannerNode.__new__(RoutePlannerNode)
    planner.state_lock = threading.Lock()
    planner.is_running = True

    planner.robot_state_callback(SimpleNamespace(data="fault"))

    assert not planner.is_running


def action_manager(monkeypatch):
    manager = recovery_manager()
    manager._navigation_generation = 1
    manager._goal_sequence = 0
    manager._active_goal_token = None
    manager.current_goal_handle = None
    manager.pending_replan_endpoints = None
    manager.states = []
    manager.publish_robot_state = manager.states.append
    manager.emergency_cmd_vel_pub = SimpleNamespace(publish=lambda _msg: None)
    manager.process_fault_callback = NavigationManager.process_fault_callback.__get__(manager)
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    manager.navigation_behavior_tree = lambda _waypoint: ''
    manager.get_process_status = lambda: {'navigation2': True}
    manager.goal_requests = []

    def send_goal(goal, **_kwargs):
        future = Future()
        manager.goal_requests.append((goal, future))
        return future

    manager.nav2_client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: True, send_goal_async=send_goal)
    monkeypatch.setattr('multi_map_navigation.navigation_manager.threading.Thread',
                        lambda target, args, daemon: SimpleNamespace(
                            start=lambda: target(*args)))
    monkeypatch.setattr('multi_map_navigation.navigation_manager.time.sleep', lambda _: None)
    return manager


def replace_faulted_task(manager):
    # Reuse the very same waypoints to exercise the generation check too.
    previous_waypoints = manager.waypoint_list
    manager.process_fault_callback(SimpleNamespace(data='injected stack fault'))
    manager.start_navigation_sequence = lambda: None
    message = WaypointList()
    message.task_id = 'new-task'
    message.start_map_name = 'map1'
    message.waypoints = previous_waypoints
    message.total_waypoints = len(previous_waypoints)
    manager.waypoint_list_callback(message)


@pytest.mark.parametrize('stage', ['status', 'sleep', 'server_ready', 'server_missing', 'pose'])
@pytest.mark.parametrize('new_task', [False, True])
def test_waiting_navigation_never_sends_goal_after_fault(monkeypatch, stage, new_task):
    manager = action_manager(monkeypatch)
    old_generation = manager._navigation_generation

    def interrupt():
        if new_task:
            replace_faulted_task(manager)
        else:
            manager.process_fault_callback(SimpleNamespace(data='injected stack fault'))

    if stage == 'status':
        manager.get_process_status = lambda: interrupt() or {'navigation2': True}
    elif stage == 'sleep':
        monkeypatch.setattr('multi_map_navigation.navigation_manager.time.sleep',
                            lambda _: interrupt())
    elif stage.startswith('server'):
        manager.nav2_client.wait_for_server = lambda **_kwargs: (
            interrupt() or stage == 'server_ready')
    else:
        manager.navigation_behavior_tree = lambda _waypoint: interrupt() or ''

    manager.navigate_to_next_waypoint()

    assert manager.goal_requests == []
    assert manager.scheduled_timers == []
    assert manager.states == ['fault', 'fault'] + (['running'] if new_task else [])
    assert manager._navigation_generation > old_generation
    assert manager.is_navigating is new_task


class DeferredGoalHandle:
    accepted = True

    def __init__(self):
        self.cancel_count = 0
        self.result_future = Future()

    def cancel_goal_async(self):
        self.cancel_count += 1

    def get_result_async(self):
        return self.result_future


def test_late_goal_acceptance_is_canceled_without_replacing_new_goal(monkeypatch):
    manager = action_manager(monkeypatch)
    manager.send_nav2_goal(manager.waypoint_list[0])
    old_response = manager.goal_requests[-1][1]
    replace_faulted_task(manager)
    manager.send_nav2_goal(manager.waypoint_list[0])
    current_handle = DeferredGoalHandle()
    manager.goal_requests[-1][1].set_result(current_handle)
    stale_handle = DeferredGoalHandle()
    states = list(manager.states)

    old_response.set_result(stale_handle)

    assert stale_handle.cancel_count == 1
    assert current_handle.cancel_count == 0
    assert manager.current_goal_handle is current_handle
    assert manager.states == states
    assert manager.current_waypoint_index == 0


@pytest.mark.parametrize('status', [4, 5, 6])
def test_old_goal_result_cannot_change_new_task(monkeypatch, status):
    manager = action_manager(monkeypatch)
    manager.send_nav2_goal(manager.waypoint_list[0])
    handle = DeferredGoalHandle()
    manager.goal_requests[-1][1].set_result(handle)
    replace_faulted_task(manager)
    states = list(manager.states)

    handle.result_future.set_result(SimpleNamespace(status=status))

    assert manager.current_waypoint_index == 0
    assert manager.states == states
    assert manager.scheduled_timers == []
    assert manager.is_navigating


@pytest.mark.parametrize('kind', ['rejected', 'exception'])
def test_old_goal_response_cannot_restart_or_fault_new_task(monkeypatch, kind):
    manager = action_manager(monkeypatch)
    manager.send_nav2_goal(manager.waypoint_list[0])
    old_response = manager.goal_requests[-1][1]
    replace_faulted_task(manager)
    states = list(manager.states)

    if kind == 'rejected':
        old_response.set_result(SimpleNamespace(accepted=False))
    else:
        old_response.set_exception(RuntimeError('late failure'))

    assert manager.states == states
    assert manager.scheduled_timers == []
    assert manager.is_navigating


def test_current_goal_advances_once_and_completes_normally(monkeypatch):
    manager = action_manager(monkeypatch)
    manager.send_nav2_goal(manager.waypoint_list[0])
    first_token = manager._active_goal_token
    first_handle = DeferredGoalHandle()
    manager.goal_requests[-1][1].set_result(first_handle)
    success = SimpleNamespace(status=4)

    first_handle.result_future.set_result(success)

    assert manager.current_waypoint_index == 1
    assert len(manager.goal_requests) == 2
    second_token = manager._active_goal_token
    manager.nav2_result_callback(CompletedFuture(success), first_token)
    assert manager._active_goal_token == second_token
    assert manager.current_waypoint_index == 1
    second_handle = DeferredGoalHandle()
    manager.goal_requests[-1][1].set_result(second_handle)
    second_handle.result_future.set_result(success)

    assert not manager.is_navigating
    assert manager.waypoint_list is None
    assert manager.states[-1] == 'idle'
    assert manager._active_goal_token is None


def test_fault_during_waypoint_delay_cannot_resume_new_task(monkeypatch):
    manager = action_manager(monkeypatch)
    manager.send_nav2_goal(manager.waypoint_list[0])
    handle = DeferredGoalHandle()
    manager.goal_requests[-1][1].set_result(handle)
    monkeypatch.setattr('multi_map_navigation.navigation_manager.time.sleep',
                        lambda _: replace_faulted_task(manager))

    handle.result_future.set_result(SimpleNamespace(status=4))

    assert len(manager.goal_requests) == 1
    assert manager.current_waypoint_index == 0
    assert manager.is_navigating
    assert manager._active_goal_token is None


def test_completion_invalidates_old_work_before_stack_cleanup(monkeypatch):
    manager = action_manager(monkeypatch)
    old_waypoint = manager.waypoint_list[0]
    old_generation = manager._navigation_generation
    new_task = WaypointList()
    new_task.waypoints = [waypoint('new')]
    started = []
    manager.start_navigation_sequence = lambda: started.append(True)

    def cleanup():
        manager.waypoint_list_callback(new_task)
        manager.send_nav2_goal(old_waypoint, old_generation, 0)

    manager.shutdown_all_processes_service = cleanup
    manager.complete_navigation()

    assert started == []
    assert manager.goal_requests == []
    assert manager.states == ['idle']
    assert not manager._navigation_cleanup_active
    manager.waypoint_list_callback(new_task)
    assert started == [True]


def test_dense_graph_plans_only_one_weighted_shortest_route(monkeypatch):
    planner = RoutePlannerNode.__new__(RoutePlannerNode)
    planner.G = nx.complete_graph(16)
    nx.set_edge_attributes(planner.G, 100.0, 'cost')
    planner.G[0][1]['cost'] = 1.0
    planner.G[1][15]['cost'] = 1.0
    planner.path_nodes = {'path1': ['old'], 'path2': ['old-alternative']}
    monkeypatch.setattr(nx, 'shortest_simple_paths',
                        lambda *_args, **_kwargs: pytest.fail('must not enumerate paths'))

    assert planner.compute_path(0, 15, 'cost')
    assert planner.path_nodes == {'path1': [0, 1, 15]}


@pytest.mark.parametrize('source,target', [('A', 'B'), ('missing', 'B'), ('A', 'missing')])
def test_unreachable_route_clears_previous_task_path(source, target):
    planner = RoutePlannerNode.__new__(RoutePlannerNode)
    planner.G = nx.Graph()
    planner.G.add_nodes_from(['A', 'B'])
    planner.path_nodes = {'path1': ['old', 'route']}

    assert not planner.compute_path(source, target, 'weight')
    assert planner.path_nodes == {}


def test_unreachable_new_task_does_not_publish_old_route():
    planner = RoutePlannerNode.__new__(RoutePlannerNode)
    planner.state_lock = threading.Lock()
    planner.is_running = False
    planner.path_nodes = {'path1': ['old', 'route']}
    planner.get_logger = lambda: FakeLogger()
    planner.pub_waypoint_list = lambda *_args, **_kwargs: pytest.fail('no reachable route')
    message = SimpleNamespace(start=waypoint('A'), end=waypoint('B'),
                              nodes=[waypoint('A'), waypoint('B')], edges=[])

    planner.start_end_graph_callback(message)

    assert planner.path_nodes == {}


@pytest.mark.parametrize("pending,names,accepted", [
    (("B", "D"), ["B", "C", "D"], True),
    (("B", "D"), ["X", "Y"], False),
    (None, ["B", "C", "D"], False),
    (("B", "D"), [], False),
])
def test_active_task_accepts_only_its_requested_alternate_route(pending, names, accepted):
    manager = NavigationManager.__new__(NavigationManager)
    manager.state_lock = threading.RLock()
    manager.is_navigating = True
    manager.pending_replan_endpoints = pending
    original = [waypoint("A"), waypoint("B"), waypoint("D")]
    manager.waypoint_list = original
    manager.get_logger = lambda: FakeLogger()
    manager.publish_robot_state = lambda _state: None
    started = []
    manager.start_navigation_sequence = lambda: started.append(True)
    message = WaypointList()
    message.task_id = "alternate"
    message.start_map_name = "map1"
    message.waypoints = [waypoint(name) for name in names]
    message.total_waypoints = len(names)

    manager.waypoint_list_callback(message)

    assert bool(started) is accepted
    if accepted:
        assert [item.name for item in manager.waypoint_list] == names
        assert manager.pending_replan_endpoints is None
        assert manager.current_waypoint_index == 0
    else:
        assert manager.waypoint_list is original
        assert manager.pending_replan_endpoints == pending
