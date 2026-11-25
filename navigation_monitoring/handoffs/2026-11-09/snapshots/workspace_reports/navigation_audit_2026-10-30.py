#!/usr/bin/env python3
"""Reproduce navigation control-flow defects without creating ROS nodes.

Load the current class definitions by AST, omitting imports and ROS base classes.
Method bodies are executed unchanged; ROS services, actions, clocks, and sleeps
are replaced with local fakes. Assertions confirm defects in the audited version,
not the desired behavior after a fix. Graph checks require NetworkX on PYTHONPATH.
Run after sourcing /opt/ros/humble/setup.bash.
"""

import ast
import contextlib
import io
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS


ROOT = Path(__file__).resolve().parent / "Car_jetson"
SOURCE = ROOT / "multi_map_navigation/src/multi_map_navigation/multi_map_navigation"
LOGGER = NS(**{name: lambda *_: None for name in
               ("debug", "info", "warning", "warn", "error")})
RESULTS = []


def load_class(filename, name, **globals_):
    tree = ast.parse((SOURCE / filename).read_text(encoding="utf-8"))
    cls = next(node for node in tree.body
               if isinstance(node, ast.ClassDef) and node.name == name)
    cls.bases = []
    module = ast.Module(body=[ast.ImportFrom(
        module="__future__", names=[ast.alias(name="annotations")], level=0), cls],
        type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {"threading": threading, "time": time, **globals_}
    exec(compile(module, str(SOURCE / filename), "exec"), namespace)
    return namespace[name], namespace


def point(name="A", kind=1, ident=1):
    return NS(name=name, type=kind, map_name="map1", next_map_name="map2",
              id=ident, x=0.0, y=0.0, yaw=0.0, lng=0.0, lat=0.0,
              next_x=0.0, next_y=0.0, next_yaw=0.0, tolerance=0.5)


def manager():
    fake_time = NS(sleep=lambda _: None, monotonic=time.monotonic, time=time.time)
    cls, namespace = load_class("navigation_manager.py", "NavigationManager",
                                time=fake_time)
    node = cls.__new__(cls)
    node.state_lock = threading.RLock()
    node.waypoint_list = [point(), point("B", ident=2)]
    node.current_waypoint_index = 0
    node.current_map = "map1"
    node.is_navigating = True
    node.is_map_switching = False
    node.stack_fault_active = False
    node.docking_hold = False
    node.current_goal_handle = None
    node.pending_replan_endpoints = None
    node.max_retries = 3
    node.states = []
    node.get_logger = lambda: LOGGER
    node.publish_robot_state = node.states.append
    return node, namespace


def record(name, **evidence):
    RESULTS.append({"check": name, "defect_confirmed": True, **evidence})


def check_recursive_restart():
    node, _ = manager()
    restarts = []
    node.get_process_status = lambda: {"navigation2": True}
    node.nav2_client = NS(wait_for_server=lambda **_: False)
    node.launch_new_stack = lambda _: restarts.append(True) or True
    node.shutdown_all_processes_service = lambda: None
    try:
        node.navigate_to_next_waypoint()
    except RecursionError:
        assert len(restarts) > node.max_retries
        record("recursive_action_unavailability", restarts=len(restarts),
               max_retries=node.max_retries, exception="RecursionError")
    else:
        raise AssertionError("Expected the unbounded recursive failure")

    node, _ = manager()
    launches = []
    node.get_process_status = lambda: {"navigation2": False}

    def launch(_):
        launches.append(True)
        return len(launches) % 2 == 0

    node.launch_new_stack = launch
    node.shutdown_all_processes_service = lambda: None
    try:
        node.navigate_to_next_waypoint()
    except RecursionError:
        assert len(launches) > node.max_retries * 2
        record("recursive_status_startup_disagreement", launches=len(launches),
               max_retries=node.max_retries, exception="RecursionError")
    else:
        raise AssertionError("Expected the unbounded recursive failure")


def check_stalled_waypoints_and_cancel():
    for kind in (2, 0):
        node, _ = manager()
        node.waypoint_list[0].type = kind
        actions = []
        node.navigate_to_next_waypoint = lambda: actions.append("next")
        node.complete_navigation = lambda: actions.append("complete")
        node.on_goal_reached()
        assert not actions and node.is_navigating
        record("waypoint_has_no_continuation", waypoint_type=kind,
               index_after_callback=node.current_waypoint_index,
               is_navigating=node.is_navigating, scheduled_actions=actions)

    node, _ = manager()
    actions = []
    node.complete_navigation = lambda: actions.append("complete")
    node.nav2_result_callback(NS(result=lambda: NS(status=5)))
    assert node.is_navigating and not actions
    record("canceled_goal_leaves_task_running", is_navigating=node.is_navigating,
           index=node.current_waypoint_index, states=node.states)


def check_stale_results():
    node, _ = manager()
    node.is_navigating = False
    node.waypoint_list = None
    try:
        node.nav2_result_callback(NS(result=lambda: NS(status=4)))
    except TypeError as exc:
        record("late_success_after_cleanup", exception=str(exc))
    else:
        raise AssertionError("Expected access to cleared waypoints")

    node, _ = manager()
    node.waypoint_list = [point("new-A"), point("new-B", ident=2)]
    advanced = []
    node.navigate_to_next_waypoint = lambda: advanced.append(node.current_waypoint_index)
    # There is no task/goal identity parameter to distinguish this old result.
    node.nav2_result_callback(NS(result=lambda: NS(status=4)))
    assert advanced == [1]
    record("unidentified_success_advances_current_route", advanced_indices=advanced)


def check_fault_during_sleep():
    from builtin_interfaces.msg import Time

    node, namespace = manager()
    node.get_process_status = lambda: {"navigation2": True}
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time()))
    node.navigation_behavior_tree = lambda _: "unused-local-tree.xml"
    namespace["NavigateToPose"] = NS(Goal=NS)
    goals = []

    def send(goal, **_):
        goals.append(goal)
        return NS(add_done_callback=lambda _: None)

    node.nav2_client = NS(wait_for_server=lambda **_: True, send_goal_async=send)

    def fault_during_delay(_):
        node.stack_fault_active = True
        node.is_navigating = False
        node.waypoint_list = None

    namespace["time"].sleep = fault_during_delay
    node.navigate_to_next_waypoint()
    assert len(goals) == 1 and node.stack_fault_active and not node.is_navigating
    record("goal_sent_after_fault_during_delay", sent_goals=len(goals),
           stack_fault_active=node.stack_fault_active, is_navigating=node.is_navigating,
           last_published_state=node.states[-1])


def check_repeated_map_switch_failure():
    node, _ = manager()
    node.waypoint_list[0].type = 4
    node.waypoint_list[1].map_name = "map2"
    node.is_map_switching = True
    restarts, switches = [], []
    node.shutdown_all_processes_service = lambda: None
    node.launch_new_stack = lambda _: restarts.append(True) or True

    def trigger(_):
        node.is_map_switching = True
        switches.append(True)

    node.trigger_map_switch = trigger
    for _ in range(12):
        node.map_switch_complete_callback(NS(data=False))
    assert len(restarts) == len(switches) == 12 and node.is_navigating
    record("map_switch_retries_have_no_total_limit", restarts=len(restarts),
           repeated_switches=len(switches), max_retries=node.max_retries)


def check_unused_deadlines():
    cases = (("navigation_manager.py", "navigation_timeout"),
             ("navigation_manager.py", "retry_delay"),
             ("map_switch_controller.py", "map_switch_timeout"))
    for filename, name in cases:
        tree = ast.parse((SOURCE / filename).read_text(encoding="utf-8"))
        uses = [item for item in ast.walk(tree)
                if isinstance(item, ast.Attribute) and item.attr == name]
        assert len(uses) == 1 and isinstance(uses[0].ctx, ast.Store)
        record("configured_value_never_read", filename=filename, parameter=name)


def check_paused_ros_clock():
    class LookupFailure(Exception):
        pass

    class StopProbe(Exception):
        pass

    for filename, classname in (("navigation_manager.py", "NavigationManager"),
                                ("map_switch_controller.py", "MapSwitchController")):
        attempts = []

        def lookup(**_):
            attempts.append(True)
            raise LookupFailure()

        def sleep(_):
            time.sleep(0.001)
            if len(attempts) >= 8:
                raise StopProbe()

        cls, _ = load_class(filename, classname,
            rclpy=NS(duration=NS(Duration=lambda seconds: seconds), time=NS(Time=lambda: 0)),
            tf2_ros=NS(Buffer=lambda: NS(lookup_transform=lookup),
                       TransformListener=lambda *_: None),
            LookupException=LookupFailure, ConnectivityException=LookupFailure,
            ExtrapolationException=LookupFailure, time=NS(sleep=sleep))
        node = cls.__new__(cls)
        node.state_lock = threading.RLock()
        node.stack_fault_active = False
        node.is_navigating = True
        node.get_clock = lambda: NS(now=lambda: 0.0)
        node.get_logger = lambda: LOGGER
        started = time.monotonic()
        try:
            node.wait_for_map_to_odom_link_tf(timeout_sec=0.002)
        except StopProbe:
            elapsed = time.monotonic() - started
            assert elapsed > 0.002
            record("paused_ros_clock_prevents_timeout", filename=filename,
                   configured_timeout_sec=0.002, elapsed_sec=round(elapsed, 4),
                   lookup_attempts=len(attempts), stopped_by="probe guard")
        else:
            raise AssertionError("Expected the frozen-clock wait to continue")


def check_graph_planning():
    import networkx as nx

    cls, _ = load_class("route_planner.py", "RoutePlannerNode", nx=nx)
    node = cls.__new__(cls)
    counts = {}
    for size in (7, 8):
        node.G = nx.complete_graph(size)
        nx.set_edge_attributes(node.G, 1.0, "weight")
        node.path_nodes = {}
        assert node.compute_path(0, size - 1, "weight")
        counts[str(size)] = len(node.path_nodes)
    assert counts == {"7": 326, "8": 1957}
    record("all_simple_paths_materialized", complete_graph_nodes_to_paths=counts)

    node = cls.__new__(cls)
    node.state_lock = threading.Lock()
    node.is_running = False
    node.get_logger = lambda: LOGGER
    node.path_nodes = {"path1": ["A", "B"]}
    node.update_graph_switch_node_from_list = lambda *_: None
    published = []
    node.pub_waypoint_list = lambda path, **_: published.append(list(path))
    message = NS(start=point(), end=point("B", ident=2),
                 nodes=[point(), point("B", ident=2)], edges=[])
    node.start_end_graph_callback(message)
    assert not nx.has_path(node.G, "A", "B") and published == [["A", "B"]]
    record("disconnected_new_graph_publishes_stale_route", published=published,
           current_graph_has_path=False)


def main():
    captured = io.StringIO()
    with contextlib.redirect_stdout(captured):
        check_recursive_restart()
        check_stalled_waypoints_and_cancel()
        check_stale_results()
        check_fault_during_sleep()
        check_repeated_map_switch_failure()
        check_unused_deadlines()
        check_paused_ros_clock()
        check_graph_planning()
    print(json.dumps({"source_root": str(SOURCE), "checks": RESULTS,
                      "confirmed_checks": len(RESULTS),
                      "ros_nodes_created": 0, "production_files_modified": 0},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
