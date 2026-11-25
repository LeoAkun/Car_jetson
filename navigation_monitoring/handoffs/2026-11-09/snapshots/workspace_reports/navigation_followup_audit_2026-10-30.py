#!/usr/bin/env python3
"""Read-only navigation audit: no vehicle nodes, processes, MQTT, or HTTP calls.

Assertions intentionally confirm existing defects; this is not a passing
regression suite for a fixed implementation. Use the ROS overlay and car-host
venv described in the accompanying report.
"""

import ast
from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import signal
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import patch

from builtin_interfaces.msg import Time
import networkx as nx
import rclpy.time
from multi_map_navigation import navigation_manager as navigation_module
from multi_map_navigation import process_manager as process_module
from multi_map_navigation.charging_navigation import ChargingNavigation
from multi_map_navigation.map_switch_controller import MapSwitchController
from multi_map_navigation.mqtt_task_receiver import MQTTTaskReceiver
from multi_map_navigation.navigation_manager import NavigationManager
from multi_map_navigation.process_manager import ProcessManagerNode
from multi_map_navigation.route_planner import RoutePlannerNode
from multi_map_navigation.status_reporter import StatusReporter
from multi_map_navigation_msgs.msg import Edge, Waypoint, WaypointList


ROOT = Path('/home/akun/workspace/Car_jetson')
SOURCE = ROOT / 'multi_map_navigation/src/multi_map_navigation/multi_map_navigation'
RESULTS = []


class Logger:
    def __init__(self):
        self.messages = []

    def __getattr__(self, name):
        return self.messages.append


class DoneFuture:
    def __init__(self, result):
        self.value = result

    def done(self):
        return True

    def result(self):
        return self.value


def point(name, index=0, kind=1, map_name='map1', next_map=''):
    value = Waypoint()
    value.name, value.id, value.type = name, index, kind
    value.map_name, value.next_map_name = map_name, next_map
    return value


def base_node(cls):
    node = cls.__new__(cls)
    node.logger = Logger()
    node.get_logger = lambda: node.logger
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Time()))
    return node


def manager():
    node = base_node(NavigationManager)
    node.state_lock = threading.RLock()
    node.waypoint_list = [point('A'), point('B', 1), point('D', 2)]
    node.current_waypoint_index = 0
    node.current_map = 'map1'
    node.is_navigating, node.is_map_switching = True, False
    node.stack_fault_active, node.docking_hold = False, False
    node.current_goal_handle = None
    node._navigation_generation, node._active_goal_token = 1, None
    node.pending_replan_endpoints = None
    node.states, node.sent, node.started = [], [], []
    node.publish_robot_state = node.states.append
    node.navigate_to_next_waypoint = lambda **_kwargs: node.sent.append(True)
    node.start_navigation_sequence = lambda: node.started.append(True)
    node.shutdown_all_processes_service = lambda: None
    node.emergency_cmd_vel_pub = NS(publish=lambda _value: None)
    return node


def planner():
    node = base_node(RoutePlannerNode)
    node.state_lock = threading.Lock()
    node.is_running = False
    node.G, node.path_nodes = nx.Graph(), {}
    node.published = []
    node.pub_waypoint_list = lambda names, **_kwargs: node.published.append(list(names))
    return node


def graph(names):
    nodes = [point(name, index) for index, name in enumerate(names)]
    edges = []
    for index in range(len(nodes) - 1):
        edge = Edge()
        edge.start_node_id, edge.end_node_id = index, index + 1
        edge.weight = 1.0
        edges.append(edge)
    return NS(start=nodes[0], end=nodes[-1], nodes=nodes, edges=edges)


def observe(name, test):
    with redirect_stdout(io.StringIO()):
        details = test()
    RESULTS.append({'observation': name, 'confirmed': True, 'details': details})


def faulted_map_completion():
    node = manager()
    node.is_map_switching = node.stack_fault_active = True
    node.map_switch_complete_callback(NS(data=True))
    assert node.states == ['running'] and node.current_waypoint_index == 1
    return {'states': node.states, 'waypoint_index_after_fault': node.current_waypoint_index}


def late_map_completion():
    node = manager()
    node.is_map_switching = True
    node.waypoint_list[0].next_map_name = 'new-map'
    # Bool carries no task or switch identifier, so an old True is accepted.
    node.map_switch_complete_callback(NS(data=True))
    assert node.current_waypoint_index == 1 and node.current_map == 'new-map'
    return {'old_completion_advances_current_task': True}


def map_worker_survives_task_fault():
    node = base_node(MapSwitchController)
    lifecycle, launches = {'faulted': False}, []
    node.shutdown_current_stack = lambda: True

    def fault_then_return():
        lifecycle['faulted'] = True
        return True

    node.wait_for_clean_shutdown = lambda _timeout: fault_then_return()
    node.process_shutdown_timeout = 0.01
    node.launch_new_stack = lambda name: launches.append((name, lifecycle['faulted'])) or True
    node.verify_stack_ready = lambda: True
    assert node.execute_map_switch('map1', 'map2')
    assert launches == [('map2', True)]
    return {'launches_after_task_fault': launches}


def concurrent_map_requests():
    node = base_node(MapSwitchController)
    node.is_switching = False
    before_execute, inside_shutdown = threading.Barrier(2), threading.Barrier(2)
    launches, errors = [], []
    logger = Logger()

    def info(message):
        if message.startswith('触发地图切换:'):
            before_execute.wait(timeout=2.0)
        logger.messages.append(message)

    logger.info = info
    node.get_logger = lambda: logger
    node.map_switch_complete_pub = NS(publish=lambda _value: None)
    node.process_shutdown_timeout = 0.01
    node.shutdown_current_stack = lambda: inside_shutdown.wait(timeout=2.0) is not None
    node.wait_for_clean_shutdown = lambda _timeout: True
    node.launch_new_stack = lambda name: launches.append(name) or True
    node.verify_stack_ready = lambda: True

    def run(name):
        try:
            node.map_switch_callback(NS(current_map='map1', next_map=name))
        except Exception as exc:
            errors.append(repr(exc))

    threads = [threading.Thread(target=run, args=(name,), daemon=True)
               for name in ('map2', 'map3')]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=3.0)
    assert not errors and sorted(launches) == ['map2', 'map3']
    return {'overlapping_switches': sorted(launches)}


def docking_fault_has_no_recovery_entry():
    node = manager()
    node.docking_hold, node.docking_in_progress = True, True
    node.docking_complete, node.docking_generation = False, 1
    node.docking_handle, node.docking_release_deadline = None, None
    node.docking_release_future = None
    node.docking_stop_pub = NS(publish=lambda _value: None)
    with patch.object(navigation_module.threading, 'Thread',
                      lambda target, args, daemon: NS(start=lambda: target(*args))):
        node.process_fault_callback(NS(data='injected stack fault while docking'))
    message = WaypointList()
    message.waypoints = [point('new')]
    node.waypoint_list_callback(message)
    retry = node.retry_docking(None, NS())
    release = node.release_docking(None, NS())
    assert not node.is_navigating and node.waypoint_list is None
    assert node.docking_hold and not node.started
    assert not retry.success and not release.success
    return {'hold': node.docking_hold, 'new_task_started': False,
            'retry_allowed': retry.success, 'release_allowed': release.success}


def canceled_goal_stays_running():
    node = manager()
    token = (1, 0, 1)
    node._active_goal_token = token
    node.nav2_result_callback(DoneFuture(NS(status=5)), token)
    assert node.is_navigating and node._active_goal_token is None
    assert not node.sent and node.states == ['running']
    return {'is_navigating': True, 'active_goal': None, 'next_action_scheduled': False}


def unsupported_waypoint_stalls():
    observations = []
    for kind in (0, 2):
        node = manager()
        node.waypoint_list[0].type = kind
        node.on_goal_reached()
        assert node.is_navigating and not node.sent
        observations.append({'type': kind, 'index': node.current_waypoint_index,
                             'is_navigating': True, 'next_action_scheduled': False})
    return observations


def graph_replacement_precedes_state_delivery():
    node, navigation = planner(), manager()
    navigation.is_navigating = False
    active_routes = []

    def publish(names, **_kwargs):
        message = WaypointList()
        message.task_id, message.start_map_name = 'task', 'map1'
        message.waypoints = [point(name) for name in names]
        navigation.waypoint_list_callback(message)
        active_routes.append([item.name for item in navigation.waypoint_list])

    node.pub_waypoint_list = publish
    # Deliberately delay /robot_state delivery between these graph callbacks.
    node.start_end_graph_callback(graph(['A', 'B', 'D']))
    node.start_end_graph_callback(graph(['A', 'C', 'D']))
    assert node.path_nodes['path1'] == ['A', 'C', 'D']
    assert active_routes[-1] == ['A', 'B', 'D']
    return {'planner_route': node.path_nodes['path1'], 'executing_route': active_routes[-1]}


def invalid_edge_reference_escapes_callback():
    node = planner()
    message = graph(['A', 'B'])
    message.edges[0].end_node_id = 999
    try:
        node.start_end_graph_callback(message)
    except KeyError as exc:
        return {'uncaught_exception': 'KeyError', 'missing_node_id': exc.args[0]}
    raise AssertionError('invalid edge did not raise')


def mqtt_accepts_invalid_values():
    node = base_node(MQTTTaskReceiver)
    invalid = node.parse_single_waypoint({'name': 'A', 'slam_x': float('nan')})
    negative = node.parse_single_edge({'from_id': 1, 'to_id': 2, 'weight': -1.0})
    assert invalid is not None and math.isnan(invalid.x) and invalid.type == 0
    assert negative is not None and negative.weight < 0
    return {'nan_coordinate_accepted': True, 'missing_type_becomes': invalid.type,
            'negative_edge_weight_accepted': negative.weight}


def invalid_weight_escapes_dijkstra():
    node = planner()
    node.G.add_weighted_edges_from([('A', 'B', 1), ('B', 'C', -3), ('C', 'D', 10)])
    try:
        node.compute_path('A', 'D', 'weight')
    except ValueError as exc:
        return {'uncaught_exception': type(exc).__name__, 'message': str(exc)}
    raise AssertionError('invalid negative weights did not raise')


def two_adjacent_map_switches():
    node = planner()
    nodes = [point('A'), point('S', 1, 4, 'map1', 'map2'),
             point('T', 2, 4, 'map2', 'map3'), point('D', 3, 1, 'map3')]
    for item in nodes:
        node.G.add_node(item.name, type=item.type, map_name=item.map_name,
                        next_map_name=item.next_map_name, x=0., y=0., yaw=0.,
                        next_x=1., next_y=1., next_yaw=0.)
    node.update_graph_switch_node_from_list([item.name for item in nodes], 'map1')
    assert node.G.nodes['T']['map_name'] == 'map3'
    return {'second_switch_expected_approach_map': 'map2',
            'second_switch_actual_approach_map': node.G.nodes['T']['map_name']}


def startup_overwrites_tracked_process():
    node = base_node(ProcessManagerNode)
    old = NS(pid=111)
    new = NS(pid=222)
    node.processes, node.process_states = {'re_localization': old}, {}
    node.health_failures, node.fault_reasons = {}, {}
    node.map_dir, node.startup_delay = '/unused-map-dir', 0
    node.is_process_running = lambda _name: True
    created = []
    with patch.object(process_module.subprocess, 'Popen',
                      lambda *args, **kwargs: created.append(True) or new):
        assert node.launch_relocalization('map1')
    assert node.processes['re_localization'] is new and created == [True]
    return {'previous_pid': old.pid, 'tracked_pid_now': new.pid,
            'old_process_shutdown_requested': False}


def root_exit_does_not_prove_children_exit():
    node = base_node(ProcessManagerNode)
    node.processes = {'liosam': NS(pid=111, poll=lambda: 0)}
    node.process_states, node.health_failures, node.fault_reasons = {}, {}, {}
    node.shutdown_timeout = 0.01
    signals = []
    with patch.object(process_module.os, 'getpgid', return_value=111), \
            patch.object(process_module.os, 'killpg', side_effect=lambda *args: signals.append(args)):
        assert node.shutdown_process('liosam')
    assert signals == [(111, signal.SIGTERM)] and 'liosam' not in node.processes
    return {'signals': [int(value[1]) for value in signals],
            'reports_success_after_root_exit_without_child_verification': True}


def shutdown_service_errors_ignored():
    node = base_node(MapSwitchController)
    calls = []
    node.shutdown_process_client = NS(call_async=lambda request: calls.append(request.process_name)
                                     or DoneFuture(NS(success=False)))
    assert node.shutdown_current_stack() is True
    return {'reported_success': True, 'failed_shutdown_calls': calls}


def idle_parking_triggers_immediate_takeover():
    node = base_node(StatusReporter)
    node.robot_status = {'task_status': 0}
    node.zero_speed_start_time = time.time() - 600.0
    node.speed_threshold_time, node.remote_driving_request_sent = 300, False
    calls = []
    node.send_remote_driving_request = lambda: calls.append(True)
    node.robot_state_callback(NS(data='running'))
    node.check_vehicle_problem()
    assert calls == [True]
    return {'remote_driving_request_immediately_after_task_start': True}


def failed_takeover_disables_retries():
    node = base_node(StatusReporter)
    node.robot_status = {'task_status': 1}
    node.zero_speed_start_time = time.time() - 600.0
    node.speed_threshold_time, node.remote_driving_request_sent = 300, False
    attempts = []
    # The real send method also returns None after HTTP errors/timeouts.
    node.send_remote_driving_request = lambda: attempts.append('failed')
    node.check_vehicle_problem()
    node.check_vehicle_problem()
    assert attempts == ['failed'] and node.remote_driving_request_sent
    return {'attempt_count': len(attempts), 'request_sent_latched_after_failure': True}


def timeout_parameters_not_read():
    findings = {}
    for filename, parameter in [('navigation_manager.py', 'navigation_timeout'),
                                ('map_switch_controller.py', 'map_switch_timeout')]:
        tree = ast.parse((SOURCE / filename).read_text())
        reads = [node.lineno for node in ast.walk(tree)
                 if isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Load)
                 and isinstance(node.value, ast.Name) and node.value.id == 'self'
                 and node.attr == parameter]
        assert reads == []
        findings[parameter] = {'reads_after_assignment': reads}
    return findings


def paused_ros_clock_prevents_tf_timeout():
    observations = []

    class HarnessStop(Exception):
        pass

    for cls, module_name in [(NavigationManager, 'navigation_manager'),
                             (MapSwitchController, 'map_switch_controller')]:
        node = manager() if cls is NavigationManager else base_node(cls)
        node.get_clock = lambda: NS(now=lambda: rclpy.time.Time(seconds=0))
        attempts = []

        def missing_tf(**_kwargs):
            attempts.append(True)
            raise navigation_module.LookupException('injected missing TF')

        def budgeted_sleep(_seconds):
            if len(attempts) >= 8:
                raise HarnessStop()

        prefix = 'multi_map_navigation.' + module_name
        with patch(prefix + '.tf2_ros.Buffer', return_value=NS(lookup_transform=missing_tf)), \
                patch(prefix + '.tf2_ros.TransformListener', return_value=NS()), \
                patch(prefix + '.time.sleep', side_effect=budgeted_sleep):
            try:
                node.wait_for_map_to_odom_link_tf(timeout_sec=0.000001)
            except HarnessStop:
                pass
            else:
                raise AssertionError('paused TF wait unexpectedly returned')
        assert len(attempts) == 8
        observations.append({'class': cls.__name__, 'attempts_before_harness_stop': len(attempts),
                             'ros_clock_seconds': 0, 'declared_timeout_seconds': 0.000001})
    return observations


def main():
    tests = [faulted_map_completion, late_map_completion, map_worker_survives_task_fault,
             concurrent_map_requests, docking_fault_has_no_recovery_entry,
             canceled_goal_stays_running, unsupported_waypoint_stalls,
             graph_replacement_precedes_state_delivery, invalid_edge_reference_escapes_callback,
             mqtt_accepts_invalid_values, invalid_weight_escapes_dijkstra,
             two_adjacent_map_switches, startup_overwrites_tracked_process,
             root_exit_does_not_prove_children_exit, shutdown_service_errors_ignored,
             idle_parking_triggers_immediate_takeover, failed_takeover_disables_retries,
             timeout_parameters_not_read, paused_ros_clock_prevents_tf_timeout]
    for test in tests:
        observe(test.__name__, test)
    result = {'source_root': str(SOURCE), 'observations': RESULTS,
              'confirmed_count': len(RESULTS), 'vehicle_nodes_started': False}
    output = Path('/home/akun/workspace/navigation_followup_audit_results_2026-10-30.json')
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
