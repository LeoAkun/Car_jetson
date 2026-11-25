"""ROS transport regression: real manager and planner, fake action outcomes.

Six-second per-waypoint delays are shortened; no hardware process is started.
"""
import os
import threading
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from multi_map_navigation.navigation_manager import NavigationManager
from multi_map_navigation.route_planner import RoutePlannerNode
from multi_map_navigation_msgs.msg import Edge, StartEndGraph, Waypoint
from test_task_navigation import ProcessServiceStubs


def test_failed_edge_alternate_route_is_actually_executed(monkeypatch):
    assert os.environ['ROS_DOMAIN_ID'] == '87'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    original_sleep = time.sleep
    monkeypatch.setattr(time, 'sleep', lambda seconds: original_sleep(.01 if seconds == 6. else seconds))
    rclpy.init(args=['--ros-args', '-r', 'cmd_vel:=/offline/recovery_cmd'])
    host = Node('offline_recovery_action_and_services')
    services = ProcessServiceStubs(host)
    services.running = dict.fromkeys(services.running, True)
    calls, states, exceptions = [], [], []

    def execute(handle):
        p = handle.request.pose.pose.position
        calls.append((p.x, p.y))
        if (p.x, p.y) == (1., 0.):
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    action = ActionServer(host, NavigateToPose, '/navigate_to_pose', execute,
                          callback_group=ReentrantCallbackGroup())
    host.create_subscription(String, '/robot_state', lambda m: states.append(m.data), 10)
    publisher = host.create_publisher(StartEndGraph, '/start_end_graph', 10)
    planner = RoutePlannerNode()
    manager = NavigationManager()
    executor = MultiThreadedExecutor(num_threads=8)
    nodes = [host, planner, manager]
    for node in nodes:
        executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception as exc:
            exceptions.append(exc)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()

    def wait(predicate, timeout=8.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert not exceptions, exceptions
            if predicate():
                return
            original_sleep(.02)
        raise AssertionError(f'Alternate route did not complete: calls={calls}, states={states}, '
                             f'manager_active={manager.is_navigating}')

    try:
        wait(lambda: publisher.get_subscription_count() and planner.waypoint_list_pub.get_subscription_count())
        message = StartEndGraph()
        for index, (name, x, y) in enumerate([('A', 0., 0.), ('B', 1., 0.), ('C', 0., 1.), ('D', 2., 0.)]):
            waypoint = Waypoint()
            waypoint.name, waypoint.id = name, index + 1
            waypoint.map_name, waypoint.type = 'offline_map', 1
            waypoint.x, waypoint.y = x, y
            message.nodes.append(waypoint)
        message.start, message.end = message.nodes[0], message.nodes[3]
        for start, end, weight in [(1, 2, 1.), (2, 4, 1.), (1, 3, 2.), (3, 4, 2.)]:
            edge = Edge()
            edge.start_node_id, edge.end_node_id, edge.weight = start, end, weight
            message.edges.append(edge)
        publisher.publish(message)
        wait(lambda: len(calls) >= 5 and states[-1] == 'idle' and not manager.is_navigating)
        assert calls == [(0., 0.), (1., 0.), (0., 0.), (0., 1.), (2., 0.)]
        assert not planner.G.has_edge('A', 'B')
        assert services.stops == ['navigation2', 'liosam', 'nav2_init_pose', 're_localization']
        print('Failed A->B was removed; actual action sequence A,B(abort),A,C,D completed', flush=True)
    finally:
        executor.shutdown(timeout_sec=3.)
        thread.join(timeout=3.)
        action.destroy()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
