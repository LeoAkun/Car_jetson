"""Real route planner -> real navigation manager -> real Nav2 -> software robot.

Process-manager services are stubs: they NEVER launch hardware or other stacks.
Localization/odometry/scan feedback is synthetic, with a unicycle motion model.
"""
import json
import math
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from multi_map_navigation_msgs.msg import Edge, StartEndGraph, Waypoint, WaypointList
from multi_map_navigation_msgs.srv import GetProcessStatus, ShutdownProcess, StartProcess
from std_msgs.msg import String

from offline_common import HERE, PARAMS, Runtime
from test_navigation import SyntheticRobot


class ProcessServiceStubs:
    def __init__(self, node):
        self.running = dict.fromkeys(['re_localization', 'nav2_init_pose', 'liosam', 'navigation2'], False)
        self.starts, self.stops = [], []
        node.create_service(StartProcess, '/process_manager/start_process', self.start)
        node.create_service(ShutdownProcess, '/process_manager/shutdown_process', self.stop)
        node.create_service(GetProcessStatus, '/process_manager/get_status', self.status)

    def start(self, request, response):
        assert request.process_name in self.running
        self.starts.append((request.process_name, request.map_name))
        self.running[request.process_name] = True
        response.success = True
        response.message = 'software test stub only; no process was launched'
        return response

    def stop(self, request, response):
        assert request.process_name in self.running
        self.stops.append(request.process_name)
        self.running[request.process_name] = False
        response.success = True
        response.message = 'software test stub only'
        return response

    def status(self, request, response):
        for key, value in self.running.items():
            setattr(response, key + '_running', value)
        response.message = 'software test stub only'
        return response


def graph(points):
    message = StartEndGraph()
    message.header.frame_id = 'map'
    for index, (x, y) in enumerate(points):
        waypoint = Waypoint()
        waypoint.id, waypoint.name = index + 1, f'sim_point_{index + 1}'
        waypoint.map_name, waypoint.type = 'simulation_arena', 1
        waypoint.x, waypoint.y, waypoint.yaw, waypoint.tolerance = float(x), float(y), 0., 0.5
        message.nodes.append(waypoint)
        if index:
            edge = Edge()
            edge.start_node_id, edge.end_node_id = index, index + 1
            edge.weight = 1.
            message.edges.append(edge)
    message.start, message.end = message.nodes[0], message.nodes[-1]
    return message


def test_route_task_completes_three_waypoints_and_stack_fault_stops_motion():
    with Runtime('task-navigation') as rt:
        robot = SyntheticRobot(rt)
        services = ProcessServiceStubs(rt.node)
        states, routes, statuses, trajectory = [], [], {}, []
        rt.node.create_subscription(String, '/robot_state', lambda m: states.append(m.data), 10)
        rt.node.create_subscription(WaypointList, '/waypoint_list', routes.append, 10)

        def action_status(message):
            for status in message.status_list:
                statuses[bytes(status.goal_info.goal_id.uuid).hex()] = status.status

        rt.node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', action_status, 10)
        graph_pub = rt.node.create_publisher(StartEndGraph, '/start_end_graph', 10)
        fault_pub = rt.node.create_publisher(String, '/process_manager/fault', 10)

        def tick():
            before = robot.last
            robot.tick()
            if before != robot.last:
                trajectory.append([time.monotonic(), robot.x, robot.y, robot.yaw, robot.v, robot.w])

        nodes = [
            ('nav2_map_server', 'map_server'), ('nav2_planner', 'planner_server'),
            ('nav2_controller', 'controller_server'), ('nav2_smoother', 'smoother_server'),
            ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator'),
            ('nav2_velocity_smoother', 'velocity_smoother'),
            ('nav2_waypoint_follower', 'waypoint_follower'),
        ]
        for package, executable in nodes:
            args = ['--params-file', str(PARAMS)]
            if executable == 'map_server':
                args += ['-p', f'yaml_filename:={HERE / "arena.yaml"}']
            if executable in ('controller_server', 'behavior_server'):
                args += ['-r', 'cmd_vel:=/offline/raw_nav_cmd']
            if executable == 'velocity_smoother':
                args += ['-r', 'cmd_vel:=/offline/raw_nav_cmd', '-r', 'cmd_vel_smoothed:=/offline/nav_cmd']
            rt.start(package, executable, args)
        for transition in (1, 3):
            for _, executable in nodes:
                rt.lifecycle(executable, transition, tick)
        rt.start('multi_map_navigation', 'route_planner')
        rt.start('multi_map_navigation', 'navigation_manager', ['-r', 'cmd_vel:=/offline/raw_nav_cmd'])
        rt.wait(lambda: graph_pub.get_subscription_count() >= 1 and
                any(name == 'navigation_manager' for name, _ in rt.node.get_node_names_and_namespaces()), 15., tick)
        rt.pump(2., tick)
        # Turning across the middle waypoint exercises more than a straight line.
        graph_pub.publish(graph([(0., 0.), (1.6, 0.6), (3., 0.6)]))
        rt.wait(lambda: 'running' in states, 15., tick)
        rt.wait(lambda: len(services.stops) >= 4 and states[-1] == 'idle', 120., tick)
        rt.pump(1.5, tick)
        assert len(routes) == 1
        assert [w.name for w in routes[0].waypoints] == ['sim_point_1', 'sim_point_2', 'sim_point_3']
        assert sum(s == GoalStatus.STATUS_SUCCEEDED for s in statuses.values()) == 3, statuses
        assert [p for p, _ in services.starts] == ['re_localization', 'nav2_init_pose', 'liosam', 'navigation2']
        assert services.stops == ['navigation2', 'liosam', 'nav2_init_pose', 're_localization']
        error = math.hypot(robot.x - 3., robot.y - 0.6)
        assert error <= 0.6, (robot.x, robot.y, error)
        assert abs(robot.v) < 1e-5 and abs(robot.w) < 1e-5
        completed = {'position': [robot.x, robot.y], 'error_m': error, 'states': list(states),
                     'successful_goals': 3, 'start_calls': list(services.starts),
                     'shutdown_calls': list(services.stops), 'commands': len(robot.commands)}
        print('Full task completed:', completed, flush=True)

        # Inject a process fault during a second task; require cancellation and stop.
        previous_commands = len(robot.commands)
        previous_states = len(states)
        graph_pub.publish(graph([(4., 0.6), (4.3, 1.2)]))
        rt.wait(lambda: 'running' in states[previous_states:], 15., tick)
        rt.wait(lambda: any(abs(v) > 0.02 for _, v, _ in robot.commands[previous_commands:]), 25., tick)
        fault_pub.publish(String(data='navigation2: deliberate offline simulation fault'))
        rt.wait(lambda: len(services.stops) >= 8 and states[-1] == 'fault', 15., tick)
        rt.pump(2., tick)
        assert abs(robot.v) < 1e-5 and abs(robot.w) < 1e-5
        assert GoalStatus.STATUS_CANCELED in statuses.values(), statuses
        assert not any(services.running.values())
        report = {'model': 'software unicycle; no physical chassis', 'completed_task': completed,
                  'fault_task_state': states[-1], 'fault_cancel_verified': True,
                  'fault_stop_velocity': [robot.v, robot.w], 'trajectory': trajectory}
        (HERE / 'task-navigation-result.json').write_text(json.dumps(report, indent=2) + '\n')
        print('Injected stack fault: task faulted, Nav2 canceled, velocity zero, cleanup requested', flush=True)
