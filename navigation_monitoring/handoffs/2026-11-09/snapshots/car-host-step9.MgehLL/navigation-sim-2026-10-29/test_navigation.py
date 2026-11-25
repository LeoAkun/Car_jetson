import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from offline_common import HERE, PARAMS, Runtime


class SyntheticRobot:
    """Software-only unicycle feedback, NOT an Ackermann hardware model."""
    def __init__(self, rt):
        self.rt = rt
        self.x = self.y = self.yaw = 0.
        self.v = self.w = 0.
        self.last = time.monotonic()
        self.last_cmd = 0.
        self.commands = []
        self.odom = rt.node.create_publisher(Odometry, '/odom', 10)
        self.scan = rt.node.create_publisher(LaserScan, '/laser/scan_merge_laser', qos_profile_sensor_data)
        self.tf = TransformBroadcaster(rt.node)
        self.static = StaticTransformBroadcaster(rt.node)
        transform = TransformStamped()
        transform.header.stamp = rt.node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.
        self.static.sendTransform(transform)
        rt.node.create_subscription(Twist, '/offline/nav_cmd', self.command, 10)

    def command(self, msg):
        assert math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)
        assert abs(msg.linear.x) <= 3.001 and abs(msg.angular.z) <= 5.001
        self.v, self.w = msg.linear.x, msg.angular.z
        self.last_cmd = time.monotonic()
        self.commands.append((self.last_cmd, self.v, self.w))

    def tick(self):
        now = time.monotonic()
        if now - self.last < 0.025:
            return
        dt = min(0.1, now - self.last)
        self.last = now
        v, w = (self.v, self.w) if now - self.last_cmd < 0.25 else (0., 0.)
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        stamp = self.rt.node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = math.sin(self.yaw / 2.)
        transform.transform.rotation.w = math.cos(self.yaw / 2.)
        self.tf.sendTransform(transform)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom.publish(odom)
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.
        scan.range_min = 0.1
        scan.range_max = 20.
        scan.ranges = [float('inf')] * 361
        self.scan.publish(scan)


def pose(rt, x, y):
    result = PoseStamped()
    result.header.frame_id = 'map'
    result.header.stamp = rt.node.get_clock().now().to_msg()
    result.pose.position.x = float(x)
    result.pose.position.y = float(y)
    result.pose.orientation.w = 1.
    return result


def send_goal(rt, client, goal, tick):
    rt.wait(lambda: client.server_is_ready(), 15., tick)
    future = client.send_goal_async(goal)
    rt.wait(future.done, 10., tick)
    handle = future.result()
    assert handle.accepted
    return handle


def test_restored_K8_map_load():
    from PIL import Image
    import yaml
    from pathlib import Path
    source = Path('/home/akun/Desktop/K8.yaml')
    config = yaml.safe_load(source.read_text())
    image = source.parent / config['image']
    expected_size = Image.open(image).size
    with Runtime('restored-map') as rt:
        maps = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        rt.node.create_subscription(OccupancyGrid, '/offline/restored_map', maps.append, qos)
        rt.start('nav2_map_server', 'map_server', [
            '-r', '__node:=restored_map_server', '-r', 'map:=/offline/restored_map',
            '-p', f'yaml_filename:={source}'])
        rt.lifecycle('restored_map_server', 1)
        rt.lifecycle('restored_map_server', 3)
        rt.wait(lambda: bool(maps), 10.)
        msg = maps[-1]
        assert (msg.info.width, msg.info.height) == expected_size
        assert abs(msg.info.resolution - config['resolution']) < 1e-6
        assert msg.info.origin.position.x == config['origin'][0]
        assert msg.info.origin.position.y == config['origin'][1]
        assert len(msg.data) == msg.info.width * msg.info.height
        assert 0 in msg.data and 100 in msg.data
        print(f'K8 actual map: {msg.info.width}x{msg.info.height}, resolution={msg.info.resolution}')


def test_real_nav2_plugins_and_closed_loop():
    with Runtime('navigation') as rt:
        robot = SyntheticRobot(rt)
        nodes = [
            ('nav2_map_server', 'map_server'),
            ('nav2_planner', 'planner_server'),
            ('nav2_controller', 'controller_server'),
            ('nav2_smoother', 'smoother_server'),
            ('nav2_behaviors', 'behavior_server'),
            ('nav2_bt_navigator', 'bt_navigator'),
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
                args += ['-r', 'cmd_vel:=/offline/raw_nav_cmd',
                         '-r', 'cmd_vel_smoothed:=/offline/nav_cmd']
            rt.start(package, executable, args)
        for _, executable in nodes:
            rt.lifecycle(executable, 1, robot.tick)
        for _, executable in nodes:
            rt.lifecycle(executable, 3, robot.tick)
        rt.pump(1., robot.tick)
        planner = ActionClient(rt.node, ComputePathToPose, '/compute_path_to_pose')
        goal = ComputePathToPose.Goal()
        goal.goal = pose(rt, 3., 0.)
        goal.planner_id = 'GridBased'
        handle = send_goal(rt, planner, goal, robot.tick)
        future = handle.get_result_async()
        rt.wait(future.done, 15., robot.tick)
        result = future.result()
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        path = result.result.path
        assert len(path.poses) >= 10
        assert all(math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y) for p in path.poses)
        assert math.hypot(path.poses[-1].pose.position.x - 3., path.poses[-1].pose.position.y) <= 0.51
        print(f'SmacHybrid REEDS_SHEPP path: {len(path.poses)} poses')
        # A goal inside the known occupied block must not return a successful plan.
        goal.goal = pose(rt, 5.5, 0.)
        handle = send_goal(rt, planner, goal, robot.tick)
        future = handle.get_result_async()
        rt.wait(future.done, 15., robot.tick)
        assert future.result().status == GoalStatus.STATUS_ABORTED
        print('Occupied-goal planning correctly rejected')
        controller = ActionClient(rt.node, FollowPath, '/follow_path')
        follow = FollowPath.Goal()
        follow.path = path
        follow.controller_id = 'FollowPath'
        follow.goal_checker_id = 'general_goal_checker'
        handle = send_goal(rt, controller, follow, robot.tick)
        rt.wait(lambda: any(abs(v) > 0.02 for _, v, _ in robot.commands), 15., robot.tick)
        cancel = handle.cancel_goal_async()
        rt.wait(cancel.done, 10., robot.tick)
        assert cancel.result().goals_canceling
        result = handle.get_result_async()
        rt.wait(result.done, 10., robot.tick)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        rt.pump(1.5, robot.tick)
        assert abs(robot.v) < 1e-5 and abs(robot.w) < 1e-5
        print('MPPI FollowPath cancellation + smoothed stop passed')
        navigator = ActionClient(rt.node, NavigateToPose, '/navigate_to_pose')
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose(rt, 3., 0.)
        handle = send_goal(rt, navigator, nav_goal, robot.tick)
        result = handle.get_result_async()
        rt.wait(result.done, 55., robot.tick)
        assert result.result().status == GoalStatus.STATUS_SUCCEEDED
        rt.pump(1.5, robot.tick)
        assert math.hypot(robot.x - 3., robot.y) <= 0.6
        assert abs(robot.v) < 1e-5
        print(f'NavigateToPose closed-loop succeeded: x={robot.x:.3f}, y={robot.y:.3f}, commands={len(robot.commands)}')
