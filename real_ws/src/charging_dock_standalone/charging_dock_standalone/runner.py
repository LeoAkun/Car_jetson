"""Capture the parked local pose as P0 and execute exactly one Dock action."""

import math

from charging_dock_msgs.action import Dock
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from yunle_msgs.msg import VehicleStatus

from charging_dock.relative_geometry import age_valid, pose2
from charging_dock_standalone.feedback_odometry import seconds


class StandaloneDocking(Node):
    """Wait for real sensors and measured parking, then skip directly to P0."""

    def __init__(self):
        super().__init__('standalone_docking')
        defaults = {
            'local_frame': 'dock_odom', 'base_frame': 'base_link',
            'scan_frame': 'rear_laser', 'scan_topic': '/laser/scan_back',
            'dry_run': True, 'auto_start': True, 'startup_timeout': 30.,
            'action_timeout': 0., 'sensor_timeout': .20, 'stable_duration': .50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.values = {name: self.get_parameter(name).value for name in defaults}
        for name in ('startup_timeout', 'sensor_timeout', 'stable_duration'):
            value = self.values[name]
            if not math.isfinite(value) or value <= 0:
                raise ValueError('invalid ' + name)
        action_timeout = self.values['action_timeout']
        if not math.isfinite(action_timeout) or action_timeout < 0:
            raise ValueError('invalid action_timeout')
        self.client = ActionClient(self, Dock, '/charging_dock/dock')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.stop_pub = self.create_publisher(Bool, '/charging_dock/emergency_stop', 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.create_service(Trigger, '~/start', self._start)
        self.create_service(Trigger, '~/stop', self._stop)
        self.vehicle = self.odom = self.scan = None
        self.create_subscription(VehicleStatus, '/vehicle_status', self._vehicle, 10)
        self.create_subscription(Odometry, '/charging_dock/local_odom', self._odom, 10)
        self.create_subscription(LaserScan, self.values['scan_topic'], self._scan,
                                 qos_profile_sensor_data)
        self.state, self.reason = 'IDLE', 'waiting_for_start'
        self.goal_handle = None
        self.started = self._now()
        self.last_now = self.started
        self.stopped_since = None
        self.p0 = None
        if self.values['auto_start']:
            self.state = 'WAIT_SENSORS'
        self.create_timer(.05, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _vehicle(self, message):
        self.vehicle = message

    def _odom(self, message):
        self.odom = message

    def _scan(self, message):
        self.scan = message

    def _start(self, _request, response):
        if self.state == 'IDLE':
            self.state, self.reason, self.started = 'WAIT_SENSORS', 'starting', self._now()
            response.success = True
            response.message = 'waiting for fresh sensors and parked P0'
        else:
            response.success = False
            response.message = 'one attempt per launch; stop and restart at the manual P0'
        return response

    def _stop(self, _request, response):
        self._halt('STOPPED', 'operator_stop')
        response.success, response.message = True, 'stop latched; restart from manual P0'
        return response

    def _halt(self, state, reason):
        self.state, self.reason = state, reason
        self.stop_pub.publish(Bool(data=True))
        if self.goal_handle is not None and self.goal_handle.accepted:
            self.goal_handle.cancel_goal_async()
        self.get_logger().error(reason)

    def _ready(self, now):
        p = self.values
        for name, message in (('can', self.vehicle), ('odometry', self.odom), ('scan', self.scan)):
            if message is None or not age_valid(now, seconds(message.header.stamp),
                                                p['sensor_timeout']):
                return 'waiting_for_' + name
        if (self.scan.header.frame_id != p['scan_frame']
                or self.odom.header.frame_id != p['local_frame']
                or self.odom.child_frame_id != p['base_frame']):
            return 'unexpected_sensor_frame'
        if not self.vehicle.is_autodrive:
            return 'waiting_for_autodrive_switch'
        speeds = (self.vehicle.cur_speed / 3.6,
                  self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z)
        if (not all(math.isfinite(v) for v in speeds)
                or abs(speeds[0]) > .01 or abs(speeds[1]) > .01 or abs(speeds[2]) > .02):
            return 'waiting_for_measured_stop'
        try:
            tf = self.buffer.lookup_transform(p['local_frame'], p['base_frame'], Time())
            pose2(tf.transform)
            if not age_valid(now, seconds(tf.header.stamp), p['sensor_timeout']):
                return 'waiting_for_fresh_tf'
        except (TransformException, ValueError):
            return 'waiting_for_tf'
        if not self.client.server_is_ready():
            return 'waiting_for_dock_action'
        if self.count_publishers('/ecu') != 1:
            return 'waiting_for_single_ecu_publisher'
        self.p0 = tf
        return ''

    def _tick(self):
        now = self._now()
        if now < self.last_now:
            self._halt('FAILED', 'clock_discontinuity')
        self.last_now = now
        if self.count_publishers('/ecu') > 1 and self.state not in ('FAILED', 'STOPPED'):
            self._halt('FAILED', 'multiple_ecu_publishers')
        if self.state == 'WAIT_SENSORS':
            self.reason = self._ready(now)
            if self.reason:
                self.stopped_since = None
            elif self.stopped_since is None:
                self.stopped_since = now
            elif now - self.stopped_since >= self.values['stable_duration']:
                self._send_goal(now)
            if (self.state == 'WAIT_SENSORS'
                    and now - self.started > self.values['startup_timeout']):
                self._halt('FAILED', 'startup_timeout:' + self.reason)
        elif self.state in ('SENDING', 'DOCKING'):
            if (self.values['action_timeout'] > 0
                    and now - self.started > self.values['action_timeout'] + 5.):
                self._halt('FAILED', 'dock_action_deadline')
        if self.state in ('FAILED', 'STOPPED'):
            self.stop_pub.publish(Bool(data=True))
        self.state_pub.publish(String(data=self.state + ' reason=' + self.reason))

    def _send_goal(self, now):
        self.state, self.reason, self.started = 'SENDING', '', now
        goal = Dock.Goal()
        goal.pre_dock_pose.header = self.p0.header
        goal.pre_dock_pose.pose.position.x = self.p0.transform.translation.x
        goal.pre_dock_pose.pose.position.y = self.p0.transform.translation.y
        goal.pre_dock_pose.pose.orientation = self.p0.transform.rotation
        goal.dry_run = self.values['dry_run']
        goal.require_charging = False
        goal.timeout_sec = self.values['action_timeout']
        self.client.send_goal_async(goal).add_done_callback(self._accepted)
        self.get_logger().info('Manual P0 captured; dry_run=' + str(goal.dry_run))

    def _accepted(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self._halt('FAILED', 'dock_goal_rejected_check_motion_and_calibration')
            elif self.state in ('FAILED', 'STOPPED'):
                self.goal_handle.cancel_goal_async()
            else:
                self.state = 'DOCKING'
                self.goal_handle.get_result_async().add_done_callback(self._finished)
        except Exception as exc:
            self._halt('FAILED', 'dock_goal_error:' + str(exc))

    def _finished(self, future):
        if self.state in ('FAILED', 'STOPPED'):
            return
        try:
            result = future.result().result
            self.reason = result.message
            if result.success:
                self.state = 'DOCKED'
            elif result.dry_run_ready:
                self.state = 'READY'
            else:
                self._halt('FAILED', 'dock_failed:' + result.message)
            self.get_logger().info(self.state + ' ' + self.reason)
        except Exception as exc:
            self._halt('FAILED', 'dock_result_error:' + str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = StandaloneDocking()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
