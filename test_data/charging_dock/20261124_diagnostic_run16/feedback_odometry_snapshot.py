"""Short-run odometry from signed wheel feedback or CAN speed/gear and IMU."""

from dataclasses import dataclass
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
from yunle_msgs.msg import VehicleStatus

from charging_dock.relative_geometry import age_valid, integrate, Pose2


def seconds(stamp):
    """Use acquisition timestamps, never callback receipt time."""
    return stamp.sec + stamp.nanosec / 1e9


@dataclass(frozen=True)
class Estimate:
    stamp: float
    pose: Pose2
    speed: float
    omega: float


class FeedbackIntegrator:
    """Reject gaps after initialization; restarting requires a new parked P0."""

    def __init__(self, timeout=.15):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('invalid feedback timeout')
        self.timeout = timeout
        self.estimate = None
        self.failed = False
        self.reason = 'waiting_for_can_and_imu'

    def update(self, now, vehicle_stamp, imu_stamp, speed_kmh, gear, omega,
               wheel_speeds=None):
        """Integrate feedback only; no commanded velocity is used."""
        if self.failed:
            return None
        reason = ''
        # An enabled signed wheel source remains the motion reference through
        # braking and neutral. Switching sources when the coarse main speed
        # becomes nonzero loses direction precisely during that transition.
        use_wheels = wheel_speeds is not None
        speed = speed_kmh / 3.6 * (-1 if gear == 3 else 1)
        if not (age_valid(now, vehicle_stamp, self.timeout)
                and age_valid(now, imu_stamp, self.timeout)):
            reason = 'can_or_imu_stale'
        elif not all(math.isfinite(v) for v in (speed_kmh, omega)):
            reason = 'non_finite_feedback'
        elif not 0 <= speed_kmh / 3.6 <= .5 or abs(omega) > 2.:
            reason = 'feedback_outside_test_limits'
        elif wheel_speeds is not None and (
                not all(math.isfinite(v) for v in wheel_speeds)
                or max(abs(v) for v in wheel_speeds) > .5):
            reason = 'invalid_wheel_feedback'
        elif not use_wheels and speed_kmh > 1e-6 and gear not in (1, 3):
            reason = 'moving_without_direction_feedback'
        if not reason and use_wheels:
            # Front wheel feedback is signed m/s. Preserve measured coasting
            # after neutral/brake and forward roll in reverse. The selected
            # gear does not establish the direction of actual motion.
            # Wheel asymmetry alone is allowed; integrate the signed mean.
            speed = sum(wheel_speeds) / 2
            if abs(speed) > .01 and (gear not in (1, 2, 3)
                                     or (gear == 1 and speed < 0)):
                reason = 'wheel_direction_disagrees_with_gear'
        stamp = min(vehicle_stamp, imu_stamp)
        previous = self.estimate
        if previous is not None and (stamp < previous.stamp
                                     or stamp - previous.stamp > self.timeout):
            reason = 'feedback_time_discontinuity'
        if reason:
            self.reason = reason
            self.failed = previous is not None
            return None
        if previous is not None and stamp == previous.stamp:
            return None
        pose = Pose2() if previous is None else integrate(
            previous.pose, (previous.speed + speed) / 2,
            (previous.omega + omega) / 2, stamp - previous.stamp)
        self.estimate = Estimate(stamp, pose, speed, omega)
        self.reason = ('forward_roll_in_reverse'
                       if use_wheels and gear == 3 and speed > .01 else 'valid')
        return self.estimate


class FeedbackOdometry(Node):
    """Publish fresh local TF and odometry; latch a stop on feedback failure."""

    def __init__(self):
        super().__init__('docking_feedback_odometry')
        for name, default in {
            'local_frame': 'dock_odom', 'base_frame': 'base_link',
            'imu_frame': 'imu_link', 'imu_topic': '/imu',
            'imu_yaw_rate_sign': 1.0, 'feedback_timeout': .15,
            # Historical parameter name retained for launch/config compatibility.
            # When enabled, valid signed wheel feedback is used at all speeds.
            'zero_speed_wheel_fallback': False,
        }.items():
            self.declare_parameter(name, default)
        self.values = {p.name: p.value for p in self.get_parameters([
            'local_frame', 'base_frame', 'imu_frame', 'imu_topic',
            'imu_yaw_rate_sign', 'feedback_timeout', 'zero_speed_wheel_fallback'])}
        if self.values['imu_yaw_rate_sign'] not in (-1., 1.):
            raise ValueError('imu_yaw_rate_sign must be -1 or 1')
        self.core = FeedbackIntegrator(self.values['feedback_timeout'])
        self.vehicle = self.imu = None
        self.vehicle_stamp = self.imu_stamp = -math.inf
        self.last_now = None
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/charging_dock/local_odom', 10)
        self.stop_pub = self.create_publisher(Bool, '/charging_dock/emergency_stop', 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.create_subscription(VehicleStatus, '/vehicle_status', self._vehicle, 10)
        self.create_subscription(Imu, self.values['imu_topic'], self._imu,
                                 qos_profile_sensor_data)
        self.create_timer(.02, self._tick)

    def _vehicle(self, message):
        stamp = seconds(message.header.stamp)
        if stamp <= self.vehicle_stamp and self.core.estimate is not None:
            self.core.failed, self.core.reason = True, 'can_time_discontinuity'
        self.vehicle, self.vehicle_stamp = message, stamp

    def _imu(self, message):
        stamp = seconds(message.header.stamp)
        if (message.header.frame_id != self.values['imu_frame']
                or message.angular_velocity_covariance[0] == -1):
            self.core.reason = 'invalid_imu_frame_or_gyro'
            self.core.failed = self.core.estimate is not None
            self.imu = None
            return
        if stamp <= self.imu_stamp and self.core.estimate is not None:
            self.core.failed, self.core.reason = True, 'imu_time_discontinuity'
        self.imu, self.imu_stamp = message, stamp

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_now is not None and now < self.last_now:
            self.core.failed, self.core.reason = True, 'clock_discontinuity'
        self.last_now = now
        estimate = self.core.update(
            now, self.vehicle_stamp, self.imu_stamp,
            self.vehicle.cur_speed if self.vehicle is not None else math.nan,
            self.vehicle.shift_level if self.vehicle is not None else 0,
            self.imu.angular_velocity.z * self.values['imu_yaw_rate_sign']
            if self.imu is not None else math.nan,
            (self.vehicle.left_wheel_speed, self.vehicle.right_wheel_speed)
            if self.vehicle is not None and self.values['zero_speed_wheel_fallback'] else None)
        self.state_pub.publish(String(data=self.core.reason))
        if self.core.failed:
            self.stop_pub.publish(Bool(data=True))
        if estimate is None:
            return
        # min(CAN stamp, IMU stamp) bounds freshness; never restamp frozen data.
        stamp = rclpy.time.Time(nanoseconds=round(estimate.stamp * 1e9)).to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, self.values['local_frame']
        odom.child_frame_id = self.values['base_frame']
        odom.pose.pose.position.x, odom.pose.pose.position.y = estimate.pose.x, estimate.pose.y
        odom.pose.pose.orientation.z = math.sin(estimate.pose.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(estimate.pose.yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = estimate.speed, estimate.omega
        # Dead reckoning is only for this short test, not map localization.
        odom.pose.covariance[0] = odom.pose.covariance[7] = .0025
        odom.pose.covariance[35] = .001
        self.odom_pub.publish(odom)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = odom.header, odom.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y = estimate.pose.x, estimate.pose.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = FeedbackOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
