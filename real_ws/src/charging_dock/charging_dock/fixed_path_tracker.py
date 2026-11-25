# Copyright 2026 akun
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Odometry-closed straight reverse tracker used by Stage 3."""

from dataclasses import dataclass
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class TrackerOutput:
    command: Twist
    state: str
    traveled_m: float
    lateral_error_m: float
    yaw_error_rad: float
    reason: str = ''


class FixedPathTrackerCore:
    """Pure control core; ROS node concerns are kept outside this class."""

    def __init__(self, reverse_distance=2.5, target_speed=0.06,
                 final_speed=0.025, slow_zone_distance=0.5,
                 max_reverse_speed=0.10, max_steer_omega=0.35,
                 k_lateral=0.8, k_yaw=1.2, steering_sign=1.0,
                 max_lateral_error=0.10, max_yaw_error_deg=3.0,
                 max_motion_distance=2.8, max_duration_sec=60.0,
                 odom_timeout_sec=0.20, max_accel_mps2=0.15):
        self.reverse_distance = float(reverse_distance)
        self.target_speed = abs(float(target_speed))
        self.final_speed = abs(float(final_speed))
        self.slow_zone_distance = max(0.0, float(slow_zone_distance))
        self.max_reverse_speed = abs(float(max_reverse_speed))
        self.max_steer_omega = abs(float(max_steer_omega))
        self.k_lateral = float(k_lateral)
        self.k_yaw = float(k_yaw)
        self.steering_sign = float(steering_sign)
        self.max_lateral_error = abs(float(max_lateral_error))
        self.max_yaw_error = math.radians(abs(float(max_yaw_error_deg)))
        self.max_motion_distance = float(max_motion_distance)
        self.max_duration_sec = float(max_duration_sec)
        self.odom_timeout_sec = float(odom_timeout_sec)
        self.max_accel_mps2 = abs(float(max_accel_mps2))
        self.active = False
        self.start_pose = None
        self.start_time = None
        self.last_command_speed = 0.0
        self.last_command_time = None

    @staticmethod
    def zero_command():
        return Twist()

    def start(self, x, y, yaw, now):
        self.start_pose = (float(x), float(y), float(yaw))
        self.start_time = float(now)
        self.last_command_speed = 0.0
        self.last_command_time = float(now)
        self.active = True

    def stop(self):
        self.active = False

    def update(self, x, y, yaw, now, odom_age_sec=0.0):
        if not self.active or self.start_pose is None:
            return TrackerOutput(self.zero_command(), 'IDLE', 0.0, 0.0, 0.0)
        if odom_age_sec > self.odom_timeout_sec:
            self.active = False
            return TrackerOutput(self.zero_command(), 'FAILED', 0.0, 0.0,
                                 0.0, 'odometry_timeout')

        x0, y0, yaw0 = self.start_pose
        dx = x - x0
        dy = y - y0
        along = math.cos(yaw0) * dx + math.sin(yaw0) * dy
        lateral = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
        traveled = -along
        yaw_error = wrap_angle(yaw - yaw0)
        elapsed = now - self.start_time

        if elapsed > self.max_duration_sec:
            self.active = False
            return TrackerOutput(self.zero_command(), 'FAILED', traveled,
                                 lateral, yaw_error, 'timeout')
        if traveled > self.max_motion_distance:
            self.active = False
            return TrackerOutput(self.zero_command(), 'FAILED', traveled,
                                 lateral, yaw_error, 'distance_limit')
        if (abs(lateral) > self.max_lateral_error or
                abs(yaw_error) > self.max_yaw_error):
            self.active = False
            return TrackerOutput(self.zero_command(), 'FAILED', traveled,
                                 lateral, yaw_error, 'error_limit')
        if traveled >= self.reverse_distance:
            self.active = False
            self.last_command_speed = 0.0
            return TrackerOutput(self.zero_command(), 'SUCCEEDED', traveled,
                                 lateral, yaw_error, 'target_reached')

        remaining = self.reverse_distance - max(0.0, traveled)
        speed = self.target_speed
        if remaining < self.slow_zone_distance:
            speed = self.final_speed
        speed = min(speed, self.max_reverse_speed)
        desired_speed = -speed
        previous_time = (self.last_command_time
                         if self.last_command_time is not None else now)
        dt = max(0.0, now - previous_time)
        max_delta = self.max_accel_mps2 * dt
        if max_delta > 0.0:
            desired_speed = max(
                self.last_command_speed - max_delta,
                min(self.last_command_speed + max_delta, desired_speed))
        self.last_command_speed = desired_speed
        self.last_command_time = now
        command = Twist()
        command.linear.x = desired_speed
        raw_omega = (self.k_yaw * yaw_error + self.k_lateral * lateral)
        command.angular.z = self.steering_sign * max(
            -self.max_steer_omega, min(raw_omega, self.max_steer_omega))
        return TrackerOutput(command, 'REVERSE_APPROACH', traveled, lateral,
                             yaw_error)


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FixedPathTracker(Node):
    """ROS wrapper with an explicit enable service and fail-closed output."""

    def __init__(self):
        super().__init__('fixed_path_tracker')
        self.declare_parameter('odom_topic', '/lio_sam/mapping/odometry')
        self.declare_parameter('command_topic', '/cmd_vel_dock')
        self.declare_parameter('status_topic', '/charging_dock/stage3_state')
        self.declare_parameter('control_frequency', 25.0)
        self.declare_parameter('reverse_distance', 2.5)
        self.declare_parameter('target_speed', 0.06)
        self.declare_parameter('final_speed', 0.025)
        self.declare_parameter('slow_zone_distance', 0.5)
        self.declare_parameter('max_reverse_speed', 0.10)
        self.declare_parameter('max_steer_omega', 0.35)
        self.declare_parameter('k_lateral', 0.8)
        self.declare_parameter('k_yaw', 1.2)
        self.declare_parameter('steering_sign', 1.0)
        self.declare_parameter('max_lateral_error', 0.10)
        self.declare_parameter('max_yaw_error_deg', 3.0)
        self.declare_parameter('max_motion_distance', 2.8)
        self.declare_parameter('max_duration_sec', 60.0)
        self.declare_parameter('odom_timeout_sec', 0.20)
        self.declare_parameter('max_accel_mps2', 0.15)
        p = self.get_parameter
        self.core = FixedPathTrackerCore(
            reverse_distance=p('reverse_distance').value,
            target_speed=p('target_speed').value,
            final_speed=p('final_speed').value,
            slow_zone_distance=p('slow_zone_distance').value,
            max_reverse_speed=p('max_reverse_speed').value,
            max_steer_omega=p('max_steer_omega').value,
            k_lateral=p('k_lateral').value, k_yaw=p('k_yaw').value,
            steering_sign=p('steering_sign').value,
            max_lateral_error=p('max_lateral_error').value,
            max_yaw_error_deg=p('max_yaw_error_deg').value,
            max_motion_distance=p('max_motion_distance').value,
            max_duration_sec=p('max_duration_sec').value,
            odom_timeout_sec=p('odom_timeout_sec').value,
            max_accel_mps2=p('max_accel_mps2').value)
        self.last_odom = None
        self.last_odom_time = None
        self.publisher = self.create_publisher(
            Twist, p('command_topic').value, 10)
        self.status_pub = self.create_publisher(
            String, p('status_topic').value, 10)
        self.create_subscription(Odometry, p('odom_topic').value,
                                 self._odom_callback, 10)
        self.create_service(SetBool, '~/enable', self._enable_callback)
        frequency = max(10.0, float(p('control_frequency').value))
        self.create_timer(1.0 / frequency, self._control_callback)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.last_odom = (position.x, position.y,
                          _yaw_from_quaternion(orientation))
        self.last_odom_time = self._now_sec()

    def _enable_callback(self, request, response):
        if request.data:
            if self.last_odom is None:
                response.success = False
                response.message = 'no odometry available'
                return response
            self.core.start(*self.last_odom, self._now_sec())
            response.success = True
            response.message = 'straight reverse tracking enabled'
        else:
            self.core.stop()
            response.success = True
            response.message = 'tracking stopped'
        return response

    def _control_callback(self):
        if self.last_odom is None or self.last_odom_time is None:
            output = TrackerOutput(Twist(), 'FAILED', 0.0, 0.0, 0.0,
                                   'no_odometry')
        else:
            now = self._now_sec()
            age = now - self.last_odom_time
            output = self.core.update(*self.last_odom, now,
                                      odom_age_sec=age)
        self.publisher.publish(output.command)
        status = String()
        status.data = (f'{output.state} traveled={output.traveled_m:.3f} '
                       f'lateral={output.lateral_error_m:.3f} '
                       f'yaw_deg={math.degrees(output.yaw_error_rad):.2f} '
                       f'reason={output.reason}')
        self.status_pub.publish(status)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = FixedPathTracker()
    try:
        rclpy.spin(node)
    finally:
        node.core.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
