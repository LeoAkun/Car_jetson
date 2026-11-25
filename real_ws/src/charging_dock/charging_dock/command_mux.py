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

"""Exclusive velocity command multiplexer for navigation and docking."""

from copy import deepcopy
import math

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from yunle_msgs.msg import VehicleStatus
from tf2_ros import Buffer, TransformException, TransformListener

from charging_dock.relative_geometry import age_valid, pose2, wrap


def select_source(dock_active, dock_fresh, nav_fresh):
    """Return the only source allowed to control the output."""
    if dock_active:
        return 'dock' if dock_fresh else 'none'
    return 'nav' if nav_fresh else 'none'


class CommandMux(Node):
    """Select one fresh command source and fail closed to zero velocity."""

    def __init__(self):
        super().__init__('command_mux')
        self.declare_parameter('output_topic', '/cmd_vel_selected')
        self.declare_parameter('nav_topic', '/cmd_vel')
        self.declare_parameter('legacy_nav_topic', '')
        self.declare_parameter('dock_topic', '/cmd_vel_dock')
        self.declare_parameter('command_timeout_sec', 0.20)
        self.declare_parameter('handoff_stop_cycles', 3)
        self.declare_parameter('measured_handoff', False)
        self.declare_parameter('local_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.measured_handoff = self.get_parameter('measured_handoff').value
        self.timeout = max(0.05, float(
            self.get_parameter('command_timeout_sec').value))
        self.handoff_cycles = max(0, int(
            self.get_parameter('handoff_stop_cycles').value))
        self.nav = None
        self.nav_time = None
        self.dock = None
        self.dock_time = None
        self.dock_active = False
        self.source = 'none'
        self.handoff_remaining = 0
        self.pending_mode = None
        self.mode_future = None
        self.mode_requested_at = None
        self.vehicle_speed = math.inf
        self.vehicle_stamp = -math.inf
        self.autodrive = False
        self.stop_latched = False
        self.stopped_since = None
        self.motion_pose = None
        self.motion_stamp = -math.inf
        self.motion_speed = self.motion_omega = math.inf
        if self.measured_handoff:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        nav_topic = self.get_parameter('nav_topic').value
        legacy_topic = self.get_parameter('legacy_nav_topic').value
        dock_topic = self.get_parameter('dock_topic').value
        self.publisher = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)
        self.create_subscription(Twist, nav_topic, self._nav_callback, 10)
        if legacy_topic and legacy_topic != nav_topic:
            self.create_subscription(
                Twist, legacy_topic, self._legacy_nav_callback, 10)
        self.create_subscription(Twist, dock_topic, self._dock_callback, 10)
        self.create_service(
            SetBool, '~/set_dock_active', self._set_dock_active)
        self.create_service(SetBool, '~/set_stop', self._set_stop)
        self.create_subscription(Bool, '/charging_dock/emergency_stop', self._emergency, 10)
        self.create_subscription(VehicleStatus, '/vehicle_status', self._vehicle, 10)
        self.owned_pub = self.create_publisher(Bool, '/charging_dock/control_owned', 10)
        self.mode_client = self.create_client(SetBool, '/cmdvel_to_ecu/set_docking_mode')
        self.create_timer(0.02, self._publish_selected)

    def _now(self):
        return self.get_clock().now()

    def _nav_callback(self, msg):
        self.nav = deepcopy(msg)
        self.nav_time = self._now()

    def _legacy_nav_callback(self, msg):
        self.nav = deepcopy(msg)
        self.nav_time = self._now()

    def _dock_callback(self, msg):
        self.dock = deepcopy(msg)
        self.dock_time = self._now()

    def _set_dock_active(self, request, response):
        if self.measured_handoff:
            if self.stop_latched:
                response.success = False
                response.message = 'stop is latched; clear it explicitly while stopped'
                return response
            if self.pending_mode is not None and self.pending_mode != request.data:
                response.success = False
                response.message = 'another control handoff is pending'
                return response
            if self.pending_mode is None and self.dock_active == request.data:
                response.success = True
                response.message = 'control mode confirmed'
                return response
            if self.pending_mode is None:
                self.pending_mode = request.data
                self.mode_future = None
                self.nav = self.dock = None
                self.nav_time = self.dock_time = None
                self.stopped_since = None
            response.success = False
            response.message = 'waiting for measured stop and ECU mode confirmation'
            return response
        if not request.data:
            self.dock_active = False
            self.handoff_remaining = self.handoff_cycles
            response.success = True
            response.message = 'docking control released'
            return response
        # Never reuse a non-zero command left over from an earlier attempt.
        self.dock = None
        self.dock_time = None
        self.dock_active = True
        self.handoff_remaining = self.handoff_cycles
        response.success = True
        response.message = 'docking control requested'
        return response

    def _fresh(self, stamp):
        return (stamp is not None and
                0 <= (self._now() - stamp).nanoseconds / 1e9 <= self.timeout)

    def _vehicle(self, message):
        self.vehicle_speed = float(message.cur_speed) / 3.6
        self.vehicle_stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        self.autodrive = bool(message.is_autodrive)

    def _stopped(self):
        return (age_valid(self._now().nanoseconds / 1e9, self.vehicle_stamp, self.timeout)
                and math.isfinite(self.vehicle_speed) and abs(self.vehicle_speed) <= 0.01
                and age_valid(self._now().nanoseconds / 1e9, self.motion_stamp, self.timeout)
                and self.motion_speed <= 0.01 and abs(self.motion_omega) <= 0.02)

    def _update_motion(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.get_parameter('local_frame').value,
                                                 self.get_parameter('base_frame').value, Time())
            pose = pose2(tf.transform)
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
            if self.motion_pose is not None and stamp > self.motion_stamp:
                dt = stamp - self.motion_stamp
                self.motion_speed = math.hypot(pose.x - self.motion_pose.x,
                                               pose.y - self.motion_pose.y) / dt
                self.motion_omega = wrap(pose.yaw - self.motion_pose.yaw) / dt
            elif stamp < self.motion_stamp:
                self.motion_speed = self.motion_omega = math.inf
            self.motion_pose, self.motion_stamp = pose, stamp
        except (TransformException, ValueError):
            self.motion_speed = self.motion_omega = math.inf

    def _emergency(self, message):
        if message.data:
            self.stop_latched = True

    def _set_stop(self, request, response):
        if request.data or self._stopped():
            self.stop_latched = request.data
            self.nav = self.dock = None
            self.nav_time = self.dock_time = None
            response.success = True
            response.message = ('stop latched' if request.data
                                else 'stop cleared; old commands discarded')
        else:
            response.success = False
            response.message = 'fresh measured stop required to clear stop'
        return response

    def _advance_handoff(self):
        if self.pending_mode is None:
            return
        now = self._now().nanoseconds / 1e9
        if not self._stopped():
            self.stopped_since = None
            return
        if self.stopped_since is None:
            self.stopped_since = now
        if now - self.stopped_since < 0.5:
            return
        if self.mode_future is not None:
            if self.mode_future.done():
                try:
                    response = self.mode_future.result()
                    if response.success:
                        self.dock_active = self.pending_mode
                        self.pending_mode = None
                        self.nav = self.dock = None
                        self.nav_time = self.dock_time = None
                    else:
                        self.stop_latched = True
                except Exception:
                    self.stop_latched = True
                self.mode_future = None
            elif now - self.mode_requested_at > 2.0:
                self.stop_latched = True
            return
        if self.mode_client.service_is_ready():
            request = SetBool.Request()
            request.data = self.pending_mode
            self.mode_future = self.mode_client.call_async(request)
            self.mode_requested_at = now

    def _publish_selected(self):
        if self.measured_handoff:
            self._update_motion()
            self._advance_handoff()
        source = select_source(
            self.dock_active,
            self._fresh(self.dock_time),
            self._fresh(self.nav_time),
        )
        if source == 'dock':
            selected = self.dock
        elif source == 'nav':
            selected = self.nav
        else:
            selected = Twist()

        if source != self.source:
            self.source = source
            self.handoff_remaining = self.handoff_cycles
        if self.handoff_remaining > 0:
            selected = Twist()
            self.handoff_remaining -= 1
        if self.measured_handoff:
            feedback_fresh = age_valid(self._now().nanoseconds / 1e9,
                                       self.vehicle_stamp, self.timeout)
            if self.dock_active and (not feedback_fresh or not self.autodrive):
                self.stop_latched = True
            if self.pending_mode is not None or not feedback_fresh:
                selected = Twist()
        if self.stop_latched:
            selected = Twist()
        values = (selected.linear.x, selected.angular.z)
        if not all(math.isfinite(v) for v in values):
            selected = Twist()
            self.stop_latched = True
        self.publisher.publish(selected)
        self.owned_pub.publish(Bool(data=bool(
            self.dock_active and self.pending_mode is None and not self.stop_latched)))


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = CommandMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
