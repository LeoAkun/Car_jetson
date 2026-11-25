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

"""ROS action adapter for live rear-lidar relative docking."""

from dataclasses import fields, replace
import math
import threading

from charging_dock_msgs.action import Dock
from charging_dock_msgs.msg import DockObservation
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from yunle_msgs.msg import Battery, VehicleStatus

from charging_dock.dock_collision import (collision, dock_plane_collision,
                                          locked_target_indices)
from charging_dock.relative_control import ControlInput, DockConfig, RelativeDockingCore
from charging_dock.relative_geometry import age_valid, longitudinal_speed, pose2, Pose2, wrap
from charging_dock.relative_lidar import FaceConfig, RelativeFaceTracker


def stamp_sec(stamp):
    """Convert ROS time to seconds without using receipt time for freshness."""
    return stamp.sec + stamp.nanosec / 1e9


class RelativeDockingNode(Node):
    """Serialize action and sensor state; retain zero control after every result."""

    def __init__(self):
        super().__init__('relative_docking_controller')
        defaults = {
            'base_frame': 'base_link', 'local_frame': 'odom', 'global_frame': 'map',
            'scan_frame': 'rear_laser', 'scan_topic': '/laser/scan_back',
            'allow_motion': False, 'nominal_dock_distance': 2.5,
            'p0_longitudinal_tolerance': 0.20, 'p0_lateral_tolerance': 0.10,
            'p0_yaw_tolerance': math.radians(5.0),
            'receiver_x': -0.45, 'receiver_y': 0.0, 'receiver_yaw': 0.0,
            'target_receiver_x': 0.01, 'target_receiver_y': 0.0,
            'target_receiver_yaw': 0.0,
            'face_to_dock_x': 0.0, 'face_to_dock_y': 0.0, 'face_to_dock_yaw': 0.0,
            'footprint_front': 0.43, 'footprint_rear': 0.43,
            'footprint_half_width': 0.43, 'collision_margin': 0.02,
            'minimum_charge_current': 0.1,
        }
        defaults.update({f.name: getattr(DockConfig(), f.name)
                         for f in fields(DockConfig)
                         if f.name not in ('base_receiver', 'dock_receiver', 'face_to_dock')})
        defaults.update({'face_' + f.name: getattr(FaceConfig(), f.name)
                         for f in fields(FaceConfig)})
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.values = {name: self.get_parameter(name).value for name in defaults}
        p = self.values
        config = {f.name: p[f.name] for f in fields(DockConfig)
                  if f.name not in ('base_receiver', 'dock_receiver', 'face_to_dock')}
        config['base_receiver'] = Pose2(p['receiver_x'], p['receiver_y'], p['receiver_yaw'])
        config['dock_receiver'] = Pose2(p['target_receiver_x'], p['target_receiver_y'],
                                        p['target_receiver_yaw'])
        config['face_to_dock'] = Pose2(p['face_to_dock_x'], p['face_to_dock_y'],
                                       p['face_to_dock_yaw'])
        self.core = RelativeDockingCore(DockConfig(**config))
        self.tracker = RelativeFaceTracker(FaceConfig(**{
            f.name: p['face_' + f.name] for f in fields(FaceConfig)}))
        self.lock = threading.RLock()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.group = ReentrantCallbackGroup()
        self.goal = None
        self.reserved = False
        self.finished = None
        self.requested_at = None
        self.vehicle = None
        self.vehicle_time = -math.inf
        self.owned = False
        self.owned_time = -math.inf
        self.charging = False
        self.charge_time = -math.inf
        self.observation = None
        self.latest_scan = None
        self.pending_scan = None
        self.last_output = None
        self.last_tf_pose = None
        self.last_tf_time = None
        self.last_linear_speed = 0.0
        self.last_longitudinal_speed = 0.0
        self.p0_stable_since = None
        self.command_pub = self.create_publisher(Twist, '/cmd_vel_dock', 10)
        self.state_pub = self.create_publisher(String, '/charging_dock/state', 10)
        self.observation_pub = self.create_publisher(
            DockObservation, '/charging_dock/observation', 10)
        self.owner_client = self.create_client(
            SetBool, '/command_mux/set_dock_active', callback_group=self.group)
        self.owner_future = None
        self.owner_request_time = -math.inf
        self.create_subscription(LaserScan, p['scan_topic'], self._scan,
                                 qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/vehicle_status', self._vehicle, 10)
        self.create_subscription(Battery, '/battery_status', self._battery, 10)
        self.create_subscription(Bool, '/charging_dock/control_owned', self._owned, 10)
        self.action = ActionServer(self, Dock, '/charging_dock/dock',
                                   execute_callback=self._execute,
                                   goal_callback=self._goal,
                                   cancel_callback=self._cancel,
                                   callback_group=self.group)
        self.create_timer(0.04, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _transform(self, parent, child, stamp=None):
        result = self.buffer.lookup_transform(parent, child, stamp or Time(),
                                              timeout=Duration(seconds=0.0))
        return pose2(result.transform), stamp_sec(result.header.stamp)

    def _goal(self, request):
        with self.lock:
            try:
                pose2(request.pre_dock_pose.pose)
                valid = (not self.reserved and request.dock_id == 0
                         and request.pre_dock_pose.header.frame_id
                         == self.values['global_frame']
                         and math.isfinite(request.timeout_sec) and request.timeout_sec >= 0)
                if not request.dry_run:
                    valid = valid and self.values['allow_motion'] and self.core.config.calibrated
                if valid:
                    self.reserved = True
                    return GoalResponse.ACCEPT
            except ValueError:
                pass
            return GoalResponse.REJECT

    def _cancel(self, _handle):
        with self.lock:
            if self.goal is not None:
                self.core.cancel(self._now())
            return CancelResponse.ACCEPT

    async def _execute(self, handle):
        with self.lock:
            self.goal = handle
            self.finished = Future()
            done = self.finished
            self.requested_at = self._now()
            self.core.state = 'IDLE'
            self.core.error = None
            self.core.traveled = 0.0
            self.observation = self.latest_scan = None
            self.pending_scan = None
            self.tracker.reset()
            self.p0_stable_since = None
            self.last_tf_pose = self.last_tf_time = None
            self.last_omega = 0.0
            self.last_linear_speed = 0.0
            self.last_longitudinal_speed = 0.0
        result = await done
        with self.lock:
            if handle.is_cancel_requested:
                handle.canceled()
            elif result.success or result.dry_run_ready:
                handle.succeed()
            else:
                handle.abort()
            self.goal = None
            self.reserved = False
            self.finished = None
        return result

    def _owned(self, message):
        with self.lock:
            self.owned, self.owned_time = message.data, self._now()

    def _vehicle(self, message):
        with self.lock:
            self.vehicle, self.vehicle_time = message, stamp_sec(message.header.stamp)

    def _battery(self, message):
        with self.lock:
            # This project Battery message has no Header; only receipt freshness
            # is available. The CAN publishing path must be validated on vehicle.
            self.charge_time = self._now()
            self.charging = (message.charge_status == 1 and math.isfinite(message.ampere)
                             and message.ampere >= self.values['minimum_charge_current'])

    def _scan(self, message):
        with self.lock:
            if self.goal is None or self.tracker.target is None:
                return
            if message.header.frame_id != self.values['scan_frame']:
                self.observation = self.latest_scan = self.pending_scan = None
                return
            # TF often arrives immediately after the scan. Keep one bounded
            # pending scan for the timer; never substitute a latest-time TF.
            self._try_scan(self._now())
            if self.pending_scan is None:
                self.pending_scan = message

    def _try_scan(self, now):
        message = self.pending_scan
        if message is None:
            return
        if not age_valid(now, stamp_sec(message.header.stamp), self.tracker.config.timeout):
            self.observation = self.latest_scan = self.pending_scan = None
            return
        try:
            stamp = Time.from_msg(message.header.stamp)
            local_base, _ = self._transform(self.values['local_frame'],
                                            self.values['base_frame'], stamp)
            base_sensor, _ = self._transform(self.values['base_frame'],
                                             message.header.frame_id, stamp)
            raw = self.tracker.process(message, now, local_base, base_sensor)
            # The beam indices/width describe the scanned back plate. Control,
            # published pose and shell clearance use the physical charging face.
            self.observation = replace(
                raw, pose=self.core.config.dock_pose(raw.pose) if raw.pose else None)
            self.latest_scan = (message, local_base, base_sensor)
            self.pending_scan = None
            self._publish_observation(message)
        except (TransformException, ValueError) as exc:
            # The old observation keeps its original timestamp and expires.
            self.last_scan_tf_error = str(exc)

    def _publish_observation(self, scan):
        obs = self.observation
        message = DockObservation()
        message.header.stamp = scan.header.stamp
        message.header.frame_id = self.values['base_frame']
        message.valid = message.center_available = obs.valid
        message.quality, message.visible_width = float(obs.quality), float(obs.width)
        message.fit_residual, message.reason = float(obs.residual), obs.reason
        if obs.pose is not None:
            message.pose.position.x, message.pose.position.y = obs.pose.x, obs.pose.y
            message.pose.orientation.z = math.sin(obs.pose.yaw / 2)
            message.pose.orientation.w = math.cos(obs.pose.yaw / 2)
        self.observation_pub.publish(message)

    def _request_owner(self, now):
        if not self.owner_client.service_is_ready():
            return
        if self.owner_future is not None and not self.owner_future.done():
            return
        if now - self.owner_request_time < 0.25:
            return
        request = SetBool.Request()
        request.data = True
        self.owner_future = self.owner_client.call_async(request)
        self.owner_request_time = now

    def _start_from_p0(self, now):
        p, c = self.values, self.core.config
        current, stamp = self._transform(p['global_frame'], p['base_frame'])
        local_base, local_stamp = self._transform(p['local_frame'], p['base_frame'])
        if not (age_valid(now, stamp, c.sensor_timeout)
                and age_valid(now, local_stamp, c.sensor_timeout)):
            raise ValueError('p0_tf_stale')
        target = pose2(self.goal.request.pre_dock_pose.pose)
        error = current.relative_to(target)
        vehicle = self.vehicle
        if (abs(error.x) > p['p0_longitudinal_tolerance']
                or abs(error.y) > p['p0_lateral_tolerance']
                or abs(error.yaw) > p['p0_yaw_tolerance']):
            raise ValueError('p0_outside_capture_region')
        if (vehicle is None or not age_valid(now, self.vehicle_time, c.sensor_timeout)
                or not vehicle.is_autodrive or not math.isfinite(vehicle.cur_speed)
                or abs(vehicle.cur_speed / 3.6) > c.stopped_speed):
            self.p0_stable_since = None
            return
        if self.p0_stable_since is None:
            self.p0_stable_since = now
        if now - self.p0_stable_since < c.stable_duration:
            return
        # Convert runtime map P0 to local frame at the same current base pose.
        local_map = local_base.compose(current.inverse())
        dock_prior = local_map.compose(target).compose(Pose2(-p['nominal_dock_distance']))
        self.tracker.reset(c.face_pose(dock_prior))
        self.core.start(now, self.goal.request.dry_run, self.goal.request.require_charging)

    def _tick(self):
        with self.lock:
            self._tick_locked()

    def _tick_locked(self):
        now = self._now()
        zero = Twist()
        if self.goal is None:
            self.command_pub.publish(zero)
            return
        self._request_owner(now)
        self._try_scan(now)
        owned = self.owned and age_valid(now, self.owned_time, 0.20)
        # A zero request/configuration disables that overall deadline. Honor
        # any explicitly positive limit without turning zero into a fallback.
        limits = [value for value in (self.goal.request.timeout_sec,
                                      self.core.config.timeout) if value > 0]
        limit = min(limits) if limits else 0.0
        expired = limit > 0 and now - self.requested_at > limit
        if self.core.state == 'IDLE':
            # P0 acquisition and measured control handoff remain bounded even
            # when the subsequent docking motion has no overall time limit.
            handoff_expired = now - self.requested_at > self.core.config.acquire_timeout
            if self.goal.is_cancel_requested or expired or handoff_expired:
                self._finish('CANCELED' if self.goal.is_cancel_requested else 'FAILED',
                             'cancelled_before_start' if self.goal.is_cancel_requested
                             else 'p0_or_control_handoff_timeout')
            elif owned:
                try:
                    self._start_from_p0(now)
                except (TransformException, ValueError) as exc:
                    self.p0_stable_since = None
                    self.state_pub.publish(String(data='WAIT_P0 reason=' + str(exc)))
            self.command_pub.publish(zero)
            return
        if self.goal.is_cancel_requested:
            self.core.cancel(now)
        elif (expired
              and self.core.state not in self.core.TERMINAL
              and self.core.state != 'STOPPING'):
            self.core._stop(now, 'FAILED', 'action_timeout')
        try:
            base, pose_stamp = self._transform(self.values['local_frame'],
                                               self.values['base_frame'])
        except (TransformException, ValueError):
            base, pose_stamp = None, -math.inf
        omega = math.inf
        if base is not None:
            if self.last_tf_time is not None and pose_stamp > self.last_tf_time:
                delta_t = pose_stamp - self.last_tf_time
                omega = wrap(base.yaw - self.last_tf_pose.yaw) / delta_t
                self.last_linear_speed = math.hypot(
                    base.x - self.last_tf_pose.x, base.y - self.last_tf_pose.y) / delta_t
                self.last_longitudinal_speed = longitudinal_speed(
                    self.last_tf_pose, base, delta_t)
                self.last_omega = omega
            else:
                omega = getattr(self, 'last_omega', 0.0)
            self.last_tf_pose, self.last_tf_time = base, pose_stamp
        vehicle = self.vehicle
        # Bring a timestamped scan observation forward to the current base.
        obs = self.observation
        scan_state = self.latest_scan
        scan_fresh = False
        if scan_state:
            scan = scan_state[0]
            scan_fresh = (age_valid(now, stamp_sec(scan.header.stamp),
                                    min(self.tracker.config.timeout,
                                        self.core.config.sensor_timeout))
                          and len(scan.ranges) >= 3
                          and all(math.isfinite(value) for value in
                                  (scan.angle_min, scan.angle_increment,
                                   scan.range_min, scan.range_max))
                          and scan.angle_increment != 0
                          and 0 <= scan.range_min < scan.range_max
                          and all(not math.isnan(value) and value != -math.inf
                                  for value in scan.ranges))
        if obs is not None and obs.pose is not None and base is not None and scan_state:
            obs = replace(obs, pose=scan_state[1].compose(obs.pose).relative_to(base))
        # Locked odometry replaces target visibility, never the rear scan or
        # its timestamped transforms. Keep an invalid-but-fresh real observation
        # when the complete outline leaves the field of view.
        if self.core.locked_dock_pose_local is not None and not scan_fresh:
            obs = None
        inputs = ControlInput(
            now=now, local_base=base, pose_stamp=pose_stamp, observation=obs,
            speed=max(abs(vehicle.cur_speed / 3.6), self.last_linear_speed)
            if vehicle else math.inf,
            omega=omega, vehicle_stamp=self.vehicle_time,
            autodrive=bool(vehicle and vehicle.is_autodrive), control_owned=owned,
            charging=self.charging, charge_stamp=self.charge_time)
        was_terminal = self.core.state in self.core.TERMINAL
        previous_state = self.core.state
        output = self.core.update(inputs)
        if (output.state == 'LOCKED_FINAL_APPROACH'
                and previous_state != 'LOCKED_FINAL_APPROACH'):
            self.get_logger().info(
                f'Actual stopped dock distance={self.core.locked_distance:.3f} m, '
                f'remaining reverse distance={self.core.locked_remaining_distance:.3f} m '
                f'(nominal stop trigger={self.core.config.lock_distance:.3f} m)')
        if base is not None:
            locked_dock = self.core.locked_dock_pose_local
            dock_pose = (locked_dock.relative_to(base) if locked_dock is not None else
                         obs.pose if obs is not None and obs.valid else None)
            bounds = tuple(self.values[name] for name in
                           ('footprint_front', 'footprint_rear', 'footprint_half_width'))
            # Sweep the conservative stopping interval, rather than projecting
            # through the dock for a full second after a valid final stop.
            sweep_velocity = -max(abs(output.velocity), abs(inputs.speed))
            # Constant-speed sweep with the equivalent stopping distance:
            # v*latency + v^2/(2*braking). This covers the full brake arc.
            sweep_horizon = max(0.05, self.core.config.latency
                                + abs(sweep_velocity) / (2 * self.core.config.braking))
            sweep_omega = (sweep_velocity * output.omega / output.velocity
                           if output.velocity else inputs.omega)
            sweeps = [(sweep_velocity, sweep_omega, sweep_horizon)]
            # Gear and the requested reverse command do not determine actual
            # travel direction. Cover measured forward roll as well as the
            # requested reverse arc, using the real yaw rate for that roll.
            if self.last_longitudinal_speed > 0:
                forward_speed = abs(inputs.speed)
                forward_horizon = max(
                    0.05, self.core.config.latency
                    + forward_speed / (2 * self.core.config.braking))
                sweeps.append((forward_speed, inputs.omega, forward_horizon))
            blocked = False
            if scan_state and scan_fresh:
                scan, scan_base, scan_sensor = scan_state
                local_sensor = scan_base.compose(scan_sensor)
                sensor_now = local_sensor.relative_to(base)
                target_indices = obs.target_indices if obs is not None and obs.valid else ()
                if locked_dock is not None:
                    target_indices = locked_target_indices(
                        scan, local_sensor, self.core.config.face_pose(locked_dock),
                        self.tracker.config.width, self.tracker.config.width_tolerance)
                blocked = any(collision(scan, sensor_now, target_indices,
                                        velocity, angular, bounds,
                                        self.values['collision_margin'], horizon)
                              for velocity, angular, horizon in sweeps)
            if dock_pose is not None:
                blocked = blocked or any(dock_plane_collision(
                    dock_pose, velocity, angular, bounds,
                    self.values['collision_margin'], horizon)
                    for velocity, angular, horizon in sweeps)
                # Validate the goal footprint as well as the imminent motion.
                goal = self.core.config.base_goal(dock_pose)
                blocked = blocked or dock_plane_collision(
                    dock_pose.relative_to(goal), 0.0, 0.0, bounds,
                    self.values['collision_margin'])
            if blocked:
                if (not was_terminal
                        and not (self.core.state == 'STOPPING'
                                 and self.core.final_state == 'FAILED')):
                    self.core._stop(now, 'FAILED', 'swept_footprint_obstacle')
                    output = self.core._output()
        command = Twist()
        if (owned and self.values['allow_motion'] and not self.goal.request.dry_run
                and self.core.config.calibrated):
            command.linear.x, command.angular.z = output.velocity, output.omega
        self.command_pub.publish(command)
        self.last_output = output
        self.state_pub.publish(String(data=output.state + ' reason=' + output.reason))
        feedback = Dock.Feedback()
        feedback.state, feedback.reason = output.state, output.reason
        feedback.traveled_distance = float(output.traveled)
        if output.error is not None:
            feedback.longitudinal_error = float(output.error.x)
            feedback.lateral_error = float(output.error.y)
            feedback.yaw_error = float(output.error.yaw)
        feedback.quality = float(obs.quality if obs else 0.0)
        self.goal.publish_feedback(feedback)
        if output.state in self.core.TERMINAL:
            self._finish(output.state, output.reason)

    def _finish(self, state, reason):
        if self.finished is None or self.finished.done():
            return
        result = Dock.Result()
        result.success = state == 'SUCCEEDED'
        result.dry_run_ready = state == 'DRY_RUN_READY'
        result.charging_confirmed = result.success and self.goal.request.require_charging
        result.error_code = '' if result.success or result.dry_run_ready else state
        result.message = reason
        error = self.core.error
        result.final_longitudinal_error = float(error.x if error else math.nan)
        result.final_lateral_error = float(error.y if error else math.nan)
        result.final_yaw_error = float(error.yaw if error else math.nan)
        self.command_pub.publish(Twist())
        self.finished.set_result(result)


def main(args=None):
    """Use one executor thread; action sleeps yield without blocking callbacks."""
    import rclpy
    rclpy.init(args=args)
    node = RelativeDockingNode()
    try:
        rclpy.spin(node)
    finally:
        node.command_pub.publish(Twist())
        node.action.destroy()
        node.destroy_node()
        rclpy.shutdown()
