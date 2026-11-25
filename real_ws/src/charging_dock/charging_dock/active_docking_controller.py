# Copyright 2026 akun
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""Fail-closed Stage 5 fixed-trajectory docking controller."""

from dataclasses import dataclass
import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import SetBool
from yunle_msgs.msg import VehicleStatus

from charging_dock.docking_controller import DockingStaticCore
from charging_dock.fixed_path_tracker import FixedPathTrackerCore, wrap_angle
from charging_dock.rear_lidar_processor import RearLidarProcessor


@dataclass
class ActiveDockingConfig:
    """Parameters used by the ROS-independent active docking state machine."""

    motion_enabled: bool = False
    planned_reverse_distance_m: float = 2.04
    run_distance_limit_m: float = 0.10
    target_speed_mps: float = 0.06
    final_speed_mps: float = 0.025
    final_approach_distance_m: float = 0.90
    stop_trigger_distance_m: float = 0.47
    stop_distance_min_m: float = 0.45
    stop_distance_max_m: float = 0.47
    expected_lidar_angle_deg: float = 0.0
    max_motion_distance_m: float = 2.30
    max_duration_sec: float = 60.0
    max_reverse_speed_mps: float = 0.10
    max_actual_speed_mps: float = 0.15
    max_actual_steer_deg: float = 10.0
    max_steer_omega: float = 0.35
    final_max_steer_omega: float = 0.10
    max_lateral_error_m: float = 0.10
    max_yaw_error_deg: float = 3.0
    max_accel_mps2: float = 0.15
    k_lateral: float = 0.8
    k_yaw: float = 1.2
    steering_sign: float = 1.0
    yaw_correction_enabled: bool = False
    lidar_yaw_gain: float = 0.5
    lidar_yaw_sign: float = 1.0
    lateral_correction_enabled: bool = False
    expected_lateral_center_m: float = 0.0
    lidar_lateral_gain: float = 0.5
    lidar_lateral_sign: float = 1.0
    odom_timeout_sec: float = 0.20
    scan_timeout_sec: float = 0.20
    vehicle_status_timeout_sec: float = 0.20
    acquire_timeout_sec: float = 5.0
    precheck_stable_duration_sec: float = 1.0
    stopped_speed_mps: float = 0.02
    stop_stable_duration_sec: float = 0.50
    stop_confirmation_timeout_sec: float = 2.0
    lidar_odom_consistency_tolerance_m: float = 0.15
    start_pose_validation_enabled: bool = False
    p0_target_x: float = 0.0
    p0_target_y: float = 0.0
    p0_target_yaw: float = 0.0
    p0_longitudinal_tolerance_m: float = 0.10
    p0_lateral_tolerance_m: float = 0.05
    p0_yaw_tolerance_deg: float = 1.0


@dataclass
class ActiveDockingInputs:
    """Latest synchronized-enough inputs evaluated on one control tick."""

    now_sec: float
    odom: tuple
    odom_age_sec: float
    vehicle_speed_mps: float
    vehicle_steer_deg: float
    vehicle_autodrive: bool
    vehicle_age_sec: float
    lidar_result: object
    scan_age_sec: float


@dataclass
class ActiveDockingOutput:
    """One command and status sample from the active docking core."""

    command: Twist
    state: str
    reason: str
    traveled_m: float
    path_lateral_error_m: float
    path_yaw_error_rad: float
    p0_longitudinal_error_m: float
    p0_lateral_error_m: float
    p0_yaw_error_rad: float


class ActiveDockingCore:
    """Combine fixed-path tracking, lidar stopping, and safety interlocks."""

    TERMINAL_STATES = frozenset({
        'SUCCEEDED', 'FAILED', 'LIMITED_RUN_COMPLETE', 'DRY_RUN_READY',
    })

    def __init__(self, config=None):
        self.config = config or ActiveDockingConfig()
        self._validate_config()
        self.tracker = self._new_tracker()
        self.armed = False
        self.state = 'IDLE'
        self.reason = ''
        self.enable_time = None
        self.precheck_stable_since = None
        self.stop_started = None
        self.stop_stable_since = None
        self.pending_terminal = None
        self.initial_lidar_distance_m = None
        self.initial_base_distance_m = None
        self.traveled_m = 0.0
        self.path_lateral_error_m = 0.0
        self.path_yaw_error_rad = 0.0
        self.p0_longitudinal_error_m = 0.0
        self.p0_lateral_error_m = 0.0
        self.p0_yaw_error_rad = 0.0

    def _validate_config(self):
        c = self.config
        if c.planned_reverse_distance_m <= 0.0:
            raise ValueError('planned_reverse_distance_m must be positive')
        if c.run_distance_limit_m <= 0.0:
            raise ValueError('run_distance_limit_m must be positive')
        if c.run_distance_limit_m > c.max_motion_distance_m:
            raise ValueError(
                'run_distance_limit_m exceeds max_motion_distance_m')
        if c.planned_reverse_distance_m > c.max_motion_distance_m:
            raise ValueError(
                'planned_reverse_distance_m exceeds max_motion_distance_m')
        if c.stop_distance_min_m > c.stop_distance_max_m:
            raise ValueError('invalid lidar stop window')
        if c.stop_trigger_distance_m < c.stop_distance_min_m:
            raise ValueError('stop trigger is beyond the minimum safe distance')

    def _new_tracker(self):
        c = self.config
        return FixedPathTrackerCore(
            reverse_distance=c.planned_reverse_distance_m,
            target_speed=c.target_speed_mps,
            final_speed=c.final_speed_mps,
            slow_zone_distance=c.final_approach_distance_m,
            max_reverse_speed=c.max_reverse_speed_mps,
            max_steer_omega=c.max_steer_omega,
            k_lateral=c.k_lateral,
            k_yaw=c.k_yaw,
            steering_sign=c.steering_sign,
            max_lateral_error=c.max_lateral_error_m,
            max_yaw_error_deg=c.max_yaw_error_deg,
            max_motion_distance=c.max_motion_distance_m,
            max_duration_sec=c.max_duration_sec,
            odom_timeout_sec=c.odom_timeout_sec,
            max_accel_mps2=c.max_accel_mps2,
        )

    @staticmethod
    def zero_command():
        return Twist()

    def enable(self, now_sec):
        """Arm a new attempt; motion still waits for all prechecks."""
        self.tracker = self._new_tracker()
        self.armed = True
        self.state = 'PRECHECK'
        self.reason = ''
        self.enable_time = float(now_sec)
        self.precheck_stable_since = None
        self.stop_started = None
        self.stop_stable_since = None
        self.pending_terminal = None
        self.initial_lidar_distance_m = None
        self.initial_base_distance_m = None
        self.traveled_m = 0.0
        self.path_lateral_error_m = 0.0
        self.path_yaw_error_rad = 0.0
        self.p0_longitudinal_error_m = 0.0
        self.p0_lateral_error_m = 0.0
        self.p0_yaw_error_rad = 0.0

    def disable(self):
        """Cancel an attempt and keep producing zero commands."""
        self.tracker.stop()
        self.armed = False
        self.state = 'IDLE'
        self.reason = 'cancelled'
        self.pending_terminal = None

    @property
    def tracking(self):
        return self.state in ('REVERSE_APPROACH', 'FINAL_APPROACH')

    def update(self, inputs):
        """Evaluate one fail-closed control cycle."""
        if not self.armed:
            return self._output(self.zero_command())
        if self.state in self.TERMINAL_STATES:
            return self._output(self.zero_command())
        if self.state == 'STOPPING':
            return self._update_stopping(inputs)
        if self.state in ('PRECHECK', 'ACQUIRE_TARGET'):
            return self._update_precheck(inputs)
        return self._update_motion(inputs)

    def _input_error(self, inputs):
        c = self.config
        if inputs.odom is None or inputs.odom_age_sec > c.odom_timeout_sec:
            return 'odometry_timeout'
        if inputs.vehicle_age_sec > c.vehicle_status_timeout_sec:
            return 'vehicle_status_timeout'
        if not inputs.vehicle_autodrive:
            return 'autodrive_disabled'
        if inputs.scan_age_sec > c.scan_timeout_sec:
            return 'rear_lidar_timeout'
        result = inputs.lidar_result
        if result is None:
            return 'no_lidar_result'
        if result.obstacle:
            return 'non_target_obstacle'
        if not result.observation.valid:
            return result.observation.reason or 'invalid_lidar_target'
        if result.state == 'DRY_RUN_LOW_QUALITY':
            return 'low_lidar_quality'
        if result.state == 'DRY_RUN_ANGLE_OUT_OF_RANGE':
            return 'lidar_angle_out_of_range'
        if (self.config.lateral_correction_enabled
                and not result.observation.center_available):
            return 'lidar_lateral_center_unavailable'
        return None

    def _update_precheck(self, inputs):
        error = self._input_error(inputs)
        elapsed = inputs.now_sec - self.enable_time
        if error:
            self.precheck_stable_since = None
            self.state = 'ACQUIRE_TARGET'
            self.reason = error
            if error in ('autodrive_disabled', 'non_target_obstacle'):
                return self._finish_without_motion(error)
            if elapsed > self.config.acquire_timeout_sec:
                return self._finish_without_motion('precheck_timeout:' + error)
            return self._output(self.zero_command())

        observation = inputs.lidar_result.observation
        distance = observation.base_link_plane_normal_distance_m
        if distance is None:
            return self._finish_without_motion('missing_lidar_distance')
        if distance < self.config.stop_distance_min_m:
            return self._finish_without_motion('initial_position_too_close')
        if abs(inputs.vehicle_speed_mps) > self.config.stopped_speed_mps:
            self.precheck_stable_since = None
            self.state = 'PRECHECK'
            self.reason = 'vehicle_not_stopped'
            if elapsed > self.config.acquire_timeout_sec:
                return self._finish_without_motion('precheck_timeout:vehicle_not_stopped')
            return self._output(self.zero_command())
        if abs(inputs.vehicle_steer_deg) > self.config.max_actual_steer_deg:
            return self._finish_without_motion('initial_steer_out_of_range')
        if not self._validate_start_pose(inputs.odom):
            return self._finish_without_motion('start_pose_out_of_range')

        if self.precheck_stable_since is None:
            self.precheck_stable_since = inputs.now_sec
        stable_time = inputs.now_sec - self.precheck_stable_since
        self.state = 'PRECHECK'
        self.reason = 'waiting_for_stable_start'
        if stable_time < self.config.precheck_stable_duration_sec:
            return self._output(self.zero_command())

        if inputs.lidar_result.in_stop_window:
            self._begin_stop(inputs.now_sec, 'SUCCEEDED', 'already_in_stop_window')
            return self._output(self.zero_command())
        if not self.config.motion_enabled:
            self.state = 'DRY_RUN_READY'
            self.reason = 'motion_interlock_disabled'
            return self._output(self.zero_command())

        self.initial_lidar_distance_m = observation.distance_m
        self.initial_base_distance_m = distance
        self.tracker.start(*inputs.odom, inputs.now_sec)
        self.state = 'REVERSE_APPROACH'
        self.reason = ''
        return self._output(self.zero_command())

    def _update_motion(self, inputs):
        error = self._input_error(inputs)
        if error:
            return self._stop_for_failure(inputs.now_sec, error)
        c = self.config
        if abs(inputs.vehicle_speed_mps) > c.max_actual_speed_mps:
            return self._stop_for_failure(inputs.now_sec, 'actual_speed_limit')
        if abs(inputs.vehicle_steer_deg) > c.max_actual_steer_deg:
            return self._stop_for_failure(inputs.now_sec, 'actual_steer_limit')

        observation = inputs.lidar_result.observation
        distance = observation.base_link_plane_normal_distance_m
        if distance is None:
            return self._stop_for_failure(inputs.now_sec, 'missing_lidar_distance')
        if distance < c.stop_distance_min_m:
            return self._stop_for_failure(inputs.now_sec, 'stop_window_overshoot')
        if distance <= c.stop_trigger_distance_m:
            self._begin_stop(inputs.now_sec, 'SUCCEEDED', 'lidar_stop_trigger')
            return self._output(self.zero_command())

        tracker_output = self.tracker.update(
            *inputs.odom, inputs.now_sec, odom_age_sec=inputs.odom_age_sec)
        self.traveled_m = tracker_output.traveled_m
        self.path_lateral_error_m = tracker_output.lateral_error_m
        self.path_yaw_error_rad = tracker_output.yaw_error_rad
        if tracker_output.state == 'FAILED':
            return self._stop_for_failure(
                inputs.now_sec, 'tracker:' + tracker_output.reason)
        if tracker_output.state == 'SUCCEEDED':
            return self._stop_for_failure(
                inputs.now_sec, 'odometry_target_reached_without_lidar')
        if self.traveled_m >= c.run_distance_limit_m:
            self._begin_stop(
                inputs.now_sec, 'LIMITED_RUN_COMPLETE',
                'configured_run_distance_reached')
            return self._output(self.zero_command())

        predicted_distance = self.initial_base_distance_m - self.traveled_m
        if abs(predicted_distance - distance) > c.lidar_odom_consistency_tolerance_m:
            return self._stop_for_failure(
                inputs.now_sec, 'lidar_odometry_inconsistent')

        command = self._corrected_command(tracker_output.command, observation)
        if distance <= c.final_approach_distance_m:
            self.state = 'FINAL_APPROACH'
            command.linear.x = -min(abs(command.linear.x), c.final_speed_mps)
            command.angular.z = max(
                -c.final_max_steer_omega,
                min(command.angular.z, c.final_max_steer_omega),
            )
        else:
            self.state = 'REVERSE_APPROACH'
        self.reason = ''
        return self._output(command)

    def _corrected_command(self, tracker_command, observation):
        command = Twist()
        command.linear.x = tracker_command.linear.x
        command.angular.z = 0.0
        c = self.config
        if c.yaw_correction_enabled:
            angle = observation.base_link_yaw_deg
            if angle is not None:
                error = math.radians(self._line_angle_error_deg(
                    angle, c.expected_lidar_angle_deg))
                command.angular.z = tracker_command.angular.z + (
                    c.lidar_yaw_sign * c.lidar_yaw_gain * error)
        if c.lateral_correction_enabled:
            error = observation.lateral_center_m - c.expected_lateral_center_m
            command.angular.z += (
                c.lidar_lateral_sign * c.lidar_lateral_gain * error)
        command.angular.z = max(
            -c.max_steer_omega, min(command.angular.z, c.max_steer_omega))
        return command

    @staticmethod
    def _line_angle_error_deg(angle, expected):
        return (float(angle) - float(expected) + 90.0) % 180.0 - 90.0

    def _validate_start_pose(self, odom):
        c = self.config
        if not c.start_pose_validation_enabled:
            return True
        x, y, yaw = odom
        dx = x - c.p0_target_x
        dy = y - c.p0_target_y
        cosine = math.cos(c.p0_target_yaw)
        sine = math.sin(c.p0_target_yaw)
        self.p0_longitudinal_error_m = cosine * dx + sine * dy
        self.p0_lateral_error_m = -sine * dx + cosine * dy
        self.p0_yaw_error_rad = wrap_angle(yaw - c.p0_target_yaw)
        return (
            abs(self.p0_longitudinal_error_m)
            <= c.p0_longitudinal_tolerance_m
            and abs(self.p0_lateral_error_m) <= c.p0_lateral_tolerance_m
            and abs(self.p0_yaw_error_rad)
            <= math.radians(c.p0_yaw_tolerance_deg)
        )

    def _begin_stop(self, now_sec, terminal_state, reason):
        self.tracker.stop()
        self.state = 'STOPPING'
        self.reason = reason
        self.pending_terminal = terminal_state
        self.stop_started = float(now_sec)
        self.stop_stable_since = None

    def _stop_for_failure(self, now_sec, reason):
        self._begin_stop(now_sec, 'FAILED', reason)
        return self._output(self.zero_command())

    def _update_stopping(self, inputs):
        c = self.config
        if self.pending_terminal == 'SUCCEEDED':
            error = self._input_error(inputs)
            if error:
                self.pending_terminal = 'FAILED'
                self.reason = 'verification:' + error
            elif not inputs.lidar_result.in_stop_window:
                distance = inputs.lidar_result.observation.base_link_plane_normal_distance_m
                if distance is not None and distance < c.stop_distance_min_m:
                    self.pending_terminal = 'FAILED'
                    self.reason = 'verification:stop_window_overshoot'

        vehicle_fresh = inputs.vehicle_age_sec <= c.vehicle_status_timeout_sec
        stopped = vehicle_fresh and (
            abs(inputs.vehicle_speed_mps) <= c.stopped_speed_mps)
        if stopped:
            if self.stop_stable_since is None:
                self.stop_stable_since = inputs.now_sec
            stable = inputs.now_sec - self.stop_stable_since
            if stable >= c.stop_stable_duration_sec:
                if (self.pending_terminal == 'SUCCEEDED'
                        and not inputs.lidar_result.in_stop_window):
                    self.state = 'FAILED'
                    self.reason = 'final_distance_outside_stop_window'
                else:
                    self.state = self.pending_terminal
                return self._output(self.zero_command())
        else:
            self.stop_stable_since = None

        if inputs.now_sec - self.stop_started > c.stop_confirmation_timeout_sec:
            self.state = 'FAILED'
            self.reason = 'stop_not_confirmed:' + self.reason
        return self._output(self.zero_command())

    def _finish_without_motion(self, reason):
        self.tracker.stop()
        self.state = 'FAILED'
        self.reason = reason
        return self._output(self.zero_command())

    def _output(self, command):
        return ActiveDockingOutput(
            command=command,
            state=self.state,
            reason=self.reason,
            traveled_m=self.traveled_m,
            path_lateral_error_m=self.path_lateral_error_m,
            path_yaw_error_rad=self.path_yaw_error_rad,
            p0_longitudinal_error_m=self.p0_longitudinal_error_m,
            p0_lateral_error_m=self.p0_lateral_error_m,
            p0_yaw_error_rad=self.p0_yaw_error_rad,
        )


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ActiveDockingController(Node):
    """ROS adapter that owns the only Stage 5 ``/cmd_vel_dock`` publisher."""

    def __init__(self):
        super().__init__('active_docking_controller')
        self._declare_parameters()
        p = self.get_parameter
        motion_enabled = (
            not p('dry_run').value and p('publish_nonzero_command').value)
        config = ActiveDockingConfig(
            motion_enabled=motion_enabled,
            planned_reverse_distance_m=p('planned_reverse_distance_m').value,
            run_distance_limit_m=p('run_distance_limit_m').value,
            target_speed_mps=p('target_speed_mps').value,
            final_speed_mps=p('final_speed_mps').value,
            final_approach_distance_m=p('final_approach_distance_m').value,
            stop_trigger_distance_m=p('stop_trigger_distance_m').value,
            stop_distance_min_m=p('stop_distance_min_m').value,
            stop_distance_max_m=p('stop_distance_max_m').value,
            expected_lidar_angle_deg=p('expected_angle_deg').value,
            max_motion_distance_m=p('max_motion_distance_m').value,
            max_duration_sec=p('max_duration_sec').value,
            max_reverse_speed_mps=p('max_reverse_speed_mps').value,
            max_actual_speed_mps=p('max_actual_speed_mps').value,
            max_actual_steer_deg=p('max_actual_steer_deg').value,
            max_steer_omega=p('max_steer_omega').value,
            final_max_steer_omega=p('final_max_steer_omega').value,
            max_lateral_error_m=p('max_lateral_error_m').value,
            max_yaw_error_deg=p('max_yaw_error_deg').value,
            max_accel_mps2=p('max_accel_mps2').value,
            k_lateral=p('k_lateral').value,
            k_yaw=p('k_yaw').value,
            steering_sign=p('steering_sign').value,
            yaw_correction_enabled=p('yaw_correction_enabled').value,
            lidar_yaw_gain=p('lidar_yaw_gain').value,
            lidar_yaw_sign=p('lidar_yaw_sign').value,
            lateral_correction_enabled=p('lateral_correction_enabled').value,
            expected_lateral_center_m=p('expected_lateral_center_m').value,
            lidar_lateral_gain=p('lidar_lateral_gain').value,
            lidar_lateral_sign=p('lidar_lateral_sign').value,
            odom_timeout_sec=p('odom_timeout_sec').value,
            scan_timeout_sec=p('scan_timeout_sec').value,
            vehicle_status_timeout_sec=p('vehicle_status_timeout_sec').value,
            acquire_timeout_sec=p('acquire_timeout_sec').value,
            precheck_stable_duration_sec=p('precheck_stable_duration_sec').value,
            stopped_speed_mps=p('stopped_speed_mps').value,
            stop_stable_duration_sec=p('stop_stable_duration_sec').value,
            stop_confirmation_timeout_sec=p('stop_confirmation_timeout_sec').value,
            lidar_odom_consistency_tolerance_m=(
                p('lidar_odom_consistency_tolerance_m').value),
            start_pose_validation_enabled=p('start_pose_validation_enabled').value,
            p0_target_x=p('p0_target_x').value,
            p0_target_y=p('p0_target_y').value,
            p0_target_yaw=p('p0_target_yaw').value,
            p0_longitudinal_tolerance_m=p('p0_longitudinal_tolerance_m').value,
            p0_lateral_tolerance_m=p('p0_lateral_tolerance_m').value,
            p0_yaw_tolerance_deg=p('p0_yaw_tolerance_deg').value,
        )
        self.core = ActiveDockingCore(config)
        self.processor = RearLidarProcessor(
            p('roi_min_deg').value,
            p('roi_max_deg').value,
            expected_range_m=p('expected_range_m').value,
            minimum_points=p('minimum_points').value,
            stable_frames=p('stable_frames').value,
            maximum_age_sec=p('scan_timeout_sec').value,
            expected_range_tolerance_m=p('expected_range_tolerance_m').value,
            max_distance_jump_m=p('max_distance_jump_m').value,
            min_width_m=p('min_target_width_m').value,
            max_width_m=p('max_target_width_m').value,
            max_fit_residual_m=p('max_fit_residual_m').value,
            expected_frames=(p('expected_scan_frame').value,),
            lateral_center_enabled=p('lateral_correction_enabled').value,
            sensor_x_m=p('sensor_x_m').value,
            sensor_y_m=p('sensor_y_m').value,
            sensor_yaw_deg=p('sensor_yaw_deg').value,
        )
        self.lidar_core = DockingStaticCore(
            self.processor,
            p('stop_distance_min_m').value,
            p('stop_distance_max_m').value,
            p('expected_angle_deg').value,
            p('angle_tolerance_deg').value,
            p('minimum_quality').value,
            p('obstacle_range_m').value,
            p('roi_min_deg').value,
            p('roi_max_deg').value,
            p('target_intrusion_margin_m').value,
        )
        self.last_odom = None
        self.last_odom_time = None
        self.last_vehicle = None
        self.last_vehicle_time = None
        self.last_lidar_result = None
        self.last_scan_time = None
        self._create_interfaces()
        if not motion_enabled:
            self.get_logger().warn(
                'Stage 5 motion is interlocked; dry_run and allow_motion '
                'must explicitly permit non-zero commands')

    def _declare_parameters(self):
        defaults = {
            'scan_topic': '/laser/scan_back',
            'odom_topic': '/lio_sam/mapping/odometry',
            'vehicle_status_topic': '/vehicle_status',
            'command_topic': '/cmd_vel_dock',
            'state_topic': '/charging_dock/state',
            'pose_topic': '/charging_dock/lidar_pose',
            'quality_topic': '/charging_dock/lidar_quality',
            'control_frequency': 25.0,
            'dry_run': True,
            'publish_nonzero_command': False,
            'planned_reverse_distance_m': 2.04,
            'run_distance_limit_m': 0.10,
            'target_speed_mps': 0.06,
            'final_speed_mps': 0.025,
            'final_approach_distance_m': 0.90,
            'stop_trigger_distance_m': 0.47,
            'stop_distance_min_m': 0.45,
            'stop_distance_max_m': 0.47,
            'max_motion_distance_m': 2.30,
            'max_duration_sec': 60.0,
            'max_reverse_speed_mps': 0.10,
            'max_actual_speed_mps': 0.15,
            'max_actual_steer_deg': 10.0,
            'max_steer_omega': 0.35,
            'final_max_steer_omega': 0.10,
            'max_lateral_error_m': 0.10,
            'max_yaw_error_deg': 3.0,
            'max_accel_mps2': 0.15,
            'k_lateral': 0.8,
            'k_yaw': 1.2,
            'steering_sign': 1.0,
            'yaw_correction_enabled': False,
            'lidar_yaw_gain': 0.5,
            'lidar_yaw_sign': 1.0,
            'lateral_correction_enabled': False,
            'expected_lateral_center_m': 0.0,
            'lidar_lateral_gain': 0.5,
            'lidar_lateral_sign': 1.0,
            'odom_timeout_sec': 0.20,
            'scan_timeout_sec': 0.20,
            'vehicle_status_timeout_sec': 0.20,
            'acquire_timeout_sec': 5.0,
            'precheck_stable_duration_sec': 1.0,
            'stopped_speed_mps': 0.02,
            'stop_stable_duration_sec': 0.50,
            'stop_confirmation_timeout_sec': 2.0,
            'lidar_odom_consistency_tolerance_m': 0.15,
            'start_pose_validation_enabled': False,
            'p0_target_x': 0.0,
            'p0_target_y': 0.0,
            'p0_target_yaw': 0.0,
            'p0_longitudinal_tolerance_m': 0.10,
            'p0_lateral_tolerance_m': 0.05,
            'p0_yaw_tolerance_deg': 1.0,
            'roi_min_deg': -30.0,
            'roi_max_deg': 30.0,
            'expected_range_m': 2.5,
            'expected_range_tolerance_m': 0.35,
            'expected_angle_deg': 0.0,
            'angle_tolerance_deg': 5.0,
            'minimum_quality': 0.0,
            'minimum_points': 5,
            'stable_frames': 10,
            'max_distance_jump_m': 0.10,
            'min_target_width_m': 0.05,
            'max_target_width_m': 0.60,
            'max_fit_residual_m': 0.01,
            'obstacle_range_m': 1.0,
            'target_intrusion_margin_m': 0.15,
            'expected_scan_frame': 'rear_laser',
            'sensor_x_m': -0.43,
            'sensor_y_m': -0.28,
            'sensor_yaw_deg': -135.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _create_interfaces(self):
        p = self.get_parameter
        self.command_pub = self.create_publisher(
            Twist, p('command_topic').value, 10)
        self.state_pub = self.create_publisher(
            String, p('state_topic').value, 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, p('pose_topic').value, 10)
        self.quality_pub = self.create_publisher(
            Float32MultiArray, p('quality_topic').value, 10)
        self.create_subscription(
            Odometry, p('odom_topic').value, self._odom_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            VehicleStatus, p('vehicle_status_topic').value,
            self._vehicle_callback, 10)
        self.create_subscription(
            LaserScan, p('scan_topic').value, self._scan_callback, 10)
        self.create_service(SetBool, '~/enable', self._enable_callback)
        frequency = max(10.0, float(p('control_frequency').value))
        self.create_timer(1.0 / frequency, self._control_callback)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _odom_callback(self, message):
        position = message.pose.pose.position
        self.last_odom = (
            position.x,
            position.y,
            _yaw_from_quaternion(message.pose.pose.orientation),
        )
        self.last_odom_time = self._now_sec()

    def _vehicle_callback(self, message):
        self.last_vehicle = (
            float(message.cur_speed) / 3.6,
            float(message.cur_steer),
            bool(message.is_autodrive),
        )
        self.last_vehicle_time = self._now_sec()

    def _scan_callback(self, message):
        if (self.core.tracking
                and self.core.initial_lidar_distance_m is not None):
            self.processor.expected_range_m = max(
                0.05,
                self.core.initial_lidar_distance_m - self.core.traveled_m,
            )
        now_ns = self.get_clock().now().nanoseconds
        self.last_lidar_result = self.lidar_core.evaluate(message, now_ns)
        self.last_scan_time = now_ns / 1e9
        self._publish_lidar_debug(message, self.last_lidar_result)

    def _publish_lidar_debug(self, message, result):
        observation = result.observation
        quality = Float32MultiArray()
        quality.data = [
            float(observation.quality),
            float(observation.target_points),
            float(observation.fit_residual_m or 0.0),
            float(observation.base_link_plane_normal_distance_m or 0.0),
            float(observation.base_link_yaw_deg or 0.0),
            1.0 if observation.center_available else 0.0,
        ]
        self.quality_pub.publish(quality)
        if not observation.valid:
            return
        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(
            observation.base_link_plane_normal_distance_m or 0.0)
        pose.pose.position.y = float(observation.lateral_center_m or 0.0)
        yaw = math.radians(float(observation.base_link_yaw_deg or 0.0))
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        self.pose_pub.publish(pose)

    def _enable_callback(self, request, response):
        if not request.data:
            self.core.disable()
            response.success = True
            response.message = 'docking cancelled; zero command held'
            return response
        if self.last_odom is None:
            response.success = False
            response.message = 'no odometry available'
            return response
        if self.last_vehicle is None:
            response.success = False
            response.message = 'no vehicle status available'
            return response
        if self.last_lidar_result is None:
            response.success = False
            response.message = 'no rear lidar result available'
            return response
        self.core.enable(self._now_sec())
        response.success = True
        response.message = 'docking armed; prechecks in progress'
        return response

    def _control_callback(self):
        now = self._now_sec()
        vehicle = self.last_vehicle or (0.0, 0.0, False)
        inputs = ActiveDockingInputs(
            now_sec=now,
            odom=self.last_odom,
            odom_age_sec=(
                math.inf if self.last_odom_time is None
                else now - self.last_odom_time),
            vehicle_speed_mps=vehicle[0],
            vehicle_steer_deg=vehicle[1],
            vehicle_autodrive=vehicle[2],
            vehicle_age_sec=(
                math.inf if self.last_vehicle_time is None
                else now - self.last_vehicle_time),
            lidar_result=self.last_lidar_result,
            scan_age_sec=(
                math.inf if self.last_scan_time is None
                else now - self.last_scan_time),
        )
        output = self.core.update(inputs)
        command = output.command
        if not self.core.config.motion_enabled:
            command = Twist()
        self.command_pub.publish(command)
        status = String()
        status.data = (
            f'{output.state} reason={output.reason} '
            f'traveled={output.traveled_m:.3f} '
            f'path_lateral={output.path_lateral_error_m:.3f} '
            f'path_yaw_deg={math.degrees(output.path_yaw_error_rad):.2f} '
            f'p0_long={output.p0_longitudinal_error_m:.3f} '
            f'p0_lat={output.p0_lateral_error_m:.3f} '
            f'p0_yaw_deg={math.degrees(output.p0_yaw_error_rad):.2f}')
        self.state_pub.publish(status)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = ActiveDockingController()
    try:
        rclpy.spin(node)
    finally:
        node.core.disable()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
