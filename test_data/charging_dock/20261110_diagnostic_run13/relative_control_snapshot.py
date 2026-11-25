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

"""Relative dock pose control with bounded curvature and fail-closed states."""

from dataclasses import dataclass, field
import math

from charging_dock.relative_geometry import age_valid, Pose2


@dataclass
class DockConfig:
    """Mechanical transforms and motion limits, independent of map coordinates."""

    calibrated: bool = False
    base_receiver: Pose2 = field(default_factory=lambda: Pose2(-0.45, 0.0, 0.0))
    dock_receiver: Pose2 = field(default_factory=lambda: Pose2(0.01, 0.0, 0.0))
    face_to_dock: Pose2 = field(default_factory=Pose2)
    wheelbase: float = 0.5
    max_steer: float = math.radians(8.0)
    speed: float = 0.06
    final_speed: float = 0.025
    acceleration: float = 0.10
    braking: float = 0.10
    latency: float = 0.20
    final_zone: float = 0.50
    alignment_zone: float = 0.15
    longitudinal_tolerance: float = 0.01
    lateral_tolerance: float = 0.015
    yaw_tolerance: float = math.radians(1.0)
    capture_lateral: float = 0.30
    capture_yaw: float = math.radians(20.0)
    motion_limit: float = 2.8
    run_limit: float = 2.5
    timeout: float = 0.0  # Zero disables the overall action deadline.
    sensor_timeout: float = 0.20
    acquire_timeout: float = 8.0
    observation_reacquire_timeout: float = 2.0
    stable_duration: float = 0.5
    stopped_speed: float = 0.01
    stopped_omega: float = 0.02
    minimum_quality: float = 0.4
    stop_timeout: float = 3.0
    charge_timeout: float = 15.0
    charge_stable_duration: float = 2.0
    enable_locked_final: bool = True
    lock_distance: float = 1.0
    lock_distance_tolerance: float = 0.05
    lock_preposition_lead: float = 0.10
    lock_required_by_distance: float = 0.60
    lock_alignment_lateral: float = 0.005
    lock_alignment_yaw: float = math.radians(0.5)
    lock_alignment_duration: float = 0.5
    lock_alignment_entry_ratio: float = 1.0
    lock_retry_duration: float = 1.0
    lock_retry_distance: float = 0.01
    lock_verify_timeout: float = 3.0
    lock_min_observations: int = 3
    lock_pose_position_tolerance: float = 0.01
    lock_pose_yaw_tolerance: float = math.radians(1.0)
    locked_final_speed: float = 0.025
    locked_final_travel_limit: float = 0.70
    locked_final_timeout: float = 0.0  # Zero disables the locked-final deadline.
    locked_lateral_limit: float = 0.03
    locked_yaw_limit: float = math.radians(3.0)

    def __post_init__(self):
        """Reject invalid limits before arming any motion."""
        numeric = [v for v in vars(self).values() if isinstance(v, (float, int))]
        if not all(math.isfinite(v) for v in numeric):
            raise ValueError('non-finite docking configuration')
        positive = (self.wheelbase, self.max_steer, self.speed, self.final_speed,
                    self.acceleration, self.braking, self.final_zone,
                    self.alignment_zone, self.longitudinal_tolerance,
                    self.lateral_tolerance, self.yaw_tolerance,
                    self.capture_lateral, self.capture_yaw, self.motion_limit,
                    self.run_limit, self.sensor_timeout,
                    self.acquire_timeout, self.observation_reacquire_timeout,
                    self.stop_timeout, self.stopped_speed, self.stopped_omega,
                    self.charge_timeout, self.lock_distance,
                    self.lock_distance_tolerance, self.lock_preposition_lead,
                    self.lock_required_by_distance,
                    self.lock_alignment_lateral, self.lock_alignment_yaw,
                    self.lock_alignment_duration, self.lock_retry_duration,
                    self.lock_retry_distance,
                    self.lock_verify_timeout, self.lock_pose_position_tolerance,
                    self.lock_pose_yaw_tolerance, self.locked_final_speed,
                    self.locked_final_travel_limit,
                    self.locked_lateral_limit, self.locked_yaw_limit)
        if (min(positive) <= 0 or self.run_limit > self.motion_limit
                or self.final_speed > self.speed or self.max_steer >= math.pi / 2
                or self.capture_yaw >= math.pi / 2 or self.latency < 0
                or self.timeout < 0 or self.locked_final_timeout < 0
                or self.stable_duration < 0 or self.charge_stable_duration < 0
                or self.lock_min_observations < 1
                or self.lock_required_by_distance >= self.lock_distance
                or self.lock_preposition_lead >= self.lock_distance
                or self.lock_alignment_lateral > self.capture_lateral
                or self.lock_alignment_yaw > self.capture_yaw
                or not 0 < self.lock_alignment_entry_ratio <= 1
                or self.locked_final_speed > self.speed
                or not 0 <= self.minimum_quality <= 1
                or not self.base_receiver.finite() or not self.dock_receiver.finite()
                or not self.face_to_dock.finite()):
            raise ValueError('invalid docking limits or mechanical transforms')

    def dock_pose(self, observed_face):
        """Project the measured face onto the physical charging reference."""
        return observed_face.compose(self.face_to_dock)

    def face_pose(self, dock):
        """Locate the measured face from a physical charging reference prior."""
        return dock.compose(self.face_to_dock.inverse())

    def base_goal(self, base_dock):
        """Convert the physical charging reference to the desired base pose."""
        return base_dock.compose(self.dock_receiver).compose(self.base_receiver.inverse())


@dataclass
class ControlInput:
    """Timestamped state already expressed at the vehicle control reference."""

    now: float
    local_base: Pose2 = None
    pose_stamp: float = -math.inf
    observation: object = None
    speed: float = 0.0
    omega: float = 0.0
    vehicle_stamp: float = -math.inf
    autodrive: bool = False
    obstacle: bool = False
    control_owned: bool = False
    charging: bool = False
    charge_stamp: float = -math.inf


@dataclass
class ControlOutput:
    """Zero is the default for every state that does not authorize motion."""

    state: str
    reason: str = ''
    velocity: float = 0.0
    omega: float = 0.0
    traveled: float = 0.0
    error: Pose2 = None


class RelativeDockingCore:
    """Replan a reverse cubic approach to the live target on each control tick."""

    TERMINAL = frozenset({'SUCCEEDED', 'FAILED', 'CANCELED',
                          'DRY_RUN_READY', 'LIMITED_RUN_COMPLETE'})

    def __init__(self, config=None):
        self.config = config or DockConfig()
        self.state = 'IDLE'
        self.reason = ''
        self.error = None
        self.traveled = 0.0

    def start(self, now, dry_run=True, require_charge=False):
        """Start a new attempt; caller must also reset its perception tracker."""
        self.state = 'PRECHECK'
        self.reason = ''
        self.started = self.last_time = now
        self.stable_since = None
        self.last_pose = None
        self.traveled = 0.0
        self.velocity = 0.0
        self.dry_run = dry_run
        self.require_charge = require_charge
        self.error = None
        self.final_state = None
        self.observation_hold_started = None
        self.observation_reacquired_since = None
        self.observation_resume_state = None
        self.locked_dock_pose_local = None
        self.locked_distance = None
        self.locked_remaining_distance = None
        self._reset_lock_alignment()
        self.lock_retry_started = None
        self.lock_retry_base = None
        self.lock_observations = []
        self.lock_verify_started = None
        self.lock_stationary_since = None
        self.lock_stable_since = None
        self.lock_last_observation_stamp = -math.inf
        self.locked_final_started = None
        self.locked_final_traveled = 0.0

    def cancel(self, now, reason='cancelled'):
        """Cancel through the same measured-stop confirmation as failures."""
        if (self.state not in self.TERMINAL and self.state != 'IDLE'
                and not (self.state == 'STOPPING' and self.final_state == 'CANCELED')):
            self._stop(now, 'CANCELED', reason)

    def _stop(self, now, terminal, reason):
        self.state = 'STOPPING'
        self.final_state = terminal
        self.stop_started = now
        self.stable_since = None
        self.reason = reason
        self.velocity = 0.0

    def _output(self, velocity=0.0, omega=0.0):
        return ControlOutput(self.state, self.reason, velocity, omega,
                             self.traveled, self.error)

    def _fault(self, i, allow_observation_loss=False):
        c = self.config
        if (i.local_base is None or not i.local_base.finite()
                or not age_valid(i.now, i.pose_stamp, c.sensor_timeout)):
            return 'vehicle_pose_stale'
        if (not age_valid(i.now, i.vehicle_stamp, c.sensor_timeout)
                or not all(math.isfinite(v) for v in (i.speed, i.omega))):
            return 'vehicle_feedback_stale'
        if not i.autodrive:
            return 'manual_control'
        if not i.control_owned:
            return 'control_not_owned'
        if i.obstacle:
            return 'obstacle'
        obs = i.observation
        if obs is None or not age_valid(i.now, obs.stamp, c.sensor_timeout):
            return 'dock_observation_invalid'
        if allow_observation_loss:
            # A complete target contour may leave the scanner's field, but a
            # malformed/out-of-order scan is not evidence of healthy sensing.
            if obs.valid:
                return ''
            if getattr(obs, 'reason', '') not in (
                    'no_complete_face_in_gate', 'ambiguous_faces', 'warming_up'):
                return 'dock_observation_invalid'
            return ''
        if not obs.valid or obs.pose is None or not obs.pose.finite():
            return 'dock_observation_invalid'
        if not math.isfinite(obs.quality) or obs.quality < c.minimum_quality:
            return 'low_dock_quality'
        return ''

    @staticmethod
    def _average_pose(poses):
        """Average nearby local-frame dock poses, including wrapped yaw."""
        x = sum(p.x for p in poses) / len(poses)
        y = sum(p.y for p in poses) / len(poses)
        yaw = math.atan2(sum(math.sin(p.yaw) for p in poses),
                         sum(math.cos(p.yaw) for p in poses))
        return Pose2(x, y, yaw)

    def _locked_error(self, local_base):
        """Re-express the immutable local dock pose at the current base pose."""
        dock_base = self.locked_dock_pose_local.relative_to(local_base)
        return self.config.base_goal(dock_base).inverse()

    def _collect_lock_observation(self, i):
        """Build a stable local dock pose from distinct fresh lidar samples."""
        obs = i.observation
        if (obs is None or not obs.valid or obs.pose is None
                or not obs.pose.finite() or obs.stamp <= self.lock_last_observation_stamp
                or obs.stamp < self.lock_stationary_since):
            return
        if (self.lock_observations and obs.stamp - self.lock_last_observation_stamp
                > self.config.sensor_timeout):
            self._reset_lock_verification()
        self.lock_last_observation_stamp = obs.stamp
        measured = i.local_base.compose(obs.pose)
        if self.lock_observations:
            reference = self._average_pose(self.lock_observations)
            position_delta = math.hypot(measured.x - reference.x,
                                        measured.y - reference.y)
            yaw_delta = abs(math.atan2(math.sin(measured.yaw - reference.yaw),
                                       math.cos(measured.yaw - reference.yaw)))
            if (position_delta > self.config.lock_pose_position_tolerance
                    or yaw_delta > self.config.lock_pose_yaw_tolerance):
                self.lock_observations = [measured]
                self.lock_stable_since = obs.stamp
                return
        else:
            self.lock_stable_since = obs.stamp
        self.lock_observations.append(measured)

    def _reset_lock_verification(self):
        self.lock_observations = []
        self.lock_stable_since = None
        self.lock_last_observation_stamp = -math.inf

    def _lock_alignment_ok(self, error=None):
        error = self.error if error is None else error
        return (error is not None
                and abs(error.y) <= self.config.lock_alignment_lateral
                and abs(error.yaw) <= self.config.lock_alignment_yaw)

    def _reset_lock_alignment(self):
        self.lock_alignment_samples = []
        self.lock_alignment_last_stamp = -math.inf

    def _lock_ready_to_stop(self, i):
        """Require sustained alignment and a real correction before retrying."""
        c, obs = self.config, i.observation
        if self.lock_retry_started is not None:
            displacement = math.hypot(i.local_base.x - self.lock_retry_base.x,
                                      i.local_base.y - self.lock_retry_base.y)
            if (i.now - self.lock_retry_started < c.lock_retry_duration
                    or displacement < c.lock_retry_distance):
                self._reset_lock_alignment()
                return False
        # Timer ticks and a frozen scan cannot extend the evidence window.
        if obs.stamp <= self.lock_alignment_last_stamp:
            return False
        if obs.stamp - self.lock_alignment_last_stamp > c.sensor_timeout:
            self._reset_lock_alignment()
        self.lock_alignment_last_stamp = obs.stamp
        if not self._lock_alignment_ok():
            self._reset_lock_alignment()
            return False
        ratio = c.lock_alignment_entry_ratio

        def entry_ok(error):
            return (abs(error.y) <= c.lock_alignment_lateral * ratio
                    and abs(error.yaw) <= c.lock_alignment_yaw * ratio)

        # An optional tighter entry gate permits hysteresis without changing
        # the acceptance gate. Defaults preserve the existing capture range.
        if not self.lock_alignment_samples and not entry_ok(self.error):
            return False
        measured = i.local_base.compose(obs.pose)
        if self.lock_alignment_samples:
            reference = self._average_pose([p for _, p in self.lock_alignment_samples])
            delta_yaw = math.atan2(math.sin(measured.yaw - reference.yaw),
                                   math.cos(measured.yaw - reference.yaw))
            if (math.hypot(measured.x - reference.x, measured.y - reference.y)
                    > c.lock_pose_position_tolerance
                    or abs(delta_yaw) > c.lock_pose_yaw_tolerance):
                self._reset_lock_alignment()
                self.lock_alignment_last_stamp = obs.stamp
                if not entry_ok(self.error):
                    return False
        self.lock_alignment_samples.append((obs.stamp, measured))
        # Bound the moving window, keeping the sample immediately before its
        # cutoff so distinct scan timestamps still span the full duration.
        while (len(self.lock_alignment_samples) > c.lock_min_observations
               and obs.stamp - self.lock_alignment_samples[1][0]
               >= c.lock_alignment_duration):
            self.lock_alignment_samples.pop(0)
        if (len(self.lock_alignment_samples) < c.lock_min_observations
                or obs.stamp - self.lock_alignment_samples[0][0]
                < c.lock_alignment_duration):
            return False
        average = self._average_pose([p for _, p in self.lock_alignment_samples])
        error = c.base_goal(average.relative_to(i.local_base)).inverse()
        return self._lock_alignment_ok(error)

    def _enter_lock_verification(self, now):
        self.state = 'LOCK_VERIFY'
        self.reason = 'waiting_for_measured_stop_before_lock'
        self.lock_verify_started = now
        self.lock_stationary_since = None
        self._reset_lock_alignment()
        self._reset_lock_verification()
        self.velocity = 0.0

    def _finish_lock_verification(self, i, dock_pose):
        self.locked_dock_pose_local = dock_pose
        self.error = self._locked_error(i.local_base)
        self.locked_distance = -dock_pose.relative_to(i.local_base).x
        self.locked_remaining_distance = self.error.x
        self.locked_final_started = i.now
        self.locked_final_traveled = self.traveled
        self.state = 'LOCKED_FINAL_APPROACH'
        self.reason = 'locked_target_confirmed'
        self.velocity = 0.0

    def _aligned(self):
        c, e = self.config, self.error
        return (e is not None and abs(e.x) <= c.longitudinal_tolerance
                and abs(e.y) <= c.lateral_tolerance
                and abs(e.yaw) <= c.yaw_tolerance)

    def _account_motion(self, i, dt):
        """Track measured displacement and enforce the configured run limit."""
        if self.last_pose is not None:
            delta = math.hypot(i.local_base.x - self.last_pose.x,
                               i.local_base.y - self.last_pose.y)
            if delta > max(0.05, self.config.speed * 2 * dt + 0.02):
                return 'odometry_jump'
            self.traveled += delta
        self.last_pose = i.local_base
        return ''

    def _motion_limit_fault(self, i):
        """Reject measured motion beyond the speed and yaw-rate envelope."""
        if abs(i.speed) > self.config.speed * 1.5 + 0.01 or abs(i.omega) > 0.20:
            return 'actual_motion_limit'
        return ''

    def update(self, i):
        """Compute one control sample; lost targets hold zero pending reacquisition."""
        c = self.config
        if self.state == 'IDLE' or self.state in self.TERMINAL:
            return self._output()
        if not math.isfinite(i.now) or i.now < self.last_time:
            self._stop(self.last_time, 'FAILED', 'clock_discontinuity')
            return self._output()
        dt = i.now - self.last_time
        self.last_time = i.now
        locked_mode = (self.locked_dock_pose_local is not None
                       and self.state in ('LOCKED_FINAL_APPROACH', 'STOPPING',
                                          'WAIT_CHARGE'))
        fault = self._fault(i, allow_observation_loss=locked_mode)
        if locked_mode and i.local_base is not None and i.local_base.finite():
            # After a stable lock, live odometry determines the error to the
            # immutable dock pose even when the full face is out of view.
            self.error = self._locked_error(i.local_base)
        elif (not fault and i.observation is not None
              and i.observation.valid and i.observation.pose is not None):
            goal = c.base_goal(i.observation.pose)
            # Current base in goal frame: x>0 means there is room to reverse.
            self.error = goal.inverse()
        if self.state == 'STOPPING':
            stopped = (age_valid(i.now, i.vehicle_stamp, c.sensor_timeout)
                       and math.isfinite(i.speed) and math.isfinite(i.omega)
                       and abs(i.speed) <= c.stopped_speed
                       and abs(i.omega) <= c.stopped_omega)
            if self.final_state == 'SUCCEEDED' and (fault or not self._aligned()):
                self.final_state = 'FAILED'
                self.reason = fault or 'final_alignment_lost'
            if stopped:
                if self.stable_since is None:
                    self.stable_since = i.now
                if i.now - self.stable_since >= c.stable_duration:
                    if self.final_state == 'SUCCEEDED' and self.require_charge:
                        self.state = 'WAIT_CHARGE'
                        self.charge_started = i.now
                        self.stable_since = None
                    else:
                        self.state = self.final_state
            else:
                self.stable_since = None
            if self.state == 'STOPPING' and i.now - self.stop_started > c.stop_timeout:
                self.state, self.reason = 'FAILED', 'stop_not_confirmed'
            return self._output()
        if self.state == 'WAIT_CHARGE':
            if (fault or not self._aligned() or abs(i.speed) > c.stopped_speed
                    or abs(i.omega) > c.stopped_omega):
                self._stop(i.now, 'FAILED', fault or 'alignment_lost_while_charging')
            elif (i.charging and age_valid(i.now, i.charge_stamp, c.sensor_timeout)):
                if self.stable_since is None:
                    self.stable_since = i.now
                if i.now - self.stable_since >= c.charge_stable_duration:
                    self.state, self.reason = 'SUCCEEDED', 'charging_confirmed'
            else:
                self.stable_since = None
            if self.state == 'WAIT_CHARGE' and i.now - self.charge_started > c.charge_timeout:
                self._stop(i.now, 'FAILED', 'charging_timeout')
            return self._output()
        if c.timeout > 0 and i.now - self.started > c.timeout:
            self._stop(i.now, 'FAILED', 'timeout')
            return self._output()
        if self.state == 'OBSERVATION_HOLD':
            if fault and fault not in ('dock_observation_invalid', 'low_dock_quality'):
                self._stop(i.now, 'FAILED', fault)
                return self._output()
            motion_fault = self._motion_limit_fault(i)
            if motion_fault:
                self._stop(i.now, 'FAILED', motion_fault)
                return self._output()
            motion_fault = self._account_motion(i, dt)
            if motion_fault:
                self._stop(i.now, 'FAILED', motion_fault)
                return self._output()
            if i.now - self.observation_hold_started >= c.observation_reacquire_timeout:
                self._stop(i.now, 'FAILED', 'dock_observation_reacquire_timeout')
                return self._output()
            if self.traveled >= min(c.run_limit, c.motion_limit):
                self._stop(i.now, 'LIMITED_RUN_COMPLETE', 'travel_limit')
                return self._output()
            if fault:
                self.reason = fault
                self.observation_reacquired_since = None
                self.velocity = 0.0
                return self._output()
            if abs(i.speed) > c.stopped_speed or abs(i.omega) > c.stopped_omega:
                self.reason = 'waiting_for_measured_stop'
                self.observation_reacquired_since = None
                self.velocity = 0.0
                return self._output()
            if self.observation_reacquired_since is None:
                self.observation_reacquired_since = i.now
            if i.now - self.observation_reacquired_since < c.stable_duration:
                self.reason = 'waiting_for_stable_reacquisition'
                self.velocity = 0.0
                return self._output()
            self.state = self.observation_resume_state
            self.reason = ''
            self.observation_hold_started = None
            self.observation_reacquired_since = None
            self.observation_resume_state = None
        if fault:
            if (self.state in ('APPROACH', 'FINAL_APPROACH')
                    and fault in ('dock_observation_invalid', 'low_dock_quality')):
                motion_fault = self._motion_limit_fault(i)
                if motion_fault:
                    self._stop(i.now, 'FAILED', motion_fault)
                    return self._output()
                motion_fault = self._account_motion(i, dt)
                if motion_fault:
                    self._stop(i.now, 'FAILED', motion_fault)
                    return self._output()
                if self.traveled >= min(c.run_limit, c.motion_limit):
                    self._stop(i.now, 'LIMITED_RUN_COMPLETE', 'travel_limit')
                    return self._output()
                self.observation_resume_state = self.state
                self._reset_lock_alignment()
                self.observation_hold_started = i.now
                self.observation_reacquired_since = None
                self.state, self.reason = 'OBSERVATION_HOLD', fault
                self.velocity = 0.0
            elif (self.state == 'LOCK_VERIFY'
                  and fault in ('dock_observation_invalid', 'low_dock_quality')):
                motion_fault = self._motion_limit_fault(i)
                if motion_fault:
                    self._stop(i.now, 'FAILED', motion_fault)
                    return self._output()
                motion_fault = self._account_motion(i, dt)
                if motion_fault:
                    self._stop(i.now, 'FAILED', motion_fault)
                    return self._output()
                if self.traveled >= min(c.run_limit, c.motion_limit):
                    self._stop(i.now, 'LIMITED_RUN_COMPLETE', 'travel_limit')
                    return self._output()
                self._reset_lock_verification()
                self.lock_stationary_since = None
                self.observation_resume_state = 'LOCK_VERIFY'
                self.observation_hold_started = i.now
                self.observation_reacquired_since = None
                self.state, self.reason = 'OBSERVATION_HOLD', fault
                self.velocity = 0.0
            elif self.state == 'PRECHECK' and fault not in ('obstacle', 'manual_control'):
                self.stable_since = None
                self.reason = fault
                if i.now - self.started > c.acquire_timeout:
                    self._stop(i.now, 'FAILED', 'precheck:' + fault)
            else:
                self._stop(i.now, 'FAILED', fault)
            return self._output()
        e = self.error
        if abs(e.y) > c.capture_lateral or abs(e.yaw) > c.capture_yaw:
            self._stop(i.now, 'FAILED', 'outside_capture_region')
            return self._output()
        if self.state == 'PRECHECK':
            if not self.dry_run and not c.calibrated:
                self._stop(i.now, 'FAILED', 'mechanical_calibration_required')
                return self._output()
            if abs(i.speed) > c.stopped_speed or abs(i.omega) > c.stopped_omega:
                self.stable_since = None
                if i.now - self.started > c.acquire_timeout:
                    self._stop(i.now, 'FAILED', 'initial_vehicle_not_stopped')
                return self._output()
            if self.stable_since is None:
                self.stable_since = i.now
            if i.now - self.stable_since < c.stable_duration:
                return self._output()
            if self.dry_run:
                self.state, self.reason = 'DRY_RUN_READY', 'sensors_and_target_valid'
                return self._output()
            self.last_pose = i.local_base
            self.state, self.reason = 'APPROACH', ''
        control_error = e
        if (c.enable_locked_final and self.state != 'LOCKED_FINAL_APPROACH'
                and i.observation is not None and i.observation.valid
                and i.observation.pose is not None
                and -i.observation.pose.x
                > c.lock_distance + c.lock_preposition_lead
                + c.lock_distance_tolerance):
            # Shape the live-lidar approach toward a virtual pose at the lock
            # distance. This leaves room to correct heading/lateral error
            # before the target enters the scanner's near-field blind zone.
            live_dock = i.observation.pose
            final_goal_in_dock = c.base_goal(Pose2())
            lock_goal_in_dock = Pose2(c.lock_distance + c.lock_preposition_lead,
                                      final_goal_in_dock.y,
                                      final_goal_in_dock.yaw)
            virtual_goal = live_dock.compose(lock_goal_in_dock)
            control_error = virtual_goal.inverse()
        motion_fault = self._motion_limit_fault(i)
        if motion_fault:
            self._stop(i.now, 'FAILED', motion_fault)
            return self._output()
        motion_fault = self._account_motion(i, dt)
        if motion_fault:
            self._stop(i.now, 'FAILED', motion_fault)
            return self._output()
        kmax = math.tan(c.max_steer) / c.wheelbase
        if self.state == 'LOCK_VERIFY':
            if self.traveled >= min(c.run_limit, c.motion_limit):
                self._stop(i.now, 'LIMITED_RUN_COMPLETE', 'travel_limit')
                return self._output()
            if i.now - self.lock_verify_started > c.lock_verify_timeout:
                self._stop(i.now, 'FAILED', 'lock_verification_timeout')
                return self._output()
            if abs(i.speed) > c.stopped_speed or abs(i.omega) > c.stopped_omega:
                self._reset_lock_verification()
                self.lock_stationary_since = None
                self.reason = 'waiting_for_measured_stop_before_lock'
                return self._output()
            if self.lock_stationary_since is None:
                self.lock_stationary_since = i.now
            # Braking can change the range from the nominal trigger distance.
            # Only new observations acquired after stopping may establish the
            # actual dock pose. Judge alignment from their stable average,
            # rather than restarting the vehicle for one noisy frame.
            self._collect_lock_observation(i)
            self.reason = 'collecting_stopped_dock_pose'
            if (len(self.lock_observations) < c.lock_min_observations
                    or self.lock_stable_since is None
                    or self.lock_last_observation_stamp - self.lock_stable_since
                    < c.stable_duration):
                return self._output()
            candidate = self._average_pose(self.lock_observations)
            candidate_base = candidate.relative_to(i.local_base)
            candidate_error = c.base_goal(candidate_base).inverse()
            if self._lock_alignment_ok(candidate_error):
                self._finish_lock_verification(i, candidate)
                return self._output()
            self.error = e = candidate_error
            self.lock_retry_started = i.now
            self.lock_retry_base = i.local_base
            self._reset_lock_alignment()
            self._reset_lock_verification()
            self.lock_stationary_since = None
            if -candidate_base.x <= c.lock_required_by_distance:
                self._stop(i.now, 'FAILED', 'lock_alignment_not_achieved')
                return self._output()
            # Resume a meaningful correction, then require a fresh sustained
            # alignment window before asking the chassis to brake again.
            self.state = 'APPROACH' if e.x >= c.final_zone else 'FINAL_APPROACH'
            self.reason = 'lock_alignment_lost_resume_tracking'

        locked_final = self.state == 'LOCKED_FINAL_APPROACH'
        if locked_final:
            if abs(i.speed) > c.locked_final_speed * 1.5 + 0.01:
                self._stop(i.now, 'FAILED', 'locked_final_speed_limit')
                return self._output()
            if (c.locked_final_timeout > 0
                    and i.now - self.locked_final_started > c.locked_final_timeout):
                self._stop(i.now, 'FAILED', 'locked_final_timeout')
                return self._output()
            if (abs(e.y) > c.locked_lateral_limit
                    or abs(e.yaw) > c.locked_yaw_limit):
                self._stop(i.now, 'FAILED', 'locked_alignment_lost')
                return self._output()
        if e.x < -c.longitudinal_tolerance:
            self._stop(i.now, 'FAILED', 'target_overshoot')
        elif self._aligned():
            self._stop(i.now, 'SUCCEEDED', 'geometric_alignment')
        elif self.traveled >= min(c.run_limit, c.motion_limit):
            self._stop(i.now, 'LIMITED_RUN_COMPLETE', 'travel_limit')
        elif (locked_final and self.traveled - self.locked_final_traveled
              >= c.locked_final_travel_limit):
            self._stop(i.now, 'FAILED', 'locked_final_travel_limit')
        elif e.x <= c.longitudinal_tolerance:
            self._stop(i.now, 'FAILED', 'final_alignment_failed')
        elif (e.x < c.alignment_zone
              and (abs(e.y) > c.lateral_tolerance + kmax * e.x**2 / 2
                   or abs(e.yaw) > c.yaw_tolerance + kmax * e.x)):
            self._stop(i.now, 'FAILED', 'insufficient_alignment_space')
        if self.state == 'STOPPING':
            return self._output()
        if c.enable_locked_final and not locked_final:
            face_distance = -i.observation.pose.x
            # The deadline in distance is independent of retry suppression.
            if face_distance <= c.lock_required_by_distance:
                self._stop(i.now, 'FAILED', 'lock_alignment_not_achieved')
                return self._output()
            ready_to_stop = self._lock_ready_to_stop(i)
            if (face_distance <= c.lock_distance + c.lock_distance_tolerance
                    and ready_to_stop):
                self._enter_lock_verification(i.now)
                return self._output()
        # Cubic y(x): y(0)=y'(0)=0, y(x)=e.y, y'(x)=tan(e.yaw).
        # omega=v*kappa remains correct for negative (reverse) velocity.
        path_error = control_error if control_error.x > c.longitudinal_tolerance else e
        x = max(path_error.x, c.longitudinal_tolerance)
        slope = math.tan(path_error.yaw)
        curvature = (4 * slope / x - 6 * path_error.y / x**2) / (1 + slope**2)**1.5
        curvature = max(-kmax, min(curvature, kmax))
        if locked_final:
            speed = c.locked_final_speed
        else:
            self.state = 'FINAL_APPROACH' if x < c.final_zone else 'APPROACH'
            speed = c.final_speed if x < c.final_zone else c.speed
        remaining = max(0.0, x - c.longitudinal_tolerance / 2)
        safe_speed = math.sqrt((c.braking * c.latency)**2 + 2 * c.braking * remaining)
        safe_speed -= c.braking * c.latency
        speed = min(speed, safe_speed, abs(self.velocity) + c.acceleration * dt)
        self.velocity = -max(0.0, speed)
        return self._output(self.velocity, self.velocity * curvature)
