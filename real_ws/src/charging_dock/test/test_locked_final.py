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

"""Behavioural coverage for locking the dock before the lidar near-field blind zone."""

from dataclasses import replace
import math

import pytest

from charging_dock.relative_control import ControlInput, DockConfig, RelativeDockingCore
from charging_dock.relative_geometry import Pose2
from charging_dock.relative_lidar import FaceObservation


DOCK = Pose2(-2.5)
FINAL_BASE_X = DOCK.x + .46


def data(t, base=Pose2(), dock=DOCK, speed=0., observation=True, **changes):
    """Build fresh samples in a fixed local odometry frame."""
    if observation is True:
        pose = dock.relative_to(base)
        obs = FaceObservation(t, True, pose, .9, .25, .003, 'ok')
    elif observation is False:
        obs = FaceObservation(t, False, None, 0., 0., 0., 'no_complete_face_in_gate')
    else:
        obs = observation
    values = dict(now=t, local_base=base, pose_stamp=t, observation=obs,
                  speed=speed, omega=0., vehicle_stamp=t, autodrive=True,
                  obstacle=False, control_owned=True)
    values.update(changes)
    return ControlInput(**values)


def start_approach(config=None, dock=DOCK):
    core = RelativeDockingCore(config or DockConfig(calibrated=True, run_limit=2.8))
    core.start(0., dry_run=False)
    assert core.update(data(0., dock=dock)).state == 'PRECHECK'
    assert core.update(data(.6, dock=dock)).state == 'APPROACH'
    return core


def approach_to_lock(core, dock=DOCK, t=.64):
    """Approach in small odometry steps, stopping when the core asks to verify."""
    base = Pose2()
    for _ in range(170):
        out = core.update(data(t, base=base, dock=dock, speed=-.025))
        if out.state == 'LOCK_VERIFY':
            assert out.velocity == out.omega == 0.
            assert -data(t, base=base, dock=dock).observation.pose.x <= 1.05
            return t, base
        assert out.state not in core.TERMINAL, out
        # The test plant follows the commanded reverse distance while holding
        # a fixed small pose error; this isolates the lock transition itself.
        base = Pose2(base.x - .01)
        t += .04
    pytest.fail('core did not enter lock verification near the configured distance')


def verify_lock(core, t, base, dock=DOCK):
    """Supply unique, stationary, stable observations to complete the lock."""
    out = None
    for _ in range(20):
        t += .04
        out = core.update(data(t, base=base, dock=dock))
        if out.state == 'LOCKED_FINAL_APPROACH':
            return t
        assert out.velocity == out.omega == 0.
        assert out.state == 'LOCK_VERIFY', out
    pytest.fail('stable pose observations did not complete lock verification')


def approach_without_alignment(core, dock=Pose2(DOCK.x, .006)):
    """Reach the preset with a persistent error, without authorizing a stop."""
    base = Pose2()
    t = .6
    for _ in range(150):
        t += .04
        base = Pose2(base.x - .01)
        out = core.update(data(t, base=base, dock=dock, speed=-.025))
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
        assert out.velocity < 0.
        assert core.locked_dock_pose_local is None
    assert -DOCK.relative_to(base).x == pytest.approx(1.)
    return t, base


def reject_stopped_alignment(core):
    """Enter verification while aligned, then measure a persistently biased stop."""
    t, base = approach_to_lock(core)
    for _ in range(20):
        t += .04
        out = core.update(data(t, base=base, dock=Pose2(DOCK.x, .014)))
        assert core.locked_dock_pose_local is None
        if out.state != 'LOCK_VERIFY':
            assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
            assert out.reason == 'lock_alignment_lost_resume_tracking'
            assert out.velocity < 0.
            return t, base
        assert out.velocity == out.omega == 0.
    pytest.fail('a persistently unaligned stopped pose did not resume correction')


def test_locked_final_finishes_after_full_face_leaves_scan_and_target_is_met():
    core = start_approach()
    t, base = approach_to_lock(core)
    t = verify_lock(core, t, base)

    # The complete face disappears after the lock. Odometry still measures the
    # remaining ~0.54m, and the controller must continue at bounded low speed.
    for n in range(1, 56):
        t += .04
        x = base.x + (FINAL_BASE_X - base.x) * n / 55
        stopped = n == 55
        out = core.update(data(t, base=Pose2(x), dock=DOCK,
                               speed=0. if stopped else -.025,
                               observation=False))
        assert out.state in ('LOCKED_FINAL_APPROACH', 'STOPPING', 'SUCCEEDED'), out
        assert out.velocity <= 0.
        assert abs(out.velocity) <= core.config.locked_final_speed + 1e-9
        if n < 55:
            assert out.velocity < 0.
    assert out.state == 'STOPPING'
    assert core.final_state == 'SUCCEEDED'
    assert out.velocity == out.omega == 0.
    assert core.update(data(t + .04, base=Pose2(FINAL_BASE_X),
                            observation=False)).state == 'STOPPING'
    assert core.update(data(t + .60, base=Pose2(FINAL_BASE_X),
                            observation=False)).state == 'SUCCEEDED'


def test_coasting_past_lock_preset_uses_actual_stopped_pose_and_remaining_distance():
    core = start_approach()
    t, base = approach_to_lock(core)
    assert -(DOCK.relative_to(base)).x == pytest.approx(1.04, abs=.011)

    # Braking starts near the 1m preset, but measured reverse motion continues
    # until the base is actually 0.92m from the charging reference. A biased
    # face estimate seen during braking must not seed the stationary lock.
    stopped_base = Pose2(DOCK.x + .92)
    braking_dock = Pose2(DOCK.x + .025, .003)
    while base.x > stopped_base.x + 1e-9:
        t += .10
        base = Pose2(max(stopped_base.x, base.x - .006))
        out = core.update(data(t, base=base, dock=braking_dock, speed=-.06))
        assert out.state == 'LOCK_VERIFY', out
        assert out.velocity == out.omega == 0.
        assert core.locked_dock_pose_local is None

    t = verify_lock(core, t, base)
    assert core.locked_dock_pose_local.x == pytest.approx(DOCK.x)
    assert core.locked_dock_pose_local.y == pytest.approx(DOCK.y)
    assert core.locked_dock_pose_local.yaw == pytest.approx(DOCK.yaw)
    assert -core.locked_dock_pose_local.relative_to(base).x == pytest.approx(.92)
    assert core.locked_distance == pytest.approx(.92)
    assert core.locked_remaining_distance == pytest.approx(.46)
    assert core.error.x == pytest.approx(.92 - .46)

    # The entire remaining 0.46m is now referenced to that measured stopped
    # pose, even if every subsequent scan lacks a complete face.
    lock_base = base
    for n in range(1, 56):
        t += .04
        base = Pose2(lock_base.x + (FINAL_BASE_X - lock_base.x) * n / 55)
        out = core.update(data(t, base=base, observation=False,
                               speed=0. if n == 55 else -.025))
        assert out.state in ('LOCKED_FINAL_APPROACH', 'STOPPING'), out
        assert out.velocity <= 0.
    assert out.state == 'STOPPING'
    assert core.final_state == 'SUCCEEDED'
    assert abs(out.error.x) <= core.config.longitudinal_tolerance
    core.update(data(t + .04, base=base, observation=False))
    assert core.update(data(t + .60, base=base,
                            observation=False)).state == 'SUCCEEDED'


def test_one_unaligned_stationary_frame_does_not_abort_stable_pose_average():
    core = start_approach()
    t, base = approach_to_lock(core)
    stationary_y = []
    for n in range(20):
        t += .04
        # A single 9mm estimate exceeds the 5mm lock alignment gate, but is
        # close enough to the other 2mm samples to remain geometrically stable.
        y = .009 if n == 3 else .002
        stationary_y.append(y)
        out = core.update(data(t, base=base, dock=Pose2(DOCK.x, y)))
        assert out.velocity == out.omega == 0.
        if out.state == 'LOCKED_FINAL_APPROACH':
            break
        assert out.state == 'LOCK_VERIFY', out
        assert core.locked_dock_pose_local is None
    assert out.state == 'LOCKED_FINAL_APPROACH'
    assert core.locked_dock_pose_local.y == pytest.approx(
        sum(stationary_y) / len(stationary_y))
    assert abs(core.locked_dock_pose_local.y) < core.config.lock_alignment_lateral


def test_stable_unaligned_stop_resumes_live_correction_then_retries_when_aligned():
    misaligned_dock = Pose2(DOCK.x, .014)
    core = start_approach()
    t, base = reject_stopped_alignment(core)
    assert core.locked_dock_pose_local is None

    # A rejected stop must leave time and measured motion for correction.
    # Merely spending timer ticks behind the preset cannot re-arm braking.
    for _ in range(30):
        t += .04
        base = Pose2(base.x - .001)
        out = core.update(data(t, base=base, dock=misaligned_dock))
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
        assert out.velocity < 0.

    # Even after retry motion has completed, one good estimate cannot stop
    # the vehicle. Continuous aligned samples then permit a fresh attempt.
    for n in range(20):
        t += .04
        out = core.update(data(t, base=base))
        if out.state == 'LOCK_VERIFY':
            assert n >= 12
            break
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
        assert out.velocity < 0.
        assert core.locked_dock_pose_local is None
    assert out.state == 'LOCK_VERIFY'
    assert out.velocity == out.omega == 0.
    verify_lock(core, t, base)
    assert core.state == 'LOCKED_FINAL_APPROACH'


def test_noise_crossing_lock_alignment_gate_does_not_repeat_stop_requests():
    core = start_approach()
    t, base = approach_without_alignment(core)
    for n in range(80):
        t += .04
        noisy_dock = Pose2(DOCK.x, .006 if n % 2 else .004)
        out = core.update(data(t, base=base, dock=noisy_dock, speed=-.025))
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
        assert out.velocity < 0.
        assert core.locked_dock_pose_local is None


def test_one_good_frame_does_not_stop_but_sustained_alignment_can_lock():
    core = start_approach()
    t, base = approach_without_alignment(core)
    t += .04
    out = core.update(data(t, base=base, dock=Pose2(DOCK.x, .003)))
    assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    assert out.velocity < 0.

    for n in range(20):
        t += .04
        out = core.update(data(t, base=base, dock=Pose2(DOCK.x, .003)))
        if out.state == 'LOCK_VERIFY':
            assert n >= 11
            break
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    assert out.state == 'LOCK_VERIFY'
    verify_lock(core, t, base, dock=Pose2(DOCK.x, .003))
    assert core.state == 'LOCKED_FINAL_APPROACH'


def test_first_aligned_frame_stops_before_stationary_lock_confirmation():
    core = start_approach(DockConfig(
        calibrated=True, run_limit=2.8, lock_stop_on_first_alignment=True))
    t, base = approach_without_alignment(core)

    t += .04
    out = core.update(data(t, base=base, dock=Pose2(DOCK.x, .003)))
    assert out.state == 'LOCK_VERIFY'
    assert out.reason == 'waiting_for_measured_stop_before_lock'
    assert out.velocity == out.omega == 0.
    assert core.locked_dock_pose_local is None

    # The triggering frame only requests a stop. A distinct stationary scan
    # window is still required before odometry-only final approach is allowed.
    for _ in range(8):
        t += .04
        out = core.update(data(t, base=base, dock=Pose2(DOCK.x, .003)))
        assert out.state == 'LOCK_VERIFY'
        assert core.locked_dock_pose_local is None
    verify_lock(core, t, base, dock=Pose2(DOCK.x, .003))
    assert core.state == 'LOCKED_FINAL_APPROACH'


def test_first_alignment_stop_does_not_stop_on_an_unaligned_frame():
    core = start_approach(DockConfig(
        calibrated=True, run_limit=2.8, lock_stop_on_first_alignment=True))
    t, base = approach_without_alignment(core)
    for _ in range(20):
        t += .04
        out = core.update(data(t, base=base, dock=Pose2(DOCK.x, .006)))
        assert out.state in ('APPROACH', 'FINAL_APPROACH')
        assert out.velocity < 0.
    assert core.locked_dock_pose_local is None


def test_lock_entry_hysteresis_requires_tighter_alignment_to_start_window():
    core = start_approach(DockConfig(calibrated=True, run_limit=2.8,
                                     lock_alignment_entry_ratio=.8))
    t, base = approach_without_alignment(core)
    between_gates = Pose2(DOCK.x, .0045)
    for _ in range(20):
        t += .04
        out = core.update(data(t, base=base, dock=between_gates))
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out

    # Start with margin, then preserve the original 5mm acceptance gate for
    # the complete window and the subsequent measured, stationary lock.
    t += .04
    assert core.update(data(t, base=base, dock=Pose2(DOCK.x, .0038))).state != 'LOCK_VERIFY'
    for n in range(20):
        t += .04
        out = core.update(data(t, base=base, dock=between_gates))
        if out.state == 'LOCK_VERIFY':
            assert n >= 11
            break
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    assert out.state == 'LOCK_VERIFY'
    verify_lock(core, t, base, dock=between_gates)
    assert core.state == 'LOCKED_FINAL_APPROACH'
    assert core.locked_dock_pose_local.y == pytest.approx(.0045)


def test_default_alignment_window_preserves_original_acceptance_tolerance():
    core = start_approach()
    assert core.config.lock_alignment_entry_ratio == 1.
    t, base = approach_without_alignment(core)
    dock = Pose2(DOCK.x, .0045)
    for n in range(20):
        t += .04
        out = core.update(data(t, base=base, dock=dock))
        if out.state == 'LOCK_VERIFY':
            assert n >= 12
            break
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    assert out.state == 'LOCK_VERIFY'
    verify_lock(core, t, base, dock=dock)
    assert core.state == 'LOCKED_FINAL_APPROACH'
    assert core.locked_dock_pose_local.y == pytest.approx(.0045)


def test_repeated_fresh_observation_does_not_complete_alignment_entry_window():
    core = start_approach(DockConfig(calibrated=True, run_limit=2.8,
                                     sensor_timeout=2.))
    t, base = approach_without_alignment(core)
    frozen = data(t + .04, base=base).observation
    for elapsed in (.04, .2, .4, .6, .8):
        out = core.update(data(t + elapsed, base=base, observation=frozen))
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
        assert out.velocity < 0.
    assert core.locked_dock_pose_local is None


def test_alignment_duration_alone_cannot_replace_minimum_distinct_samples():
    core = start_approach(DockConfig(calibrated=True, run_limit=2.8,
                                     lock_alignment_duration=.1,
                                     lock_min_observations=5))
    t, base = approach_without_alignment(core)
    for _ in range(4):
        t += .04
        out = core.update(data(t, base=base))
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    t += .04
    assert core.update(data(t, base=base)).state == 'LOCK_VERIFY'


def test_alignment_entry_observation_gap_restarts_continuous_window():
    core = start_approach()
    t, base = approach_without_alignment(core)
    for _ in range(8):
        t += .04
        assert core.update(data(t, base=base)).state != 'LOCK_VERIFY'
    t += core.config.sensor_timeout + .04
    assert core.update(data(t, base=base)).state != 'LOCK_VERIFY'
    for _ in range(8):
        t += .04
        out = core.update(data(t, base=base))
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    for _ in range(10):
        t += .04
        out = core.update(data(t, base=base))
        if out.state == 'LOCK_VERIFY':
            break
    assert out.state == 'LOCK_VERIFY'


def test_observation_hold_discards_alignment_window_before_resuming():
    core = start_approach()
    t, base = approach_without_alignment(core)
    for _ in range(8):
        t += .04
        assert core.update(data(t, base=base)).state != 'LOCK_VERIFY'
    assert core.lock_alignment_samples
    t += .04
    assert core.update(data(t, base=base, observation=False)).state == 'OBSERVATION_HOLD'
    assert not core.lock_alignment_samples
    for _ in range(20):
        t += .04
        out = core.update(data(t, base=base))
        if out.state != 'OBSERVATION_HOLD':
            break
    assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    resumed = t
    # Waiting motionless for observation reacquisition is not evidence of
    # alignment during a new live correction window.
    for _ in range(20):
        t += .04
        out = core.update(data(t, base=base))
        if out.state == 'LOCK_VERIFY':
            break
        assert out.velocity < 0.
    assert out.state == 'LOCK_VERIFY'
    assert t - resumed >= core.config.lock_alignment_duration


@pytest.mark.parametrize('missing_requirement', ['time', 'distance'])
def test_rejected_lock_requires_both_retry_time_and_actual_translation(missing_requirement):
    core = start_approach()
    t, base = reject_stopped_alignment(core)
    retry_started = t
    original_base = base
    if missing_requirement == 'time':
        # Enough measured translation, but less than the minimum response time.
        for _ in range(10):
            t += .04
            base = Pose2(base.x - .002)
            out = core.update(data(t, base=base))
            assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    else:
        # Enough elapsed time, but a stationary chassis has not corrected itself.
        for _ in range(40):
            t += .04
            out = core.update(data(t, base=base))
            assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
        assert base == original_base

    while t - retry_started < core.config.lock_retry_duration + .08:
        t += .04
        out = core.update(data(t, base=base))
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    if missing_requirement == 'distance':
        for _ in range(6):
            t += .04
            base = Pose2(base.x - .002)
            out = core.update(data(t, base=base))
            assert out.state in ('APPROACH', 'FINAL_APPROACH'), out

    for _ in range(24):
        t += .04
        out = core.update(data(t, base=base))
        if out.state == 'LOCK_VERIFY':
            break
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    assert out.state == 'LOCK_VERIFY'


def test_retry_oscillation_does_not_count_as_net_correction_distance():
    core = start_approach()
    t, base = reject_stopped_alignment(core)
    retry_base = base
    # Wheel/body oscillation accumulates ample path length while remaining
    # within 6mm of the rejected pose. It must not re-arm repeated braking.
    for n in range(60):
        t += .04
        base = Pose2(retry_base.x - (.006 if n % 2 else 0.))
        out = core.update(data(t, base=base))
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
        assert out.velocity < 0.
    base = Pose2(retry_base.x - .012)
    for _ in range(20):
        t += .04
        out = core.update(data(t, base=base))
        if out.state == 'LOCK_VERIFY':
            break
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    assert out.state == 'LOCK_VERIFY'


@pytest.mark.parametrize('change,reason', [
    ({'obstacle': True}, 'obstacle'),
    ({'autodrive': False}, 'manual_control'),
])
def test_retry_alignment_gate_does_not_delay_external_stop_faults(change, reason):
    core = start_approach()
    t, base = reject_stopped_alignment(core)
    out = core.update(data(t + .04, base=base, **change))
    assert out.state == 'STOPPING'
    assert out.reason == reason
    assert out.velocity == out.omega == 0.


def test_retry_alignment_gate_does_not_delay_required_lock_distance_failure():
    core = start_approach()
    t, base = reject_stopped_alignment(core)
    while -DOCK.relative_to(base).x > core.config.lock_required_by_distance:
        t += .04
        base = Pose2(base.x - .02)
        out = core.update(data(t, base=base, dock=Pose2(DOCK.x, .006)))
        if out.state == 'STOPPING':
            break
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
    assert out.state == 'STOPPING'
    assert out.reason == 'lock_alignment_not_achieved'
    assert out.velocity == out.omega == 0.


@pytest.mark.parametrize('field', [
    'lock_alignment_duration', 'lock_retry_duration', 'lock_retry_distance',
])
@pytest.mark.parametrize('value', [0., -.01, math.nan, math.inf, -math.inf])
def test_alignment_and_retry_require_positive_finite_configuration(field, value):
    with pytest.raises(ValueError):
        DockConfig(**{field: value})


@pytest.mark.parametrize('ratio', [0., -.01, 1.01, math.nan, math.inf, -math.inf])
def test_alignment_entry_ratio_rejects_invalid_range_or_nonfinite_value(ratio):
    with pytest.raises(ValueError):
        DockConfig(lock_alignment_entry_ratio=ratio)


def test_locked_final_rejects_measured_speed_above_its_own_limit():
    core = start_approach()
    t, base = approach_to_lock(core)
    t = verify_lock(core, t, base)

    # This is below the whole-run speed fault, but above the locked-final cap.
    out = core.update(data(t + .04, base=base, speed=-.05, observation=False))
    assert out.state == 'STOPPING'
    assert out.reason == 'locked_final_speed_limit'
    assert out.velocity == out.omega == 0.


@pytest.mark.parametrize('lateral,yaw', [(.04, 0.), (0., math.radians(4.))])
def test_unaligned_live_target_cannot_be_locked_or_blindly_followed(lateral, yaw):
    core = start_approach()
    # A complete face is deliberately kept visible while the vehicle advances
    # past the required lock distance. The core must fail closed, never switch
    # to odometry-only motion with an unaligned prior.
    base = Pose2(0., lateral, yaw)
    t = .6
    for _ in range(240):
        t += .04
        speed = 0. if core.state == 'LOCK_VERIFY' else -.025
        out = core.update(data(t, base=base, dock=DOCK, speed=speed))
        assert out.state != 'LOCKED_FINAL_APPROACH', out
        if out.state == 'STOPPING':
            break
        if out.state == 'LOCK_VERIFY':
            assert out.velocity == out.omega == 0.
            continue
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
        base = Pose2(base.x - .01, lateral, yaw)
    assert out.state == 'STOPPING'
    assert out.reason in ('lock_alignment_not_achieved', 'insufficient_alignment_space')
    assert core.locked_dock_pose_local is None
    assert out.velocity == out.omega == 0.


def test_lock_requires_unique_fresh_observations_and_stopped_vehicle():
    core = start_approach()
    t, base = approach_to_lock(core)

    # A live but invalid face cannot seed a fixed odometry target.
    t += .04
    out = core.update(data(t, base=base, observation=False))
    assert out.velocity == out.omega == 0.
    assert out.state != 'LOCKED_FINAL_APPROACH'

    # Resume fresh samples but keep measured motion nonzero: the lock cannot
    # complete while the base pose is still changing.
    out = core.update(data(t + .04, base=base, speed=-.02))
    assert out.velocity == out.omega == 0.
    assert out.state != 'LOCKED_FINAL_APPROACH'

    # Reacquisition first confirms a measured stop and stable observations.
    # Tracking must then independently restore its continuous alignment window
    # before another stop can verify the immutable target.
    t += .08
    for _ in range(40):
        t += .04
        out = core.update(data(t, base=base))
        if out.state == 'LOCK_VERIFY':
            break
        assert out.state in ('OBSERVATION_HOLD', 'APPROACH', 'FINAL_APPROACH'), out
        if out.state == 'OBSERVATION_HOLD':
            assert out.velocity == out.omega == 0.
    assert out.state == 'LOCK_VERIFY'
    t = verify_lock(core, t, base)
    assert core.state == 'LOCKED_FINAL_APPROACH'


def test_persistent_face_loss_before_lock_never_authorizes_blind_motion():
    core = start_approach()
    t, base = approach_to_lock(core)
    for _ in range(60):
        t += .04
        out = core.update(data(t, base=base, observation=False))
        assert out.state in ('OBSERVATION_HOLD', 'STOPPING', 'FAILED'), out
        assert out.velocity == out.omega == 0.
        if out.state == 'STOPPING':
            break
    assert out.state == 'STOPPING'
    assert core.final_state == 'FAILED'
    assert core.locked_dock_pose_local is None


@pytest.mark.parametrize('changes,reason', [
    ({'speed': -.12}, 'actual_motion_limit'),
    ({'local_base': Pose2(-1.7)}, 'odometry_jump'),
])
def test_motion_fault_on_first_lost_frame_during_lock_verification_is_not_hidden(changes, reason):
    core = start_approach()
    t, base = approach_to_lock(core)
    out = core.update(replace(data(t + .04, base=base, observation=False), **changes))
    assert out.state == 'STOPPING'
    assert out.reason == reason
    assert core.final_state == 'FAILED'
    assert out.velocity == out.omega == 0.


def test_repeated_observation_cannot_complete_stationary_lock():
    core = start_approach(DockConfig(calibrated=True, run_limit=2.8,
                                     sensor_timeout=2.))
    t, base = approach_to_lock(core)
    frozen = data(t + .04, base=base).observation
    for elapsed in (.04, .20, .40, .60, .80):
        out = core.update(data(t + elapsed, base=base, observation=frozen))
        assert out.state == 'LOCK_VERIFY'
        assert out.velocity == out.omega == 0.

    # Control timer updates do not count as independent lidar confirmations.
    # Once new scans arrive, a bounded interval of stable measured geometry
    # can complete verification.
    verify_lock(core, t + .8, base)
    assert core.state == 'LOCKED_FINAL_APPROACH'


def test_lock_samples_before_measured_motion_do_not_contaminate_stopped_pose():
    core = start_approach()
    t, base = approach_to_lock(core)
    for _ in range(4):
        t += .04
        out = core.update(data(t, base=base, dock=Pose2(DOCK.x + .025)))
        assert out.state == 'LOCK_VERIFY'
    t += .04
    base = Pose2(base.x - .005)
    out = core.update(data(t, base=base, speed=-.02))
    assert out.state == 'LOCK_VERIFY'
    assert core.locked_dock_pose_local is None

    t = verify_lock(core, t, base)
    assert core.locked_dock_pose_local.x == pytest.approx(DOCK.x)
    assert core.locked_distance == pytest.approx(-DOCK.relative_to(base).x)


def test_lock_observation_gap_requires_a_new_continuous_stationary_sample_window():
    core = start_approach()
    t, base = approach_to_lock(core)
    for _ in range(3):
        t += .04
        assert core.update(data(t, base=base)).state == 'LOCK_VERIFY'

    # Every supplied scan is fresh, but the missing interval breaks continuous
    # confirmation. Elapsed control time alone must not complete the lock.
    t += core.config.sensor_timeout + .04
    assert core.update(data(t, base=base)).state == 'LOCK_VERIFY'
    for _ in range(10):
        t += .04
        out = core.update(data(t, base=base))
        assert out.state == 'LOCK_VERIFY'
        assert out.velocity == out.omega == 0.
        assert core.locked_dock_pose_local is None
    verify_lock(core, t, base)
    assert core.state == 'LOCKED_FINAL_APPROACH'


@pytest.mark.parametrize('observation', [True, False])
def test_braking_travel_limit_during_lock_verification_is_never_hidden(observation):
    core = start_approach(DockConfig(calibrated=True, run_limit=1.49))
    t, base = approach_to_lock(core)
    out = core.update(data(t + .04, base=Pose2(base.x - .04), speed=-.025,
                           observation=observation))
    assert out.state == 'STOPPING'
    assert out.reason == 'travel_limit'
    assert core.final_state == 'LIMITED_RUN_COMPLETE'
    assert core.locked_dock_pose_local is None
    assert out.velocity == out.omega == 0.


def test_target_pose_must_stabilize_before_odometry_only_motion_is_authorized():
    core = start_approach()
    t, base = approach_to_lock(core)
    for n in range(16):
        t += .04
        moving_estimate = Pose2(DOCK.x + (.025 if n % 2 else 0.), DOCK.y)
        out = core.update(data(t, base=base, dock=moving_estimate))
        assert out.state == 'LOCK_VERIFY'
        assert out.velocity == out.omega == 0.
    verify_lock(core, t, base)
    assert core.state == 'LOCKED_FINAL_APPROACH'


def test_locked_goal_does_not_jump_when_lidar_returns_a_different_face_pose():
    first = start_approach()
    t1, base1 = approach_to_lock(first)
    t1 = verify_lock(first, t1, base1)

    second = start_approach()
    t2, base2 = approach_to_lock(second)
    t2 = verify_lock(second, t2, base2)
    assert t1 == pytest.approx(t2)
    assert base1 == base2

    base = Pose2(base1.x - .03, base1.y, base1.yaw)
    lost = first.update(data(t1 + .04, base=base, observation=False, speed=-.025))
    misleading_dock = Pose2(DOCK.x + .12, DOCK.y + .06, DOCK.yaw + .08)
    shifted = second.update(data(t2 + .04, base=base, dock=misleading_dock,
                                 speed=-.025))
    assert lost.state == shifted.state == 'LOCKED_FINAL_APPROACH'
    assert lost.velocity == pytest.approx(shifted.velocity)
    assert lost.omega == pytest.approx(shifted.omega)


@pytest.mark.parametrize('change,reason', [
    ({'pose_stamp': -10.}, 'vehicle_pose_stale'),
    ({'vehicle_stamp': -10.}, 'vehicle_feedback_stale'),
    ({'obstacle': True}, 'obstacle'),
    ({'autodrive': False}, 'manual_control'),
    ({'control_owned': False}, 'control_not_owned'),
])
def test_locked_final_still_fails_safe_on_feedback_and_obstacle_faults(change, reason):
    core = start_approach()
    t, base = approach_to_lock(core)
    t = verify_lock(core, t, base)
    # Removing the old 35s deadline must not weaken fault handling, including
    # after that deadline would previously have stopped a slow approach.
    for _ in range(400):
        t += .1
        assert core.update(data(t, base=base,
                                observation=False)).state == 'LOCKED_FINAL_APPROACH'
    out = core.update(data(t + .04, base=Pose2(base.x - .02), speed=-.025,
                           observation=False, **change))
    assert out.state == 'STOPPING'
    assert out.reason == reason
    assert out.velocity == out.omega == 0.


def test_locked_final_rejects_stale_scan_even_though_complete_face_is_optional():
    core = start_approach()
    t, base = approach_to_lock(core)
    t = verify_lock(core, t, base)
    stale = FaceObservation(t - 1., False, None, 0., 0., 0., 'stale_scan')
    out = core.update(data(t + .04, base=Pose2(base.x - .02), speed=-.025,
                           observation=stale))
    assert out.state == 'STOPPING'
    assert out.velocity == out.omega == 0.


@pytest.mark.parametrize('base_delta,reason', [
    (Pose2(-.15), 'odometry_jump'),
    (Pose2(-.01, .04), 'locked_alignment_lost'),
    (Pose2(-.01, 0., math.radians(4.)), 'locked_alignment_lost'),
])
def test_locked_final_cannot_continue_when_odometry_or_alignment_is_unreliable(base_delta, reason):
    core = start_approach()
    t, base = approach_to_lock(core)
    t = verify_lock(core, t, base)
    out = core.update(data(t + .04, base=base.compose(base_delta), speed=-.025,
                           observation=False))
    assert out.state == 'STOPPING'
    assert out.reason == reason
    assert out.velocity == out.omega == 0.


def test_locked_final_has_independent_travel_and_time_bounds():
    core = start_approach(DockConfig(calibrated=True, run_limit=2.8,
                                     locked_final_travel_limit=.04))
    t, base = approach_to_lock(core)
    t = verify_lock(core, t, base)
    assert core.update(data(t + .04, base=Pose2(base.x - .03), speed=-.025,
                            observation=False)).state == 'LOCKED_FINAL_APPROACH'
    out = core.update(data(t + .08, base=Pose2(base.x - .05), speed=-.025,
                           observation=False))
    assert out.state == 'STOPPING'
    assert out.reason in ('locked_final_travel_limit', 'travel_limit')
    assert out.velocity == out.omega == 0.

    core = start_approach(DockConfig(calibrated=True, run_limit=2.8,
                                     locked_final_timeout=.08))
    t, base = approach_to_lock(core)
    t = verify_lock(core, t, base)
    out = core.update(data(t + .04, base=base, observation=False))
    assert out.state == 'LOCKED_FINAL_APPROACH'
    out = core.update(data(t + .12, base=base, observation=False))
    assert out.state == 'STOPPING'
    assert out.reason == 'locked_final_timeout'
    assert core.final_state == 'FAILED'
    assert out.velocity == out.omega == 0.


@pytest.mark.parametrize('final_timeout', [None, 0.])
def test_disabled_deadlines_allow_late_lock_and_slow_final_approach(final_timeout):
    config = DockConfig(calibrated=True, run_limit=2.8, timeout=0.)
    if final_timeout is not None:
        config = replace(config, locked_final_timeout=final_timeout)
    core = start_approach(config)
    for n in range(1, 920):
        t = .6 + n * .1
        out = core.update(data(t))
        assert out.state == 'APPROACH', out
    assert t - core.started > 90.

    t, base = approach_to_lock(core, t=t + .04)
    t = verify_lock(core, t, base)
    assert core.state == 'LOCKED_FINAL_APPROACH'
    assert core.locked_final_started > 90.
    assert core.config.locked_final_timeout == 0.

    # Fresh feedback, scans, and small measured odometry increments permit a
    # slow approach to continue beyond both former 35s and 90s deadlines.
    for _ in range(1210):
        t += .1
        base = Pose2(base.x - .00015)
        out = core.update(data(t, base=base, speed=-.0015, observation=False))
        assert out.state == 'LOCKED_FINAL_APPROACH', out
        assert out.velocity < 0.
    assert t - core.locked_final_started > 120.

    # The geometric target still ends the approach after a long duration.
    while base.x > FINAL_BASE_X + .025:
        t += .04
        base = Pose2(base.x - .01)
        out = core.update(data(t, base=base, speed=-.025, observation=False))
        assert out.state == 'LOCKED_FINAL_APPROACH', out
    t += .04
    base = Pose2(FINAL_BASE_X)
    out = core.update(data(t, base=base, observation=False))
    assert out.state == 'STOPPING'
    assert out.reason == 'geometric_alignment'
    assert core.final_state == 'SUCCEEDED'
    assert out.velocity == out.omega == 0.
    core.update(data(t + .04, base=base, observation=False))
    assert core.update(data(t + .60, base=base, observation=False)).state == 'SUCCEEDED'


@pytest.mark.parametrize('final_timeout', [-.01, -35., math.nan, math.inf, -math.inf])
def test_locked_final_deadline_rejects_negative_and_nonfinite_configuration(final_timeout):
    with pytest.raises(ValueError):
        DockConfig(locked_final_timeout=final_timeout)


def test_locked_final_overshoot_and_cancel_both_command_measured_stop():
    core = start_approach()
    t, base = approach_to_lock(core)
    t = verify_lock(core, t, base)
    while base.x > FINAL_BASE_X + .035:
        t += .04
        base = Pose2(base.x - .01)
        assert core.update(data(t, base=base, speed=-.025,
                                observation=False)).state == 'LOCKED_FINAL_APPROACH'
    out = core.update(data(t + .04, base=Pose2(FINAL_BASE_X - .015), speed=-.025,
                           observation=False))
    assert out.state == 'STOPPING'
    assert out.reason == 'target_overshoot'
    assert core.final_state == 'FAILED'
    assert out.velocity == out.omega == 0.

    core = start_approach()
    t, base = approach_to_lock(core)
    t = verify_lock(core, t, base)
    core.cancel(t + .01)
    out = core.update(data(t + .04, base=base, speed=-.02, observation=False))
    assert out.state == 'STOPPING'
    assert out.velocity == out.omega == 0.
    core.update(data(t + .08, base=base, observation=False))
    assert core.update(data(t + .65, base=base, observation=False)).state == 'CANCELED'


def test_starting_another_attempt_clears_the_previous_odometry_target():
    core = start_approach()
    t, base = approach_to_lock(core)
    t = verify_lock(core, t, base)
    assert core.locked_dock_pose_local is not None
    core.start(t + .1, dry_run=False)
    assert core.locked_dock_pose_local is None
    assert core.locked_distance is None
    assert core.locked_remaining_distance is None
    out = core.update(data(t + .1, base=base, observation=False))
    assert out.state == 'PRECHECK'
    assert out.velocity == out.omega == 0.


def test_locked_pose_remains_in_odometry_frame_when_local_frame_has_yaw_offset():
    frame = Pose2(3., -2., math.radians(65.))
    dock = frame.compose(DOCK)
    core = RelativeDockingCore(DockConfig(calibrated=True, run_limit=2.8))
    core.start(0., dry_run=False)
    core.update(data(0., base=frame, dock=dock))
    core.update(data(.6, base=frame, dock=dock))
    t, x = .6, 0.
    for _ in range(170):
        t += .04
        out = core.update(data(t, base=frame.compose(Pose2(x)), dock=dock,
                               speed=-.025))
        if out.state == 'LOCK_VERIFY':
            break
        assert out.state in ('APPROACH', 'FINAL_APPROACH'), out
        x -= .01
    assert out.state == 'LOCK_VERIFY'
    base = frame.compose(Pose2(x))
    t = verify_lock(core, t, base, dock)
    locked = core.locked_dock_pose_local
    assert locked.x == pytest.approx(dock.x)
    assert locked.y == pytest.approx(dock.y)
    assert locked.yaw == pytest.approx(dock.yaw)

    for n in range(1, 56):
        t += .04
        current_x = x + (FINAL_BASE_X - x) * n / 55
        base = frame.compose(Pose2(current_x))
        out = core.update(data(t, base=base, dock=dock, speed=-.025,
                               observation=False))
        assert out.state in ('LOCKED_FINAL_APPROACH', 'STOPPING'), out
    assert out.state == 'STOPPING'
    assert core.final_state == 'SUCCEEDED'
    assert abs(out.error.x) <= core.config.longitudinal_tolerance
    assert abs(out.error.y) < 1e-8
    assert abs(out.error.yaw) < 1e-8
