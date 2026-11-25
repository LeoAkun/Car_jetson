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

"""Ray-cast perception, independent bicycle simulation and fault regression."""

from dataclasses import fields, replace
import math

import numpy as np
import pytest

from charging_dock.dock_collision import collision, dock_plane_collision
from charging_dock.relative_control import ControlInput, DockConfig, RelativeDockingCore
from charging_dock.relative_geometry import Pose2, integrate, longitudinal_speed
from charging_dock.relative_lidar import FaceConfig, FaceObservation, RelativeFaceTracker
from relative_scan_fixture import docking_parameters, scan_faces


SENSOR = Pose2(-0.43, -0.28, math.radians(225))


@pytest.mark.parametrize('previous,current,expected', [
    (Pose2(1., 2., math.pi / 2), Pose2(1., 2.004, math.pi / 2), .04),
    (Pose2(1., 2., math.pi / 2), Pose2(1., 1.996, math.pi / 2), -.04),
    (Pose2(yaw=math.radians(179)), Pose2(-.004, 0., math.radians(-179)), .04),
    (Pose2(yaw=math.radians(179)), Pose2(.004, 0., math.radians(-179)), -.04),
])
def test_measured_direction_uses_vehicle_heading_across_yaw_wrap(previous, current, expected):
    assert longitudinal_speed(previous, current, .1) == pytest.approx(expected)


@pytest.mark.parametrize('distance,sensor', [
    (2.5, Pose2()), (2.5, SENSOR), (0.47, SENSOR), (1.0, Pose2(0, 0, math.pi)),
])
def test_finite_face_center_and_normal_across_scan_seam(distance, sensor):
    dock = Pose2(-distance)
    tracker = RelativeFaceTracker(FaceConfig(width=.185, stable_frames=3))
    tracker.reset(dock)
    for n in range(3):
        t = 1 + n * .04
        obs = tracker.process(scan_faces(t, sensor=sensor, faces=((dock, .185),)),
                              t, Pose2(), sensor)
        assert obs.valid == (n == 2)
    assert obs.pose.x == pytest.approx(-distance, abs=.002)
    assert abs(obs.pose.y) < .006
    assert abs(obs.pose.yaw) < .001


@pytest.mark.parametrize('depth', [.01, .03, .10])
def test_charging_face_is_distinct_from_25cm_back_plate(depth):
    dock = Pose2(-2.5)
    tracker = RelativeFaceTracker(FaceConfig(width=.185, stable_frames=1))
    tracker.reset(dock)
    scan = scan_faces(1, faces=((dock, .185), (Pose2(-2.5-depth), .25)))
    obs = tracker.process(scan, 1, Pose2(), Pose2())
    assert obs.valid
    assert obs.pose.x == pytest.approx(-2.5, abs=.001)
    assert .17 < obs.width <= .185


@pytest.mark.parametrize('width', [.10, .25, 4.0])
def test_partial_face_back_plate_and_wall_cannot_supply_dock_center(width):
    tracker = RelativeFaceTracker(FaceConfig(width=.185, stable_frames=1))
    tracker.reset(Pose2(-2.5))
    obs = tracker.process(scan_faces(1, faces=((Pose2(-2.5), width),)),
                          1, Pose2(), Pose2())
    assert not obs.valid
    assert obs.pose is None


def test_corrupt_beams_are_not_physical_edges():
    tracker = RelativeFaceTracker(FaceConfig(width=.185, stable_frames=1))
    tracker.reset(Pose2(-2.5))
    scan = scan_faces(1)
    finite = np.flatnonzero(np.isfinite(scan.ranges))
    for i in finite:
        for j in ((i-1) % len(scan.ranges), (i+1) % len(scan.ranges)):
            if not math.isfinite(scan.ranges[j]):
                scan.ranges[j] = math.nan
    assert not tracker.process(scan, 1, Pose2(), Pose2()).valid


def test_local_target_tracking_survives_vehicle_rotation_and_sensor_offset():
    dock = Pose2(-2.5, .1, .06)
    tracker = RelativeFaceTracker(FaceConfig(width=.185, stable_frames=1))
    tracker.reset(dock)
    for n, base in enumerate([Pose2(), Pose2(-.6, .06, .12), Pose2(-1.4, -.02, -.05)]):
        t = 1 + .1*n
        obs = tracker.process(scan_faces(t, base, SENSOR, ((dock, .185),)),
                              t, base, SENSOR)
        assert obs.valid
        measured = base.compose(obs.pose)
        assert measured.x == pytest.approx(dock.x, abs=.005)
        assert measured.y == pytest.approx(dock.y, abs=.007)
        assert measured.yaw == pytest.approx(dock.yaw, abs=.002)


def test_fresh_ordered_consecutive_observations_required_after_loss():
    tracker = RelativeFaceTracker(FaceConfig(width=.185, stable_frames=2))
    tracker.reset(Pose2(-2.5))
    tracker.process(scan_faces(1), 1, Pose2(), Pose2())
    assert tracker.process(scan_faces(1.04), 1.04, Pose2(), Pose2()).valid
    assert not tracker.process(scan_faces(1.04), 1.05, Pose2(), Pose2()).valid
    assert not tracker.process(scan_faces(1.05), 1.4, Pose2(), Pose2()).valid
    assert not tracker.process(scan_faces(1.44), 1.44, Pose2(), Pose2()).valid
    assert tracker.process(scan_faces(1.48), 1.48, Pose2(), Pose2()).valid
    assert not tracker.process(scan_faces(2.0), 1.5, Pose2(), Pose2()).valid


def state_input(t, base=Pose2(), dock=Pose2(-2.5), speed=0., omega=0.):
    return ControlInput(t, base, t, FaceObservation(t, True, dock.relative_to(base), 1.),
                        speed, omega, t, True, False, True)


def simulate(y, yaw, with_lidar=False, receiver=-.45, config=None, face_config=None,
             scan_geometry=None, expected_failure_reasons=()):
    cfg = config or DockConfig(calibrated=True, run_limit=2.8,
                               base_receiver=Pose2(receiver))
    core = RelativeDockingCore(cfg)
    core.start(0., dry_run=False)
    dock, base = Pose2(-2.5), Pose2(0., y, math.radians(yaw))
    tracker = RelativeFaceTracker(face_config or FaceConfig(width=.185, stable_frames=3))
    tracker.reset(cfg.face_pose(dock))
    scan_geometry = scan_geometry or ((dock, .185), (Pose2(-2.53), .25))
    v = omega = 0.
    for n in range(2400):
        t = n * .04
        data = state_input(t, base, dock, v, omega)
        if with_lidar:
            raw = tracker.process(scan_faces(t, base, SENSOR, scan_geometry),
                                  t, base, SENSOR)
            data.observation = replace(raw, pose=cfg.dock_pose(raw.pose) if raw.pose else None)
        out = core.update(data)
        assert out.velocity <= 0.
        assert abs(out.velocity) <= cfg.speed
        sweep_v = -max(abs(out.velocity), abs(v))
        horizon = max(.05, cfg.latency + abs(sweep_v) / (2 * cfg.braking))
        sweep_w = (sweep_v * out.omega / out.velocity if out.velocity
                   else omega)
        if with_lidar and data.observation.valid:
            assert not dock_plane_collision(data.observation.pose, sweep_v, sweep_w,
                                            (.43, .43, .43), .02, horizon)
        # Independent bicycle plant with 120ms velocity response. Steering is
        # derived from output curvature, then applied to actual vehicle speed.
        steer = math.atan(cfg.wheelbase * out.omega / out.velocity) if out.velocity else 0.
        assert abs(steer) <= cfg.max_steer + 1e-9
        # Match actual CAN quantization: docking converts m/s to km/h and
        # the chassis encoder rounds to 0.1 km/h (not arbitrary tiny speeds).
        requested = -math.floor(abs(out.velocity) * 3.6 * 10 + .5) / 36
        v += (.04 / .12) * (requested - v)
        omega = v * math.tan(steer) / cfg.wheelbase
        base = integrate(base, v, omega, .04)
        if out.state in core.TERMINAL:
            break
    actual = base.relative_to(cfg.base_goal(dock))
    if expected_failure_reasons:
        assert out.state == 'FAILED', (out, actual)
        assert out.reason in expected_failure_reasons
        assert out.velocity == out.omega == 0.
        assert core.locked_dock_pose_local is None
        assert (abs(out.error.y) > cfg.lock_alignment_lateral
                or abs(out.error.yaw) > cfg.lock_alignment_yaw)
        return n * .04, actual
    assert out.state == 'SUCCEEDED', (out, actual)
    assert abs(actual.x) <= cfg.longitudinal_tolerance
    assert abs(actual.y) <= cfg.lateral_tolerance
    assert abs(actual.yaw) <= cfg.yaw_tolerance
    assert core.traveled < cfg.motion_limit
    return n * .04, actual


@pytest.mark.parametrize('y,yaw', [(0, 0), (.05, 3), (-.05, -3),
                                   (.1, -3), (-.1, 3), (.1, 5)])
def test_reverse_bicycle_converges_with_lateral_and_heading_errors(y, yaw):
    simulate(y, yaw)


def test_full_scan_detection_to_bicycle_control_converges():
    simulate(.05, 3, with_lidar=True)


def configured_geometry():
    p = docking_parameters()['relative_docking_controller']['ros__parameters']
    values = {f.name: p[f.name] for f in fields(DockConfig) if f.name in p}
    for field_name, prefix in (('base_receiver', 'receiver'),
                               ('dock_receiver', 'target_receiver'),
                               ('face_to_dock', 'face_to_dock')):
        values[field_name] = Pose2(*(p[prefix + '_' + axis] for axis in ('x', 'y', 'yaw')))
    face = FaceConfig(**{f.name: p['face_' + f.name] for f in fields(FaceConfig)
                         if 'face_' + f.name in p})
    return DockConfig(**values), face


@pytest.mark.parametrize('y,yaw', [(0, 0), (.05, 3), (-.05, -3), (.1, -3)])
def test_photo_back_plate_geometry_converges_using_shipped_configuration(y, yaw):
    cfg, face = configured_geometry()
    assert not cfg.calibrated
    # Scan height clears the protruding module: ONLY the 25cm backing plane
    # can return rays. Its physical location is independent of model transforms.
    simulate(y, yaw, with_lidar=True, config=replace(cfg, calibrated=True),
             face_config=face, scan_geometry=((Pose2(-2.557), .25),))


def test_photo_geometry_stops_without_lock_when_alignment_window_is_missed():
    cfg, face = configured_geometry()
    simulate(-.1, 5, with_lidar=True, config=replace(cfg, calibrated=True),
             face_config=face, scan_geometry=((Pose2(-2.557), .25),),
             expected_failure_reasons=('insufficient_alignment_space',
                                       'lock_alignment_not_achieved'))


@pytest.mark.parametrize('distance', [.45, .46, .47])
def test_back_plate_and_physical_charging_windows_are_not_interchangeable(distance):
    cfg, _ = configured_geometry()
    observed = Pose2(-distance - .057)
    dock = cfg.dock_pose(observed)
    assert dock.x == pytest.approx(-distance)
    assert cfg.face_pose(dock).x == pytest.approx(observed.x)
    assert cfg.base_goal(dock).x == pytest.approx(.46 - distance)
    assert cfg.base_goal(observed).x == pytest.approx(.46 - distance - .057)


def test_back_plate_offset_follows_observed_normal_and_rejects_invalid_transform():
    cfg, _ = configured_geometry()
    observed = Pose2(-.7, .08, math.radians(20))
    dock = cfg.dock_pose(observed)
    assert dock.x == pytest.approx(observed.x + .057 * math.cos(observed.yaw))
    assert dock.y == pytest.approx(observed.y + .057 * math.sin(observed.yaw))
    assert dock.yaw == observed.yaw
    recovered = cfg.face_pose(dock)
    assert recovered.x == pytest.approx(observed.x)
    assert recovered.y == pytest.approx(observed.y)
    with pytest.raises(ValueError, match='mechanical transforms'):
        replace(cfg, face_to_dock=Pose2(math.nan))


@pytest.mark.parametrize('distance', [2.557, .527, .517, .507])
def test_scan_height_above_module_identifies_back_plate_with_side_mounted_sensor(distance):
    cfg, face = configured_geometry()
    tracker = RelativeFaceTracker(face)
    tracker.reset(Pose2(-distance))
    for n in range(face.stable_frames):
        t = 1 + n * .04
        raw = tracker.process(scan_faces(t, sensor=SENSOR, faces=((Pose2(-distance), .25),)),
                              t, Pose2(), SENSOR)
    assert raw.valid, raw
    assert cfg.dock_pose(raw.pose).x == pytest.approx(-distance + .057, abs=.002)
    assert abs(raw.pose.y) < .01
    assert raw.width == pytest.approx(.25, abs=face.width_tolerance)


def test_oblique_back_plate_center_is_stable_across_sub_beam_motion():
    _, face = configured_geometry()
    sensor = SENSOR
    back_plate = Pose2(-.517)
    errors = []
    # These poses move the physical edges through different angular samples.
    for y in np.linspace(-.02, .02, 101):
        tracker = RelativeFaceTracker(replace(face, stable_frames=1))
        tracker.reset(back_plate)
        base = Pose2(y=float(y))
        raw = tracker.process(scan_faces(1, base, sensor, ((back_plate, .25),)),
                              1, base, sensor)
        assert raw.valid
        errors.append(base.compose(raw.pose).y)
    # Finite angular sampling leaves millimetre uncertainty at this incidence,
    # but should not bias the centre towards the more densely sampled edge.
    assert abs(np.mean(errors)) < .0003
    assert max(abs(e) for e in errors) < .003


def test_shell_clearance_uses_projected_charging_face_below_scan_height():
    cfg, _ = configured_geometry()
    back_plate, bounds = Pose2(-.49), (.43, .43, .43)
    assert not dock_plane_collision(back_plate, 0., 0., bounds, .02)
    assert dock_plane_collision(cfg.dock_pose(back_plate), 0., 0., bounds, .02)


def test_receiver_change_needs_only_mechanical_transform():
    simulate(.05, -2, receiver=-.52)


def test_tracker_cannot_walk_away_from_the_original_dock_prior():
    tracker = RelativeFaceTracker(FaceConfig(width=.185, stable_frames=1))
    tracker.reset(Pose2(-2.5))
    for n, x in enumerate((-2.4, -2.3, -2.2, -2.1)):
        t = 1 + n*.04
        obs = tracker.process(scan_faces(t, faces=((Pose2(x), .185),)),
                              t, Pose2(), Pose2())
    assert not obs.valid


def test_face_candidate_outside_p0_depth_band_is_rejected():
    tracker = RelativeFaceTracker(FaceConfig(
        width=.25, stable_frames=1, position_gate=.35, depth_gate=.15))
    tracker.reset(Pose2(-2.557))
    closer_same_width_return = Pose2(-2.30)
    obs = tracker.process(scan_faces(1, faces=((closer_same_width_return, .25),)),
                          1, Pose2(), Pose2())
    assert not obs.valid
    assert obs.reason == 'no_complete_face_in_gate'


def test_depth_gate_preserves_lateral_correction_at_p0():
    tracker = RelativeFaceTracker(FaceConfig(
        width=.25, stable_frames=1, position_gate=.35, depth_gate=.15))
    target = Pose2(-2.557, .25)
    tracker.reset(Pose2(-2.557))
    obs = tracker.process(scan_faces(1, faces=((target, .25),)), 1, Pose2(), Pose2())
    assert obs.valid
    assert obs.pose.x == pytest.approx(target.x, abs=.003)
    assert obs.pose.y == pytest.approx(target.y, abs=.01)


@pytest.mark.parametrize('lateral,visible', [(.15, False), (.22, True)])
def test_closer_same_width_distractor_does_not_replace_dock(lateral, visible):
    prior = Pose2(-2.557)
    tracker = RelativeFaceTracker(FaceConfig(width=.25, stable_frames=1))
    tracker.reset(prior)
    # Both complete faces would pass the circular 0.35m position gate. At
    # .15m lateral offset the closer object occludes part of the real dock.
    distractor = Pose2(-2.30, lateral)
    scan = scan_faces(1, sensor=SENSOR, faces=((distractor, .25), (prior, .25)))
    obs = tracker.process(scan, 1, Pose2(), SENSOR)
    assert obs.valid == visible
    if visible:
        assert obs.pose.x == pytest.approx(prior.x, abs=.003)
        assert abs(obs.pose.y) < .01
    else:
        assert obs.pose is None
        assert obs.reason == 'no_complete_face_in_gate'


def test_scan_fov_boundary_cannot_be_accepted_as_a_face_edge():
    tracker = RelativeFaceTracker(FaceConfig(width=.185, stable_frames=1))
    tracker.reset(Pose2(-2.5))
    scan = scan_faces(1, n=100, angle_min=math.pi-.037, angle_span=.074)
    assert not tracker.process(scan, 1, Pose2(), Pose2()).valid


def test_two_equally_plausible_faces_are_rejected():
    tracker = RelativeFaceTracker(FaceConfig(width=.185, stable_frames=1))
    tracker.reset(Pose2(-2.5))
    scan = scan_faces(1, faces=((Pose2(-2.5, -.15), .185),
                                (Pose2(-2.5, .15), .185)))
    obs = tracker.process(scan, 1, Pose2(), Pose2())
    assert not obs.valid
    assert obs.reason == 'ambiguous_faces'


def test_dry_run_rejects_unreachable_lateral_offset():
    core = RelativeDockingCore()
    core.start(1., dry_run=True)
    out = core.update(state_input(1., dock=Pose2(-2.5, .4)))
    assert out.velocity == 0.
    assert out.state == 'STOPPING'
    assert out.reason == 'outside_capture_region'


def running_core(config=None):
    core = RelativeDockingCore(config or DockConfig(calibrated=True, run_limit=2.8))
    core.start(1., dry_run=False)
    core.update(state_input(1.))
    assert core.update(state_input(1.6)).velocity < 0
    return core


@pytest.mark.parametrize('timeout', [None, 0.])
def test_total_deadline_is_disabled_by_default_and_by_explicit_zero(timeout):
    config = DockConfig(calibrated=True, run_limit=2.8)
    if timeout is not None:
        config = replace(config, timeout=timeout)
    assert config.timeout == 0.
    core = running_core(config)
    # Continue supplying fresh, valid observations and feedback beyond the
    # former 90s deadline. Slow progress must not become a total-time failure.
    for n in range(1, 920):
        out = core.update(state_input(1.6 + n * .1))
        assert out.state == 'APPROACH', out
        assert out.velocity < 0.
    assert core.last_time - core.started > 90.


def test_explicit_positive_total_deadline_still_stops_an_active_attempt():
    core = running_core(DockConfig(calibrated=True, run_limit=2.8, timeout=1.))
    assert core.update(state_input(1.96)).state == 'APPROACH'
    out = core.update(state_input(2.04))
    assert out.state == 'STOPPING'
    assert out.reason == 'timeout'
    assert core.final_state == 'FAILED'
    assert out.velocity == out.omega == 0.


@pytest.mark.parametrize('timeout', [-.01, -90., math.nan, math.inf, -math.inf])
def test_total_deadline_rejects_negative_and_nonfinite_configuration(timeout):
    with pytest.raises(ValueError):
        DockConfig(timeout=timeout)


@pytest.mark.parametrize('change,reason', [
    ({'vehicle_stamp': 1.}, 'vehicle_feedback_stale'),
    ({'pose_stamp': 1.}, 'vehicle_pose_stale'),
    ({'autodrive': False}, 'manual_control'),
    ({'control_owned': False}, 'control_not_owned'),
    ({'obstacle': True}, 'obstacle'),
    ({'speed': math.nan}, 'vehicle_feedback_stale'),
    ({'local_base': Pose2(.4)}, 'odometry_jump'),
])
def test_active_fault_immediately_commands_zero_then_confirms_stop(change, reason):
    core = running_core()
    out = core.update(replace(state_input(1.64), **change))
    assert out.state == 'STOPPING'
    assert out.velocity == out.omega == 0.
    assert out.reason == reason
    core.update(state_input(1.68))
    assert core.update(state_input(2.2)).state == 'FAILED'


def test_short_observation_loss_holds_zero_until_stable_target_reacquired():
    core = running_core()
    held = core.update(replace(state_input(1.64), observation=None))
    assert held.state == 'OBSERVATION_HOLD'
    assert held.velocity == held.omega == 0.
    assert held.reason == 'dock_observation_invalid'

    still_held = core.update(replace(state_input(1.8), observation=None))
    assert still_held.state == 'OBSERVATION_HOLD'
    assert still_held.velocity == 0.

    reacquired = core.update(state_input(1.84))
    assert reacquired.state == 'OBSERVATION_HOLD'
    assert reacquired.velocity == 0.
    still_stabilizing = core.update(state_input(2.2))
    assert still_stabilizing.state == 'OBSERVATION_HOLD'
    resumed = core.update(state_input(2.4))
    assert resumed.state == 'APPROACH'
    assert resumed.velocity < 0.


def test_reacquisition_stability_restarts_if_vehicle_is_still_moving():
    core = running_core()
    core.update(replace(state_input(1.64), observation=None))
    core.update(state_input(1.68))
    moving = core.update(state_input(2.0, speed=-.02))
    assert moving.state == 'OBSERVATION_HOLD'
    assert moving.reason == 'waiting_for_measured_stop'
    assert moving.velocity == 0.
    assert core.update(state_input(2.2)).state == 'OBSERVATION_HOLD'
    resumed = core.update(state_input(2.71))
    assert resumed.state == 'APPROACH'
    assert resumed.velocity < 0.


def test_observation_hold_accounts_for_odom_and_enforces_run_limit():
    core = running_core(DockConfig(calibrated=True, run_limit=.06))
    first = core.update(replace(state_input(1.64, base=Pose2(-.02), speed=-.06),
                                observation=None))
    assert first.state == 'OBSERVATION_HOLD'
    assert core.traveled == pytest.approx(.02)

    second = core.update(replace(state_input(1.8, base=Pose2(-.05), speed=-.03),
                                 observation=None))
    assert second.state == 'OBSERVATION_HOLD'
    assert core.traveled == pytest.approx(.05)

    limited = core.update(state_input(1.84, base=Pose2(-.06)))
    assert limited.state == 'STOPPING'
    assert limited.reason == 'travel_limit'
    assert limited.velocity == limited.omega == 0.


def test_motion_limit_fault_is_terminal_even_while_observation_is_lost():
    core = running_core()
    faulted = core.update(replace(state_input(1.64, speed=-.12), observation=None))
    assert faulted.state == 'STOPPING'
    assert faulted.reason == 'actual_motion_limit'
    assert faulted.velocity == faulted.omega == 0.


def test_removing_wheel_difference_check_preserves_actual_speed_limit():
    core = running_core()
    # run11's wheel mean is valid for odometry after removing its difference
    # check; an actual vehicle speed this large still exceeds the dock limit.
    speed = (.01270250603556633 + .31756266951560974) / 2
    faulted = core.update(state_input(1.64, speed=speed))
    assert faulted.state == 'STOPPING'
    assert faulted.reason == 'actual_motion_limit'
    assert faulted.velocity == faulted.omega == 0.


def test_unrecovered_observation_loss_fails_after_bounded_zero_speed_hold():
    core = running_core(DockConfig(calibrated=True, run_limit=2.8,
                                   observation_reacquire_timeout=.30))
    assert core.update(replace(state_input(1.64), observation=None)).state == 'OBSERVATION_HOLD'
    held = core.update(replace(state_input(1.90), observation=None))
    assert held.state == 'OBSERVATION_HOLD'
    failed = core.update(replace(state_input(1.96), observation=None))
    assert failed.state == 'STOPPING'
    assert failed.velocity == failed.omega == 0.
    assert failed.reason == 'dock_observation_reacquire_timeout'
    core.update(state_input(2.0))
    assert core.update(state_input(2.52)).state == 'FAILED'


def test_dry_run_and_uncalibrated_live_attempt_never_command_motion():
    core = RelativeDockingCore()
    core.start(0.)
    core.update(state_input(0.))
    assert core.update(state_input(.6)).state == 'DRY_RUN_READY'
    core.start(1., dry_run=False)
    assert core.update(state_input(1.)).reason == 'mechanical_calibration_required'


def test_cancel_waits_for_measured_stop_and_does_not_reset_stop_deadline():
    core = running_core()
    core.cancel(1.65)
    core.cancel(1.66)
    assert core.stop_started == 1.65
    assert core.update(state_input(1.68, speed=-.04)).state == 'STOPPING'
    core.update(state_input(1.8))
    assert core.update(state_input(2.4)).state == 'CANCELED'


def test_limited_run_cannot_be_reported_as_docked():
    core = running_core(DockConfig(calibrated=True, run_limit=.03))
    out = core.update(state_input(1.9, base=Pose2(-.031)))
    assert out.state == 'STOPPING'
    assert core.final_state == 'LIMITED_RUN_COMPLETE'
    assert out.velocity == 0.


def test_final_pose_requires_all_axes_and_stopped_confirmation():
    core = running_core()
    # No odometry jump: only the observation is near. A bad lateral/heading
    # estimate must fail even if distance alone is in the old 0.45-0.47 window.
    out = core.update(state_input(1.7, dock=Pose2(-.46, .03, .04)))
    assert out.state == 'STOPPING'
    assert core.final_state == 'FAILED'
    core = running_core()
    assert core.update(state_input(1.7, dock=Pose2(-.46))).state == 'STOPPING'
    assert core.update(state_input(1.8, dock=Pose2(-.46), speed=-.02)).state == 'STOPPING'
    core.update(state_input(1.9, dock=Pose2(-.46)))
    assert core.update(state_input(2.5, dock=Pose2(-.46))).state == 'SUCCEEDED'


def test_optional_charge_confirmation_waits_for_fresh_sustained_battery_state():
    core = RelativeDockingCore(DockConfig(calibrated=True, run_limit=2.8))
    core.start(1., False, True)
    core.update(state_input(1., dock=Pose2(-.46)))
    core.update(state_input(1.6, dock=Pose2(-.46)))
    core.update(state_input(1.7, dock=Pose2(-.46)))
    assert core.update(state_input(2.3, dock=Pose2(-.46))).state == 'WAIT_CHARGE'
    for t in (2.4, 3., 4.5):
        data = replace(state_input(t, dock=Pose2(-.46)), charging=True, charge_stamp=t)
        out = core.update(data)
    assert out.state == 'SUCCEEDED'
    assert out.reason == 'charging_confirmed'


def test_swept_footprint_detects_non_target_obstacle_and_preserves_shell_clearance():
    scan = scan_faces(1, faces=((Pose2(-.6), .185),))
    target = tuple(np.flatnonzero(np.isfinite(scan.ranges)))
    bounds = (.43, .40, .43)
    assert not collision(scan, Pose2(), target, -.06, 0., bounds)
    assert collision(scan, Pose2(), (), -.20, 0., bounds)
    assert not dock_plane_collision(Pose2(-.46), 0., 0., bounds)
    assert dock_plane_collision(Pose2(-.46), 0., 0., (.43, .47, .43))
    assert dock_plane_collision(Pose2(-.46), -.06, 0., bounds)
