# Copyright 2026 akun
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""Tests for the Stage 5 active docking control core."""

import pytest

from charging_dock.active_docking_controller import (
    ActiveDockingConfig,
    ActiveDockingCore,
    ActiveDockingInputs,
)
from charging_dock.docking_controller import StaticDockResult
from charging_dock.rear_lidar_processor import LidarObservation


def _lidar(distance=2.5, valid=True, obstacle=False, in_window=False,
           center_available=False):
    observation = LidarObservation(
        stamp_ns=0,
        valid=valid,
        distance_m=distance,
        plane_normal_distance_m=distance,
        yaw_deg=0.0,
        base_link_plane_normal_distance_m=distance,
        base_link_yaw_deg=0.0,
        lateral_center_m=0.0 if center_available else None,
        center_available=center_available,
        visible_width_m=0.2,
        fit_residual_m=0.001,
        quality=0.9 if valid else 0.0,
        target_points=20 if valid else 0,
        reason='ok' if valid else 'no_target_segment',
    )
    state = 'DRY_RUN_STOP_WINDOW' if in_window else 'DRY_RUN_TARGET_VALID'
    return StaticDockResult(
        state=state,
        reason='stop_window' if in_window else 'target_valid',
        in_stop_window=in_window,
        obstacle=obstacle,
        observation=observation,
    )


def _inputs(now=0.0, x=0.0, y=0.0, yaw=0.0, distance=2.5,
            speed=0.0, steer=0.0, autodrive=True, odom_age=0.0,
            vehicle_age=0.0, scan_age=0.0, lidar=None):
    return ActiveDockingInputs(
        now_sec=now,
        odom=(x, y, yaw),
        odom_age_sec=odom_age,
        vehicle_speed_mps=speed,
        vehicle_steer_deg=steer,
        vehicle_autodrive=autodrive,
        vehicle_age_sec=vehicle_age,
        lidar_result=lidar or _lidar(distance),
        scan_age_sec=scan_age,
    )


def _core(**overrides):
    values = {
        'motion_enabled': True,
        'planned_reverse_distance_m': 2.04,
        'run_distance_limit_m': 0.10,
        'precheck_stable_duration_sec': 0.0,
        'stop_stable_duration_sec': 0.0,
    }
    values.update(overrides)
    return ActiveDockingCore(ActiveDockingConfig(**values))


def _arm(core, inputs=None):
    sample = inputs or _inputs()
    core.enable(sample.now_sec)
    return core.update(sample)


def test_motion_interlock_finishes_precheck_without_nonzero_command():
    core = _core(motion_enabled=False)
    output = _arm(core)
    assert output.state == 'DRY_RUN_READY'
    assert output.command.linear.x == 0.0


def test_straight_mode_commands_reverse_without_steering():
    core = _core(run_distance_limit_m=0.50)
    _arm(core)
    output = core.update(_inputs(now=0.1, x=-0.01, distance=2.49))
    assert output.state == 'REVERSE_APPROACH'
    assert output.command.linear.x < 0.0
    assert output.command.angular.z == 0.0


def test_limited_run_stops_without_claiming_docked():
    core = _core(run_distance_limit_m=0.10)
    _arm(core)
    stopping = core.update(_inputs(now=1.0, x=-0.11, distance=2.39))
    assert stopping.state == 'STOPPING'
    assert stopping.command.linear.x == 0.0
    complete = core.update(_inputs(now=1.1, x=-0.11, distance=2.39))
    assert complete.state == 'LIMITED_RUN_COMPLETE'


def test_lidar_window_stops_and_requires_stable_vehicle():
    core = _core(run_distance_limit_m=1.0)
    _arm(core, _inputs(distance=0.60))
    stopping = core.update(
        _inputs(now=0.5, x=-0.13, speed=0.02, lidar=_lidar(0.47)))
    assert stopping.state == 'STOPPING'
    assert stopping.reason == 'lidar_stop_trigger'
    success = core.update(
        _inputs(now=0.6, x=-0.13, lidar=_lidar(0.46, in_window=True)))
    assert success.state == 'SUCCEEDED'


def test_sensor_timeout_during_motion_fails_closed():
    core = _core(run_distance_limit_m=0.50)
    _arm(core)
    output = core.update(
        _inputs(now=0.3, x=-0.01, distance=2.49, scan_age=0.21))
    assert output.state == 'STOPPING'
    assert output.reason == 'rear_lidar_timeout'
    assert output.command.linear.x == 0.0
    failed = core.update(_inputs(now=0.4, x=-0.01, distance=2.49))
    assert failed.state == 'FAILED'


def test_obstacle_rejects_attempt_before_motion():
    core = _core()
    output = _arm(core, _inputs(lidar=_lidar(obstacle=True)))
    assert output.state == 'FAILED'
    assert output.reason == 'non_target_obstacle'
    assert output.command.linear.x == 0.0


def test_p0_error_outside_tolerance_rejects_attempt():
    core = _core(
        start_pose_validation_enabled=True,
        p0_target_x=1.0,
        p0_longitudinal_tolerance_m=0.05,
    )
    output = _arm(core, _inputs(x=1.10))
    assert output.state == 'FAILED'
    assert output.reason == 'start_pose_out_of_range'


def test_lateral_correction_requires_a_valid_finite_center():
    core = _core(lateral_correction_enabled=True)
    core.enable(0.0)
    output = core.update(_inputs())
    assert output.state == 'ACQUIRE_TARGET'
    assert output.command.linear.x == 0.0


def test_initial_steer_outside_limit_rejects_attempt():
    core = _core(max_actual_steer_deg=5.0)
    output = _arm(core, _inputs(steer=6.0))
    assert output.state == 'FAILED'
    assert output.reason == 'initial_steer_out_of_range'


def test_invalid_motion_limits_are_rejected_at_startup():
    with pytest.raises(ValueError, match='run_distance_limit_m'):
        _core(run_distance_limit_m=2.5, max_motion_distance_m=2.3)
