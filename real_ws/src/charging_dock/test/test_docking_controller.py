"""Tests for the Stage 4 static lidar integration core."""

import math
from types import SimpleNamespace

import numpy as np

from charging_dock.docking_controller import DockingStaticCore
from charging_dock.rear_lidar_processor import RearLidarProcessor


def _scan(distance=1.0, frame_id='rear_laser', target=True,
          half_width_deg=10.0, extra_obstacle=None):
    count = 1440
    angle_min = -math.pi
    increment = 2.0 * math.pi / count
    angles = angle_min + np.arange(count) * increment
    ranges = np.full(count, math.inf)
    if target:
        mask = np.abs(angles) < math.radians(half_width_deg)
        ranges[mask] = distance / np.cos(angles[mask])
    if extra_obstacle:
        angle_deg, obstacle_distance = extra_obstacle
        index = int(round((math.radians(angle_deg) - angle_min) / increment))
        if 0 <= index < count:
            ranges[index:index + 3] = obstacle_distance
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id,
                               stamp=SimpleNamespace(sec=1, nanosec=0)),
        angle_min=angle_min, angle_increment=increment,
        ranges=ranges.tolist(), range_min=0.05, range_max=30.0)


def _core(stable_frames=1):
    processor = RearLidarProcessor(
        -30.0, 30.0, expected_range_m=None, stable_frames=stable_frames,
        expected_frames=('rear_laser',), sensor_x_m=0.0, sensor_y_m=0.0,
        sensor_yaw_deg=0.0, max_width_m=2.0)
    return DockingStaticCore(processor, stop_min_m=0.45,
                             stop_max_m=0.47, expected_angle_deg=90.0,
                             angle_tolerance_deg=5.0,
                             target_roi_min_deg=-30.0,
                             target_roi_max_deg=30.0)


def test_stop_window_accepts_center_and_rejects_outside():
    core = _core()
    center = core.evaluate(_scan(distance=0.46), 1_000_000_000)
    outside = core.evaluate(_scan(distance=0.50), 1_000_000_000)
    assert center.state == 'DRY_RUN_STOP_WINDOW'
    assert center.in_stop_window
    assert outside.state == 'DRY_RUN_TARGET_VALID'
    assert not outside.in_stop_window


def test_temporal_warmup_is_not_declared_valid():
    core = _core(stable_frames=3)
    results = [core.evaluate(_scan(distance=0.46), 1_000_000_000)
               for _ in range(3)]
    assert results[0].state in ('DRY_RUN_WARMING_UP', 'DRY_RUN_NO_TARGET')
    assert results[1].observation.valid is False
    assert results[2].state == 'DRY_RUN_STOP_WINDOW'


def test_stale_and_wrong_frame_fail_closed():
    core = _core()
    stale = core.evaluate(_scan(distance=0.46), 1_300_000_000)
    wrong = core.evaluate(
        _scan(distance=0.46, frame_id='laser'), 1_000_000_000)
    assert stale.state == 'DRY_RUN_STALE_SCAN'
    assert wrong.state == 'DRY_RUN_WRONG_FRAME'
    assert not stale.observation.valid
    assert not wrong.observation.valid


def test_target_jump_is_rejected_until_stable_again():
    core = _core()
    assert core.evaluate(_scan(distance=1.0), 1_000_000_000).observation.valid
    jumped = core.evaluate(_scan(distance=1.2), 1_000_000_000)
    assert jumped.state == 'DRY_RUN_TARGET_JUMP'
    assert not jumped.observation.valid


def test_non_target_close_return_is_obstacle():
    core = _core()
    result = core.evaluate(_scan(distance=0.46, extra_obstacle=(90.0, 0.3)),
                           1_000_000_000)
    assert result.state == 'DRY_RUN_OBSTACLE'
    assert result.obstacle


def test_close_return_inside_target_roi_is_also_an_obstacle():
    core = _core()
    result = core.evaluate(_scan(distance=0.46, extra_obstacle=(20.0, 0.2)),
                           1_000_000_000)
    assert result.state == 'DRY_RUN_OBSTACLE'
    assert result.obstacle


def test_infinite_wall_does_not_enable_center():
    core = _core()
    result = core.evaluate(_scan(distance=0.46, half_width_deg=90.0),
                           1_000_000_000)
    assert result.observation.valid
    assert not result.observation.center_available
    assert result.observation.lateral_center_m is None
