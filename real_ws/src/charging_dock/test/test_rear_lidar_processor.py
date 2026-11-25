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

"""Tests for the Stage 2 rear lidar processor."""

import math
from types import SimpleNamespace

import numpy as np

from charging_dock.rear_lidar_processor import RearLidarProcessor


def _scan(frame_id='rear_laser', target=True, distance=1.0,
          half_width_deg=10.0):
    count = 1440
    angle_min = -math.pi
    increment = 2.0 * math.pi / count
    angles = angle_min + np.arange(count) * increment
    ranges = np.full(count, math.inf)
    if target:
        mask = np.abs(angles) < math.radians(half_width_deg)
        ranges[mask] = distance / np.cos(angles[mask])
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame_id,
            stamp=SimpleNamespace(sec=1, nanosec=0)),
        angle_min=angle_min, angle_increment=increment,
        ranges=ranges.tolist(), range_min=0.05, range_max=30.0)


def test_target_becomes_valid_after_temporal_warmup():
    processor = RearLidarProcessor(-12, 12, expected_range_m=1.0,
                                   stable_frames=3)
    results = [processor.process(_scan(), 1_000_000_000) for _ in range(3)]
    assert [result.valid for result in results] == [False, False, True]
    assert results[-1].quality > 0.0
    assert abs(results[-1].distance_m - 1.0) < 0.01
    assert results[-1].base_link_plane_normal_distance_m is not None
    assert abs(results[-1].base_link_yaw_deg + 45.0) < 0.2
    assert not results[-1].center_available
    assert results[-1].lateral_center_m is None


def test_missing_target_fails_closed():
    processor = RearLidarProcessor(-12, 12, stable_frames=1)
    result = processor.process(_scan(target=False), 1_000_000_000)
    assert not result.valid
    assert result.reason == 'no_target_segment'
    assert result.quality == 0.0


def test_unexpected_frame_fails_closed():
    processor = RearLidarProcessor(-12, 12, stable_frames=1)
    result = processor.process(_scan(frame_id='laser'), 1_000_000_000)
    assert not result.valid
    assert result.reason == 'unexpected_frame'


def test_stale_message_fails_closed():
    processor = RearLidarProcessor(-12, 12, stable_frames=1)
    result = processor.process(_scan(), 1_300_000_000)
    assert not result.valid
    assert result.reason == 'stale_timestamp'


def test_isolated_person_like_returns_fail_minimum_points():
    message = _scan(target=False)
    ranges = np.asarray(message.ranges)
    ranges[719:722] = 1.0
    message.ranges = ranges.tolist()
    processor = RearLidarProcessor(-12, 12, stable_frames=1)
    result = processor.process(message, 1_000_000_000)
    assert not result.valid
    assert result.reason == 'no_target_segment'


def test_narrow_box_like_segment_is_rejected():
    processor = RearLidarProcessor(-12, 12, stable_frames=1,
                                   min_width_m=0.05)
    result = processor.process(
        _scan(half_width_deg=0.5), 1_000_000_000)
    assert not result.valid
    assert result.reason == 'insufficient_visible_width'


def test_infinite_wall_never_exports_a_lateral_center():
    processor = RearLidarProcessor(-30, 30, stable_frames=1,
                                   min_width_m=0.05,
                                   max_width_m=2.0)
    result = processor.process(
        _scan(half_width_deg=90), 1_000_000_000)
    assert result.valid
    assert not result.center_available
    assert result.lateral_center_m is None


def test_excessively_wide_structure_is_not_a_dock_candidate():
    processor = RearLidarProcessor(-30, 30, stable_frames=1,
                                   max_width_m=0.60)
    result = processor.process(
        _scan(half_width_deg=90), 1_000_000_000)
    assert not result.valid
    assert result.reason == 'excessive_visible_width'


def test_target_jump_resets_temporal_validation():
    processor = RearLidarProcessor(-12, 12, stable_frames=1,
                                   max_distance_jump_m=0.10)
    assert processor.process(
        _scan(distance=1.0), 1_000_000_000).valid
    result = processor.process(
        _scan(distance=1.2), 1_000_000_000)
    assert not result.valid
    assert result.reason == 'target_jump'
    assert processor.process(
        _scan(distance=1.2), 1_000_000_000).valid


def test_target_outside_expected_range_gate_is_rejected():
    processor = RearLidarProcessor(
        -12, 12, expected_range_m=1.0,
        expected_range_tolerance_m=0.10, stable_frames=1)
    result = processor.process(
        _scan(distance=1.2), 1_000_000_000)
    assert not result.valid
    assert result.reason == 'expected_range_mismatch'
