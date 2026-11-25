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

"""Tests for Stage 1 rear-lidar metrics."""

import math
import json
from types import SimpleNamespace

import numpy as np

from charging_dock.stage1_lidar_metrics import analyze_scan
from charging_dock.stage1_lidar_metrics import summarize_samples


def _scan(sequence, noise=0.0, frame_id='rear_laser'):
    count = 1440
    angle_min = -math.pi
    increment = 2.0 * math.pi / count
    angles = angle_min + np.arange(count) * increment
    ranges = np.full(count, math.inf)
    target = np.abs(angles) <= math.radians(10.0)
    ranges[target] = 1.0 / np.cos(angles[target]) + noise
    stamp_ns = int(sequence * 1e9 / 30.0)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
            frame_id=frame_id,
        ),
        angle_min=angle_min,
        angle_increment=increment,
        ranges=ranges.tolist(),
        range_min=0.05,
        range_max=30.0,
    )


def test_line_geometry_and_summary_pass_provisional_gates():
    samples = []
    for sequence in range(300):
        noise = 0.001 * math.sin(sequence * 0.1)
        message = _scan(sequence, noise=noise)
        storage_stamp = message.header.stamp.sec * 1_000_000_000
        storage_stamp += message.header.stamp.nanosec + 2_000_000
        samples.append(analyze_scan(
            message,
            storage_stamp,
            roi_min_deg=-12.0,
            roi_max_deg=12.0,
            expected_range_m=1.0,
        ))

    metrics = summarize_samples(samples, expected_range_m=1.0)

    assert metrics['frame_ids'] == ['rear_laser']
    assert metrics['frequency_hz'] > 29.9
    assert metrics['valid_frame_rate'] == 1.0
    assert metrics['distance_std_m'] < 0.002
    assert abs(abs(metrics['line_yaw_mean_deg']) - 90.0) < 0.2
    assert metrics['line_residual_mean_m'] < 0.002
    assert metrics['visible_width_mean_m'] > 0.3
    assert metrics['provisional_pass']
    json.dumps(metrics)


def test_missing_target_fails_valid_frame_gate():
    samples = []
    for sequence in range(300):
        message = _scan(sequence)
        message.ranges = [math.inf] * len(message.ranges)
        samples.append(analyze_scan(
            message,
            int(sequence * 1e9 / 30.0),
            roi_min_deg=-12.0,
            roi_max_deg=12.0,
            expected_range_m=1.0,
        ))

    metrics = summarize_samples(samples, expected_range_m=1.0)

    assert metrics['valid_frame_rate'] == 0.0
    assert not metrics['checks']['valid_frame_rate']
    assert not metrics['provisional_pass']


def test_missing_ground_truth_cannot_pass_stage1():
    message = _scan(0)
    samples = [analyze_scan(
        message,
        sequence * 33_333_333,
        roi_min_deg=-12.0,
        roi_max_deg=12.0,
    ) for sequence in range(300)]

    metrics = summarize_samples(samples)

    assert not metrics['checks']['expected_range_available']
    assert not metrics['provisional_pass']


def test_full_circle_roi_includes_all_beams():
    message = _scan(0)
    sample = analyze_scan(
        message,
        storage_stamp_ns=0,
        roi_min_deg=-180.0,
        roi_max_deg=180.0,
    )

    assert sample.sector_beams == len(message.ranges)
    assert sample.raw_valid_beams > 0


def test_plane_normal_distance_is_used_for_plane_acceptance():
    samples = []
    for sequence in range(300):
        message = _scan(sequence)
        storage_stamp = sequence * 1_000_000_000 // 30
        samples.append(analyze_scan(
            message,
            storage_stamp,
            roi_min_deg=-12.0,
            roi_max_deg=12.0,
            expected_range_m=1.0,
        ))

    metrics = summarize_samples(
        samples,
        expected_range_m=1.0,
        expected_plane_normal_distance_m=1.0,
    )

    assert metrics['acceptance_distance_type'] == 'plane_normal'
    assert metrics['plane_normal_distance_mean_m'] is not None
    assert metrics['plane_normal_distance_std_m'] < 0.002
    assert abs(metrics['plane_normal_distance_bias_m']) < 0.002
    assert metrics['provisional_pass']


def test_plane_normal_distance_matches_known_vertical_plane():
    message = _scan(0)
    sample = analyze_scan(
        message,
        storage_stamp_ns=0,
        roi_min_deg=-12.0,
        roi_max_deg=12.0,
    )

    assert sample.plane_normal_distance_m is not None
    assert abs(sample.plane_normal_distance_m - 1.0) < 0.002


def test_plane_only_ground_truth_enables_plane_acceptance():
    samples = []
    for sequence in range(300):
        message = _scan(sequence)
        samples.append(analyze_scan(
            message,
            sequence * 1_000_000_000 // 30,
            roi_min_deg=-12.0,
            roi_max_deg=12.0,
        ))

    metrics = summarize_samples(
        samples,
        expected_plane_normal_distance_m=1.0,
    )

    assert metrics['checks']['expected_range_available']
    assert metrics['checks']['absolute_bias_m']
    assert metrics['provisional_pass']


def test_plane_normal_ground_truth_does_not_require_radial_ground_truth():
    samples = []
    for sequence in range(300):
        message = _scan(sequence)
        samples.append(analyze_scan(
            message,
            sequence * 1_000_000_000 // 30,
            roi_min_deg=-12.0,
            roi_max_deg=12.0,
        ))

    metrics = summarize_samples(
        samples,
        expected_plane_normal_distance_m=1.0,
    )

    assert metrics['acceptance_distance_type'] == 'plane_normal'
    assert metrics['checks']['expected_range_available']
    assert metrics['checks']['absolute_bias_m']
