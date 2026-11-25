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

"""Metrics used by the Stage 1 rear-lidar characterization tools."""

from dataclasses import asdict
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class ScanSample:
    """Measurements extracted from one LaserScan frame."""

    message_stamp_ns: int
    storage_stamp_ns: int
    frame_id: str
    sector_beams: int
    raw_valid_beams: int
    target_points: int
    distance_m: float | None
    plane_normal_distance_m: float | None
    line_yaw_rad: float | None
    line_residual_m: float | None
    visible_width_m: float | None
    center_x_m: float | None
    center_y_m: float | None

    def to_dict(self):
        """Return a JSON-compatible representation."""
        return asdict(self)


def _message_stamp_ns(message):
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _angle_mask(angles, minimum_deg, maximum_deg):
    if maximum_deg - minimum_deg >= 360.0:
        return np.ones_like(angles, dtype=bool)
    minimum = math.radians(minimum_deg)
    maximum = math.radians(maximum_deg)
    normalized = (angles + math.pi) % (2.0 * math.pi) - math.pi
    minimum = (minimum + math.pi) % (2.0 * math.pi) - math.pi
    maximum = (maximum + math.pi) % (2.0 * math.pi) - math.pi
    if minimum <= maximum:
        return (normalized >= minimum) & (normalized <= maximum)
    return (normalized >= minimum) | (normalized <= maximum)


def _contiguous_segments(indices, points, jump_threshold_m):
    if not len(indices):
        return []
    segments = []
    start = 0
    for offset in range(1, len(indices)):
        adjacent = indices[offset] == indices[offset - 1] + 1
        jump = np.linalg.norm(points[offset] - points[offset - 1])
        if not adjacent or jump > jump_threshold_m:
            segments.append((indices[start:offset], points[start:offset]))
            start = offset
    segments.append((indices[start:], points[start:]))
    return segments


def _select_segment(segments, ranges, expected_range_m, minimum_points):
    candidates = [segment for segment in segments
                  if len(segment[0]) >= minimum_points]
    if not candidates:
        return None
    if expected_range_m is None:
        return max(candidates, key=lambda segment: len(segment[0]))
    return min(
        candidates,
        key=lambda segment: (
            abs(float(np.median(ranges[segment[0]])) - expected_range_m),
            -len(segment[0]),
        ),
    )


def _fit_line(points):
    center = np.mean(points, axis=0)
    centered = points - center
    covariance = centered.T @ centered / len(points)
    _, vectors = np.linalg.eigh(covariance)
    direction = vectors[:, -1]
    if direction[0] < 0.0:
        direction = -direction
    normal = np.array([-direction[1], direction[0]])
    along = centered @ direction
    residual = centered @ normal
    yaw = math.atan2(direction[1], direction[0])
    return {
        'yaw': yaw,
        'residual': float(np.sqrt(np.mean(np.square(residual)))),
        'width': float(np.max(along) - np.min(along)),
        'center_x': float(center[0]),
        'center_y': float(center[1]),
        'plane_normal_distance': float(abs(np.dot(center, normal))),
    }


def analyze_scan(
        message,
        storage_stamp_ns,
        roi_min_deg,
        roi_max_deg,
        expected_range_m=None,
        minimum_points=5,
        jump_threshold_m=0.08):
    """Extract the target segment and geometry from one LaserScan."""
    ranges = np.asarray(message.ranges, dtype=np.float64)
    angles = (
        float(message.angle_min)
        + np.arange(len(ranges), dtype=np.float64)
        * float(message.angle_increment)
    )
    sector = _angle_mask(angles, roi_min_deg, roi_max_deg)
    lower = max(0.0, float(message.range_min))
    upper = float(message.range_max)
    valid = sector & np.isfinite(ranges) & (ranges > lower) & (ranges <= upper)
    indices = np.flatnonzero(valid)
    points = np.column_stack((
        ranges[indices] * np.cos(angles[indices]),
        ranges[indices] * np.sin(angles[indices]),
    ))
    segments = _contiguous_segments(indices, points, jump_threshold_m)
    selected = _select_segment(
        segments, ranges, expected_range_m, minimum_points
    )
    geometry = None
    distance = None
    target_points = 0
    if selected is not None:
        selected_indices, selected_points = selected
        target_points = len(selected_indices)
        distance = float(np.median(ranges[selected_indices]))
        geometry = _fit_line(selected_points)

    return ScanSample(
        message_stamp_ns=_message_stamp_ns(message),
        storage_stamp_ns=int(storage_stamp_ns),
        frame_id=str(message.header.frame_id),
        sector_beams=int(np.count_nonzero(sector)),
        raw_valid_beams=int(np.count_nonzero(valid)),
        target_points=target_points,
        distance_m=distance,
        plane_normal_distance_m=(
            None if geometry is None
            else geometry['plane_normal_distance']
        ),
        line_yaw_rad=None if geometry is None else geometry['yaw'],
        line_residual_m=None if geometry is None else geometry['residual'],
        visible_width_m=None if geometry is None else geometry['width'],
        center_x_m=None if geometry is None else geometry['center_x'],
        center_y_m=None if geometry is None else geometry['center_y'],
    )


def _values(samples, attribute):
    return np.asarray([
        value for value in (getattr(sample, attribute) for sample in samples)
        if value is not None and math.isfinite(value)
    ], dtype=np.float64)


def _mean_std(values):
    if not len(values):
        return None, None
    return float(np.mean(values)), float(np.std(values))


def _axial_mean_std(values):
    if not len(values):
        return None, None
    doubled = values * 2.0
    vector = np.mean(np.exp(1j * doubled))
    mean = 0.5 * math.atan2(vector.imag, vector.real)
    magnitude = min(1.0, max(1e-12, abs(vector)))
    std = 0.5 * math.sqrt(-2.0 * math.log(magnitude))
    return math.degrees(mean), math.degrees(std)


def summarize_samples(
        samples,
        expected_range_m=None,
        expected_plane_normal_distance_m=None,
        minimum_frames=250,
        minimum_frequency_hz=25.0,
        maximum_gap_sec=0.2,
        minimum_valid_frame_rate=0.95,
        maximum_distance_std_m=0.01,
        maximum_abs_bias_m=0.02,
        maximum_outlier_rate=0.01):
    """Aggregate Stage 1 metrics and evaluate the provisional gates."""
    if not samples:
        raise ValueError('no LaserScan samples were provided')
    ordered = sorted(samples, key=lambda sample: sample.storage_stamp_ns)
    storage_stamps = np.asarray(
        [sample.storage_stamp_ns for sample in ordered], dtype=np.int64
    )
    gaps = np.diff(storage_stamps) / 1e9
    duration = (storage_stamps[-1] - storage_stamps[0]) / 1e9
    frequency = (len(ordered) - 1) / duration if duration > 0.0 else 0.0
    max_gap = float(np.max(gaps)) if len(gaps) else 0.0

    valid_samples = [sample for sample in ordered
                     if sample.distance_m is not None]
    distances = _values(valid_samples, 'distance_m')
    distance_mean, distance_std = _mean_std(distances)
    distance_median = float(np.median(distances)) if len(distances) else None
    bias = None
    if distance_mean is not None and expected_range_m is not None:
        bias = distance_mean - expected_range_m

    plane_distances = _values(valid_samples, 'plane_normal_distance_m')
    plane_distance_mean, plane_distance_std = _mean_std(plane_distances)
    plane_distance_median = (
        float(np.median(plane_distances))
        if len(plane_distances) else None
    )
    plane_bias = None
    if (plane_distance_mean is not None
            and expected_plane_normal_distance_m is not None):
        plane_bias = (
            plane_distance_mean - expected_plane_normal_distance_m
        )

    outlier_rate = None
    outlier_threshold = None
    if len(distances):
        median = float(np.median(distances))
        mad = float(np.median(np.abs(distances - median)))
        outlier_threshold = max(0.03, 3.0 * 1.4826 * mad)
        outlier_rate = float(np.mean(np.abs(distances - median)
                                     > outlier_threshold))

    plane_outlier_rate = None
    plane_outlier_threshold = None
    if len(plane_distances):
        plane_median = float(np.median(plane_distances))
        plane_mad = float(
            np.median(np.abs(plane_distances - plane_median))
        )
        plane_outlier_threshold = max(0.03, 3.0 * 1.4826 * plane_mad)
        plane_outlier_rate = float(
            np.mean(
                np.abs(plane_distances - plane_median)
                > plane_outlier_threshold
            )
        )

    acceptance_type = (
        'plane_normal' if expected_plane_normal_distance_m is not None
        else 'radial'
    )
    acceptance_std = (
        plane_distance_std if acceptance_type == 'plane_normal'
        else distance_std
    )
    acceptance_bias = (
        plane_bias if acceptance_type == 'plane_normal' else bias
    )
    acceptance_outlier_rate = (
        plane_outlier_rate if acceptance_type == 'plane_normal'
        else outlier_rate
    )

    yaw_mean, yaw_std = _axial_mean_std(
        _values(valid_samples, 'line_yaw_rad')
    )
    residual_mean, residual_std = _mean_std(
        _values(valid_samples, 'line_residual_m')
    )
    width_mean, width_std = _mean_std(
        _values(valid_samples, 'visible_width_m')
    )
    center_x_mean, center_x_std = _mean_std(
        _values(valid_samples, 'center_x_m')
    )
    center_y_mean, center_y_std = _mean_std(
        _values(valid_samples, 'center_y_m')
    )

    ages = np.asarray([
        (sample.storage_stamp_ns - sample.message_stamp_ns) / 1e9
        for sample in ordered if sample.message_stamp_ns > 0
    ], dtype=np.float64)
    sector_beams = sum(sample.sector_beams for sample in ordered)
    raw_valid = sum(sample.raw_valid_beams for sample in ordered)
    target_points = sum(sample.target_points for sample in ordered)
    valid_frame_rate = len(valid_samples) / len(ordered)

    checks = {
        'expected_range_available': (
            expected_range_m is not None
            or expected_plane_normal_distance_m is not None
        ),
        'minimum_frames': bool(len(ordered) >= minimum_frames),
        'frequency_hz': bool(frequency >= minimum_frequency_hz),
        'maximum_gap_sec': bool(max_gap <= maximum_gap_sec),
        'valid_frame_rate': bool(
            valid_frame_rate >= minimum_valid_frame_rate
        ),
        'distance_std_m': bool(
            acceptance_std is not None
            and acceptance_std <= maximum_distance_std_m
        ),
        'outlier_rate': bool(
            acceptance_outlier_rate is not None
            and acceptance_outlier_rate <= maximum_outlier_rate
        ),
    }
    if expected_plane_normal_distance_m is not None:
        checks['expected_plane_normal_distance_available'] = True
    if (expected_range_m is not None
            or expected_plane_normal_distance_m is not None):
        checks['absolute_bias_m'] = bool(
            acceptance_bias is not None
            and abs(acceptance_bias) <= maximum_abs_bias_m
        )

    return {
        'frame_count': len(ordered),
        'duration_sec': duration,
        'frequency_hz': frequency,
        'maximum_gap_sec': max_gap,
        'frame_ids': sorted({sample.frame_id for sample in ordered}),
        'message_age_mean_sec': float(np.mean(ages)) if len(ages) else None,
        'message_age_p95_sec': (
            float(np.percentile(ages, 95)) if len(ages) else None
        ),
        'valid_frame_rate': valid_frame_rate,
        'raw_valid_return_rate': (
            raw_valid / sector_beams if sector_beams else 0.0
        ),
        'target_return_rate': (
            target_points / sector_beams if sector_beams else 0.0
        ),
        'distance_mean_m': distance_mean,
        'distance_median_m': distance_median,
        'distance_std_m': distance_std,
        'distance_bias_m': bias,
        'outlier_threshold_m': outlier_threshold,
        'outlier_rate': outlier_rate,
        'plane_normal_distance_mean_m': plane_distance_mean,
        'plane_normal_distance_median_m': plane_distance_median,
        'plane_normal_distance_std_m': plane_distance_std,
        'plane_normal_distance_bias_m': plane_bias,
        'plane_normal_outlier_threshold_m': plane_outlier_threshold,
        'plane_normal_outlier_rate': plane_outlier_rate,
        'acceptance_distance_type': acceptance_type,
        'acceptance_distance_mean_m': (
            plane_distance_mean if acceptance_type == 'plane_normal'
            else distance_mean
        ),
        'acceptance_distance_std_m': acceptance_std,
        'acceptance_distance_bias_m': acceptance_bias,
        'acceptance_outlier_rate': acceptance_outlier_rate,
        'line_yaw_mean_deg': yaw_mean,
        'line_yaw_std_deg': yaw_std,
        'line_residual_mean_m': residual_mean,
        'line_residual_std_m': residual_std,
        'visible_width_mean_m': width_mean,
        'visible_width_std_m': width_std,
        'center_x_mean_m': center_x_mean,
        'center_x_std_m': center_x_std,
        'center_y_mean_m': center_y_mean,
        'center_y_std_m': center_y_std,
        'checks': checks,
        'provisional_pass': bool(all(checks.values())),
    }
