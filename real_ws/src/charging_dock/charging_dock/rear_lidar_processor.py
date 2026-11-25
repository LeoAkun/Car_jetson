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

"""
Offline-safe rear lidar geometry processing for charging-dock docking.

The processor deliberately produces observations only.  It never owns or
publishes a vehicle command.  The final parking window is fixed by the
project configuration to 0.45--0.47 m in the vehicle/base_link reference.
"""

from dataclasses import asdict, dataclass
import math
from typing import Optional

from charging_dock.stage1_lidar_metrics import analyze_scan


@dataclass
class LidarObservation:
    """One filtered geometry observation."""

    stamp_ns: int
    valid: bool
    distance_m: Optional[float]
    plane_normal_distance_m: Optional[float]
    yaw_deg: Optional[float]
    base_link_plane_normal_distance_m: Optional[float]
    base_link_yaw_deg: Optional[float]
    lateral_center_m: Optional[float]
    center_available: bool
    visible_width_m: Optional[float]
    fit_residual_m: Optional[float]
    quality: float
    target_points: int
    reason: str

    def to_dict(self):
        """Return a JSON-compatible dictionary."""
        return asdict(self)


def _finite(value):
    return value is not None and math.isfinite(float(value))


def _angle_error_deg(angle):
    """Map a line angle to a continuous signed angle in [-90, 90]."""
    if not _finite(angle):
        return None
    value = math.degrees(float(angle))
    while value > 90.0:
        value -= 180.0
    while value < -90.0:
        value += 180.0
    return value


class RearLidarProcessor:
    """Extract and temporally validate the configured target structure."""

    def __init__(self, roi_min_deg, roi_max_deg, expected_range_m=None,
                 minimum_points=5, jump_threshold_m=0.08,
                 maximum_age_sec=0.20, stable_frames=10,
                 max_distance_jump_m=0.10, min_width_m=0.05,
                 max_width_m=0.60, max_fit_residual_m=0.01,
                 expected_range_tolerance_m=None,
                 expected_frames=('rear_laser',),
                 lateral_center_enabled=False, sensor_x_m=-0.43,
                 sensor_y_m=-0.28, sensor_yaw_deg=-135.0):
        self.roi_min_deg = float(roi_min_deg)
        self.roi_max_deg = float(roi_max_deg)
        self.expected_range_m = expected_range_m
        self.minimum_points = int(minimum_points)
        self.jump_threshold_m = float(jump_threshold_m)
        self.maximum_age_sec = float(maximum_age_sec)
        self.stable_frames = int(stable_frames)
        self.max_distance_jump_m = float(max_distance_jump_m)
        self.min_width_m = float(min_width_m)
        self.max_width_m = float(max_width_m)
        self.max_fit_residual_m = float(max_fit_residual_m)
        self.expected_range_tolerance_m = (
            None if expected_range_tolerance_m is None
            else abs(float(expected_range_tolerance_m))
        )
        self.expected_frames = frozenset(expected_frames)
        self.lateral_center_enabled = bool(lateral_center_enabled)
        self.sensor_x_m = float(sensor_x_m)
        self.sensor_y_m = float(sensor_y_m)
        self.sensor_yaw_rad = math.radians(float(sensor_yaw_deg))
        self._last_valid = None
        self._stable_count = 0

    def process(self, message, storage_stamp_ns):
        """Process one LaserScan and return a fail-closed observation."""
        message_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        age_sec = (int(storage_stamp_ns) - message_stamp_ns) / 1e9
        if age_sec > self.maximum_age_sec or age_sec < -0.05:
            self._stable_count = 0
            return self._invalid(message_stamp_ns, 'stale_timestamp')
        if str(message.header.frame_id) not in self.expected_frames:
            self._stable_count = 0
            return self._invalid(message_stamp_ns, 'unexpected_frame')

        sample = analyze_scan(
            message, storage_stamp_ns, self.roi_min_deg, self.roi_max_deg,
            expected_range_m=self.expected_range_m,
            minimum_points=self.minimum_points,
            jump_threshold_m=self.jump_threshold_m,
        )
        distance = sample.distance_m
        width = sample.visible_width_m
        residual = sample.line_residual_m
        valid = _finite(distance) and _finite(width) and _finite(residual)
        reason = 'ok'
        if not valid:
            reason = 'no_target_segment'
        elif width < self.min_width_m:
            valid = False
            reason = 'insufficient_visible_width'
        elif width > self.max_width_m:
            valid = False
            reason = 'excessive_visible_width'
        elif residual > self.max_fit_residual_m:
            valid = False
            reason = 'fit_residual'
        elif (self.expected_range_m is not None
              and self.expected_range_tolerance_m is not None
              and abs(distance - self.expected_range_m)
              > self.expected_range_tolerance_m):
            valid = False
            reason = 'expected_range_mismatch'
        elif self._last_valid is not None and abs(
                distance - self._last_valid.distance_m) > self.max_distance_jump_m:
            valid = False
            reason = 'target_jump'

        if valid:
            self._stable_count += 1
            self._last_valid = sample
        else:
            self._stable_count = 0
            if reason == 'target_jump':
                self._last_valid = sample

        quality = self._quality(sample, valid)
        observation_valid = bool(valid and self._stable_count >= self.stable_frames)
        center_available = bool(
            observation_valid and self.lateral_center_enabled
            and width is not None and width >= self.min_width_m
        )
        output_reason = reason
        if valid and not observation_valid:
            output_reason = 'warming_up'
        base_distance, base_yaw, base_center_y = self._base_geometry(sample)
        return LidarObservation(
            stamp_ns=message_stamp_ns,
            valid=observation_valid,
            distance_m=distance,
            plane_normal_distance_m=sample.plane_normal_distance_m,
            yaw_deg=_angle_error_deg(sample.line_yaw_rad),
            base_link_plane_normal_distance_m=base_distance,
            base_link_yaw_deg=base_yaw,
            lateral_center_m=base_center_y if center_available else None,
            center_available=center_available,
            visible_width_m=width,
            fit_residual_m=residual,
            quality=quality,
            target_points=sample.target_points,
            reason=output_reason,
        )

    def _invalid(self, stamp_ns, reason):
        return LidarObservation(
            stamp_ns=stamp_ns, valid=False, distance_m=None,
            plane_normal_distance_m=None, yaw_deg=None,
            base_link_plane_normal_distance_m=None,
            base_link_yaw_deg=None,
            lateral_center_m=None, center_available=False,
            visible_width_m=None,
            fit_residual_m=None, quality=0.0, target_points=0,
            reason=reason,
        )

    def _base_geometry(self, sample):
        if not (_finite(sample.center_x_m) and _finite(sample.center_y_m)
                and _finite(sample.line_yaw_rad)):
            return None, None, None
        cosine = math.cos(self.sensor_yaw_rad)
        sine = math.sin(self.sensor_yaw_rad)
        center_x = (
            self.sensor_x_m + cosine * sample.center_x_m
            - sine * sample.center_y_m
        )
        center_y = (
            self.sensor_y_m + sine * sample.center_x_m
            + cosine * sample.center_y_m
        )
        yaw = sample.line_yaw_rad + self.sensor_yaw_rad
        normal_x = -math.sin(yaw)
        normal_y = math.cos(yaw)
        distance = abs(center_x * normal_x + center_y * normal_y)
        return distance, _angle_error_deg(yaw), center_y

    @staticmethod
    def _quality(sample, valid):
        if not valid:
            return 0.0
        point_score = min(1.0, sample.target_points / 20.0)
        residual_score = max(0.0, 1.0 - sample.line_residual_m / 0.01)
        width_score = min(1.0, max(0.0, sample.visible_width_m / 0.20))
        return float(0.4 * point_score + 0.4 * residual_score + 0.2 * width_score)
