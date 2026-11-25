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

"""Finite dock-face detection and SE(2) tracking, without vehicle commands."""

from dataclasses import dataclass
import math

import numpy as np

from charging_dock.relative_geometry import age_valid, Pose2, wrap
from charging_dock.stage1_lidar_metrics import _contiguous_segments, _fit_line


@dataclass
class FaceConfig:
    """Measured finite face model; zero width deliberately disables detection."""

    width: float = 0.0
    width_tolerance: float = 0.025
    min_points: int = 6
    segment_jump: float = 0.08
    edge_depth: float = 0.006
    residual: float = 0.008
    min_range: float = 0.06
    position_gate: float = 0.35
    depth_gate: float = 0.15
    angle_gate: float = math.radians(20.0)
    stable_frames: int = 5
    timeout: float = 0.20
    filter_alpha: float = 0.5

    def __post_init__(self):
        """Validate gates instead of silently accepting an invalid model."""
        values = vars(self)
        if not all(math.isfinite(v) for v in values.values()):
            raise ValueError('non-finite face configuration')
        if (self.width < 0 or self.width_tolerance <= 0 or self.edge_depth <= 0
                or self.min_points < 3 or self.stable_frames < 1
                or min(self.segment_jump, self.residual, self.min_range,
                       self.position_gate, self.depth_gate,
                       self.angle_gate, self.timeout) <= 0
                or not 0 < self.filter_alpha <= 1):
            raise ValueError('invalid face configuration')


@dataclass
class FaceObservation:
    """Tracker pose is the scanned face; the ROS adapter projects it to dock."""

    stamp: float
    valid: bool = False
    pose: Pose2 = None
    quality: float = 0.0
    width: float = 0.0
    residual: float = 0.0
    reason: str = 'no_target'
    target_indices: tuple = ()


class RelativeFaceTracker:
    """Associate a complete finite face to a prior in a smooth local frame."""

    def __init__(self, config=None):
        self.config = config or FaceConfig()
        self.reset()

    def reset(self, prior=None):
        """Reset between actions; prior is local->dock, never an old scan."""
        self.target = prior
        self.anchor = prior
        self.last_stamp = None
        self.count = 0

    def process(self, scan, now, local_base, base_sensor):
        """Fit all candidates before gating; transform at the scan timestamp."""
        c = self.config
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
        invalid = FaceObservation(stamp)
        if not age_valid(now, stamp, c.timeout):
            self.count = 0
            invalid.reason = 'stale_scan'
            return invalid
        if self.last_stamp is not None and stamp <= self.last_stamp:
            invalid.reason = 'out_of_order_scan'
            return invalid
        previous_stamp = self.last_stamp
        self.last_stamp = stamp
        if previous_stamp is not None and stamp - previous_stamp > c.timeout:
            self.count = 0
        if (not local_base.finite() or not base_sensor.finite()
                or self.target is None or not self.target.finite()):
            invalid.reason = 'missing_pose_prior'
            return invalid
        if c.width <= 0:
            invalid.reason = 'face_width_not_calibrated'
            return invalid
        ranges = np.asarray(scan.ranges, dtype=float)
        if (len(ranges) < 3 or not math.isfinite(scan.angle_increment)
                or scan.angle_increment == 0 or not math.isfinite(scan.angle_min)):
            invalid.reason = 'invalid_scan_geometry'
            return invalid
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = (np.isfinite(ranges) & (ranges > max(c.min_range, scan.range_min))
                 & (ranges <= scan.range_max))
        indices = np.flatnonzero(valid)
        points = np.column_stack((ranges[indices] * np.cos(angles[indices]),
                                  ranges[indices] * np.sin(angles[indices])))
        segments = _contiguous_segments(indices, points, c.segment_jump)
        # Merge a real segment crossing the +/-pi seam of a full-circle scan.
        full = abs(abs(scan.angle_increment) * len(ranges) - 2 * math.pi) < 0.02
        if (full and len(segments) > 1 and segments[0][0][0] == 0
                and segments[-1][0][-1] == len(ranges) - 1
                and np.linalg.norm(segments[-1][1][-1] - segments[0][1][0])
                < c.segment_jump):
            merged = (np.concatenate((segments[-1][0], segments[0][0])),
                      np.vstack((segments[-1][1], segments[0][1])))
            segments = [merged] + segments[1:-1]
        local_sensor = local_base.compose(base_sensor)
        normal_yaw = self.target.yaw - local_sensor.yaw
        expected_normal = np.array([math.cos(normal_yaw), math.sin(normal_yaw)])
        # The 18.5cm charging face can sit on the 25cm back plate. Separate a
        # depth step along its predicted normal even when its range jump is
        # smaller than the generic clustering threshold.
        surfaces = []
        for beam_indices, segment in segments:
            cuts = np.flatnonzero(np.abs(np.diff(segment, axis=0) @ expected_normal)
                                  > c.edge_depth) + 1
            surfaces.extend(zip(np.split(beam_indices, cuts), np.split(segment, cuts)))
        candidates = []
        for beam_indices, segment in surfaces:
            if len(segment) < c.min_points:
                continue
            fit = _fit_line(segment)
            if (fit['residual'] > c.residual
                    or abs(fit['width'] - c.width) > c.width_tolerance):
                continue
            tangent = np.array([math.cos(fit['yaw']), math.sin(fit['yaw'])])
            centre = np.mean(segment, axis=0)
            projections = (segment - centre) @ tangent
            centre += tangent * (np.min(projections) + np.max(projections)) / 2
            normal = np.array([-tangent[1], tangent[0]])
            if np.dot(normal, -centre) < 0:
                normal = -normal
            # Both edges must open into no-return or a recessed surface;
            # missing/corrupt beams and the ends of a cropped FOV do not count.
            if not all(self._edge(ranges, angles, int(i), step, full, normal)
                       for i, step in ((beam_indices[0], -1),
                                       (beam_indices[-1], 1))):
                continue
            # A verified edge lies between the last return and its neighbour,
            # not at the last return itself. Intersect the half-beam rays with
            # the fitted line to avoid a shifting centre at oblique incidence.
            edge_angles = np.array([angles[beam_indices[0]] - scan.angle_increment / 2,
                                    angles[beam_indices[-1]] + scan.angle_increment / 2])
            rays = np.column_stack((np.cos(edge_angles), np.sin(edge_angles)))
            denominators = rays @ normal
            if np.any(np.abs(denominators) < 1e-9):
                continue
            distances = np.dot(centre, normal) / denominators
            if np.any(distances <= 0) or not np.all(np.isfinite(distances)):
                continue
            centre = np.mean(rays * distances[:, None], axis=0)
            sensor_face = Pose2(float(centre[0]), float(centre[1]),
                                math.atan2(normal[1], normal[0]))
            local_face = local_sensor.compose(sensor_face)
            error = local_face.relative_to(self.target)
            position_error = math.hypot(error.x, error.y)
            anchor_error = local_face.relative_to(self.anchor)
            if (position_error > c.position_gate or abs(error.yaw) > c.angle_gate
                    or abs(error.x) > c.depth_gate
                    or math.hypot(anchor_error.x, anchor_error.y) > c.position_gate
                    or abs(anchor_error.x) > c.depth_gate
                    or abs(anchor_error.yaw) > c.angle_gate):
                continue
            fit_error = max(fit['residual'] / c.residual,
                            0.5 * abs(fit['width'] - c.width) / c.width_tolerance)
            quality = max(0.0, 1.0 - fit_error)
            score = position_error / c.position_gate + abs(error.yaw) / c.angle_gate
            candidates.append((score, local_face, fit, quality, beam_indices))
        if not candidates:
            self.count = 0
            invalid.reason = 'no_complete_face_in_gate'
            return invalid
        candidates.sort(key=lambda item: item[0])
        if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < 0.15:
            self.count = 0
            invalid.reason = 'ambiguous_faces'
            return invalid
        _, face, fit, quality, beams = candidates[0]
        alpha = c.filter_alpha if self.count else 1.0
        self.target = Pose2(
            self.target.x + alpha * (face.x - self.target.x),
            self.target.y + alpha * (face.y - self.target.y),
            wrap(self.target.yaw + alpha * wrap(face.yaw - self.target.yaw)))
        self.count += 1
        ready = self.count >= c.stable_frames
        return FaceObservation(stamp, ready, self.target.relative_to(local_base),
                               quality, fit['width'], fit['residual'],
                               'ok' if ready else 'warming_up',
                               tuple(int(i) for i in beams))

    def _edge(self, ranges, angles, index, step, full, normal):
        neighbor = index + step
        if not full and not 0 <= neighbor < len(ranges):
            return False
        neighbor %= len(ranges)
        value = ranges[neighbor]
        # No-return is allowed beyond a known-width face; corrupt returns are not.
        if math.isinf(value) and value > 0:
            return True
        if not math.isfinite(value) or value <= 0:
            return False
        edge = ranges[index] * np.array([math.cos(angles[index]), math.sin(angles[index])])
        other = value * np.array([math.cos(angles[neighbor]), math.sin(angles[neighbor])])
        return float((edge - other) @ normal) > self.config.edge_depth
