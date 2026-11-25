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

"""Narrow locked-dock beam association preserves nearby obstacle checks."""

import math

from charging_dock.dock_collision import collision, locked_target_indices
from charging_dock.relative_geometry import Pose2
from relative_scan_fixture import scan_faces


SENSOR = Pose2(-0.43, -0.28, math.radians(225))
BACK_PLATE = Pose2(-0.517)


def scene():
    """Return one synthetic scan of the known backing plane."""
    return scan_faces(1.0, sensor=SENSOR, faces=((BACK_PLATE, 0.25),))


def set_local_return(scan, point):
    """Insert one return on the closest beam to a local-frame point."""
    sensor_point = point.relative_to(SENSOR)
    angle = math.atan2(sensor_point.y, sensor_point.x)
    index = round((angle - scan.angle_min) / scan.angle_increment)
    index %= len(scan.ranges)
    scan.ranges[index] = math.hypot(sensor_point.x, sensor_point.y)
    return index


def test_locked_face_mask_accepts_remaining_partial_back_plate_returns():
    scan = scene()
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance):
            continue
        angle = scan.angle_min + index * scan.angle_increment
        point = SENSOR.compose(Pose2(distance * math.cos(angle),
                                     distance * math.sin(angle))).relative_to(BACK_PLATE)
        if point.y > 0.01:
            scan.ranges[index] = math.inf

    expected = {index for index, distance in enumerate(scan.ranges)
                if math.isfinite(distance)}
    masked = set(locked_target_indices(scan, SENSOR, BACK_PLATE, 0.25, 0.05, 0.015))

    assert expected
    assert masked == expected


def test_locked_face_mask_rejects_returns_outside_width_and_depth():
    scan = scene()
    outside_width = set_local_return(scan, BACK_PLATE.compose(Pose2(0.0, 0.18)))
    outside_depth = set_local_return(scan, BACK_PLATE.compose(Pose2(0.025, 0.0)))

    masked = set(locked_target_indices(scan, SENSOR, BACK_PLATE, 0.25, 0.05, 0.015))

    assert outside_width not in masked
    assert outside_depth not in masked


def test_association_uses_scanned_plane_and_is_independent_of_local_origin():
    scan = scene()
    expected = locked_target_indices(scan, SENSOR, BACK_PLATE, 0.25)
    shifted_local = Pose2(4.2, -1.7, math.radians(63))

    assert locked_target_indices(
        scan, shifted_local.compose(SENSOR), shifted_local.compose(BACK_PLATE),
        0.25) == expected
    # The scan is above the protruding module. Masking around its physical
    # charging face, 57 mm closer to the car, cannot identify backing returns.
    physical_face = BACK_PLATE.compose(Pose2(0.057))
    assert expected
    assert locked_target_indices(scan, SENSOR, physical_face, 0.25) == ()


def test_masked_dock_returns_do_not_hide_nearby_shell_obstacle():
    scan = scene()
    obstacle_index = set_local_return(scan, Pose2(-0.40, -0.10))
    target_indices = locked_target_indices(scan, SENSOR, BACK_PLATE, 0.25, 0.05, 0.015)

    assert obstacle_index not in target_indices
    assert collision(scan, SENSOR, target_indices, 0.0, 0.0,
                     (0.43, 0.43, 0.43), margin=0.02, horizon=0.05)


def test_invalid_geometry_never_creates_a_target_mask():
    scan = scene()
    scan.angle_increment = 0.0

    assert locked_target_indices(scan, SENSOR, BACK_PLATE, 0.25) == ()
