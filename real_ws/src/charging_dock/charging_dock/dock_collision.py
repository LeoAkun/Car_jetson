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

"""Check the complete vehicle rectangle along a short predicted motion arc."""

import math

from charging_dock.relative_geometry import integrate, Pose2


def locked_target_indices(scan, local_sensor, local_face, width,
                          width_tolerance=0.05, depth_tolerance=0.015):
    """
    Associate only returns on the finite, locked scanned backing plane.

    All poses share the local odometry frame. The face is the scanned back
    plate, after undoing the physical charging-face offset. This deliberately
    does not exempt an angular sector or require both edges to remain visible.
    """
    if (not local_sensor.finite() or not local_face.finite()
            or not all(math.isfinite(v) for v in
                       (width, width_tolerance, depth_tolerance, scan.angle_min,
                        scan.angle_increment, scan.range_min, scan.range_max))
            or width <= 0 or width_tolerance < 0 or depth_tolerance <= 0
            or scan.angle_increment == 0 or scan.range_min < 0
            or scan.range_max <= scan.range_min):
        return ()
    face_sensor = local_sensor.relative_to(local_face)
    half_width = (width + width_tolerance) / 2
    indices = []
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min < distance <= scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        point = face_sensor.compose(Pose2(distance * math.cos(angle),
                                          distance * math.sin(angle)))
        if abs(point.x) <= depth_tolerance and abs(point.y) <= half_width:
            indices.append(index)
    return tuple(indices)


def dock_plane_collision(base_dock, velocity, omega, bounds,
                         margin=0.02, horizon=1.0, step=0.05):
    """Check shell clearance against a confirmed target plane."""
    front, rear, half_width = bounds
    if (not base_dock.finite() or min(*bounds, horizon, step) <= 0
            or margin < 0 or not all(math.isfinite(v) for v in
                                     (*bounds, velocity, omega, margin, horizon, step))):
        return True
    dock_base = base_dock.inverse()
    for n in range(int(math.ceil(horizon / step)) + 1):
        future = dock_base.compose(integrate(Pose2(), velocity, omega, n * step))
        for x in (-rear, front):
            for y in (-half_width, half_width):
                if future.compose(Pose2(x, y)).x < margin:
                    return True
    return False


def collision(scan, base_sensor, target_indices, velocity, omega,
              bounds, margin=0.04, horizon=1.0, step=0.05):
    """Only confirmed target returns are exempt; there is no angular blind spot."""
    front, rear, half_width = bounds
    if (min(front, rear, half_width, step) <= 0 or margin < 0 or horizon <= 0
            or not all(math.isfinite(v) for v in (*bounds, margin, horizon, step,
                                                  velocity, omega))):
        return True
    exempt = set(target_indices)
    poses = [integrate(Pose2(), velocity, omega, n * step)
             for n in range(int(math.ceil(horizon / step)) + 1)]
    for index, value in enumerate(scan.ranges):
        if index in exempt or not math.isfinite(value):
            continue
        if not max(0.0, scan.range_min) < value <= scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        point = base_sensor.compose(Pose2(value * math.cos(angle),
                                          value * math.sin(angle)))
        # Do not mask points inside the vehicle: unexpected self-returns must
        # stop until a measured sensor-specific mask has been configured.
        for pose in poses:
            p = point.relative_to(pose)
            if -rear - margin <= p.x <= front + margin and abs(p.y) <= half_width + margin:
                return True
    return False
