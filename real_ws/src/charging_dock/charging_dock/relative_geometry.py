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

"""Planar transforms shared by lidar association, control and collision checks."""

from dataclasses import dataclass
import math


def pose2(message):
    """Convert a ROS-shaped pose/transform after quaternion validation."""
    q = message.orientation if hasattr(message, 'orientation') else message.rotation
    p = message.position if hasattr(message, 'position') else message.translation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    norm = sum(v * v for v in (q.x, q.y, q.z, q.w))
    if not all(math.isfinite(v) for v in values) or abs(norm - 1) > 0.01:
        raise ValueError('invalid_pose_or_quaternion')
    return Pose2(p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y**2 + q.z**2)))


def wrap(angle):
    """Return a signed angle in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2:
    """Pose of a child frame in its parent, in metres and radians."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def finite(self):
        """Check numeric validity before entering a control calculation."""
        return all(math.isfinite(v) for v in (self.x, self.y, self.yaw))

    def compose(self, child):
        """Compose parent->self with self->child."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2(self.x + c * child.x - s * child.y,
                     self.y + s * child.x + c * child.y,
                     wrap(self.yaw + child.yaw))

    def inverse(self):
        """Invert this transform."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2(-c * self.x - s * self.y,
                     s * self.x - c * self.y, wrap(-self.yaw))

    def relative_to(self, parent):
        """Express this pose in another frame with the same parent."""
        return parent.inverse().compose(self)


def age_valid(now, stamp, timeout):
    """Reject missing, stale, non-finite and future sensor timestamps."""
    return (all(math.isfinite(v) for v in (now, stamp, timeout))
            and -0.01 <= now - stamp <= timeout)


def integrate(pose, velocity, omega, dt):
    """Integrate a constant body twist, including reverse arcs."""
    turn = omega * dt
    if abs(omega) < 1e-9:
        delta = Pose2(velocity * dt, 0.0, turn)
    else:
        delta = Pose2(velocity * math.sin(turn) / omega,
                      velocity * (1.0 - math.cos(turn)) / omega, turn)
    return pose.compose(delta)


def longitudinal_speed(previous, current, dt):
    """Project measured translation onto the midpoint vehicle heading."""
    heading = previous.yaw + wrap(current.yaw - previous.yaw) / 2
    return ((current.x - previous.x) * math.cos(heading)
            + (current.y - previous.y) * math.sin(heading)) / dt
