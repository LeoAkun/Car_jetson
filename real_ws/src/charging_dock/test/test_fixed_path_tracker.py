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

import math

from charging_dock.fixed_path_tracker import FixedPathTrackerCore


def test_reverse_progress_is_positive_when_x_decreases():
    tracker = FixedPathTrackerCore(reverse_distance=2.5)
    tracker.start(0.0, 0.0, 0.0, 0.0)
    output = tracker.update(-1.0, 0.0, 0.0, 1.0)
    assert output.state == 'REVERSE_APPROACH'
    assert math.isclose(output.traveled_m, 1.0)
    assert output.command.linear.x < 0.0


def test_target_distance_stops_and_reports_success():
    tracker = FixedPathTrackerCore(reverse_distance=1.0)
    tracker.start(0.0, 0.0, 0.0, 0.0)
    output = tracker.update(-1.01, 0.0, 0.0, 2.0)
    assert output.state == 'SUCCEEDED'
    assert output.reason == 'target_reached'
    assert output.command.linear.x == 0.0


def test_lateral_error_fails_closed():
    tracker = FixedPathTrackerCore(max_lateral_error=0.10)
    tracker.start(0.0, 0.0, 0.0, 0.0)
    output = tracker.update(-0.1, 0.2, 0.0, 1.0)
    assert output.state == 'FAILED'
    assert output.reason == 'error_limit'
    assert output.command.linear.x == 0.0


def test_stale_odometry_fails_closed():
    tracker = FixedPathTrackerCore(odom_timeout_sec=0.20)
    tracker.start(0.0, 0.0, 0.0, 0.0)
    output = tracker.update(-0.1, 0.0, 0.0, 1.0, odom_age_sec=0.21)
    assert output.state == 'FAILED'
    assert output.reason == 'odometry_timeout'
    assert output.command.linear.x == 0.0


def test_speed_and_steering_are_limited():
    tracker = FixedPathTrackerCore(max_reverse_speed=0.10,
                                   max_steer_omega=0.35,
                                   k_lateral=100.0, k_yaw=100.0)
    tracker.start(0.0, 0.0, 0.0, 0.0)
    output = tracker.update(-0.1, 0.1, math.radians(5.0), 1.0)
    assert abs(output.command.linear.x) <= 0.10
    assert abs(output.command.angular.z) <= 0.35


def test_speed_ramp_limits_acceleration():
    tracker = FixedPathTrackerCore(max_accel_mps2=0.10,
                                   target_speed=0.10)
    tracker.start(0.0, 0.0, 0.0, 0.0)
    first = tracker.update(-0.01, 0.0, 0.0, 0.1)
    second = tracker.update(-0.02, 0.0, 0.0, 0.2)
    assert abs(first.command.linear.x) <= 0.01 + 1e-9
    assert abs(second.command.linear.x - first.command.linear.x) <= 0.01 + 1e-9
