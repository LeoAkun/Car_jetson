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

from apriltag_msgs.msg import AprilTagDetection
from apriltag_msgs.msg import Point

from charging_dock.dock_tag_adapter import DockTagAdapter


def make_detection():
    detection = AprilTagDetection()
    detection.family = '36h11'
    detection.id = 0
    detection.hamming = 0
    detection.goodness = 1.0
    detection.decision_margin = 80.0
    detection.centre = Point(x=640.0, y=360.0)
    detection.corners = [
        Point(x=600.0, y=400.0),
        Point(x=680.0, y=400.0),
        Point(x=680.0, y=320.0),
        Point(x=600.0, y=320.0),
    ]
    detection.homography = [1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    return detection


def make_adapter_validator():
    adapter = object.__new__(DockTagAdapter)
    adapter._family = '36h11'
    adapter._max_hamming = 0
    adapter._min_margin = 20.0
    return adapter


def test_valid_detection_is_accepted():
    assert make_adapter_validator()._invalid_detection_reason(make_detection()) is None


def test_wrong_family_is_rejected():
    detection = make_detection()
    detection.family = '25h9'

    assert make_adapter_validator()._invalid_detection_reason(detection) == 'wrong_family'


def test_hamming_correction_is_rejected():
    detection = make_detection()
    detection.hamming = 1

    assert (
        make_adapter_validator()._invalid_detection_reason(detection)
        == 'hamming_rejected'
    )


def test_low_decision_margin_is_rejected():
    detection = make_detection()
    detection.decision_margin = 19.9

    assert make_adapter_validator()._invalid_detection_reason(detection) == 'low_margin'


def test_non_finite_corner_is_rejected():
    detection = make_detection()
    detection.corners[0].x = float('nan')

    assert (
        make_adapter_validator()._invalid_detection_reason(detection)
        == 'invalid_corners'
    )
