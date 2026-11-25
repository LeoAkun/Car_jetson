"""End-to-end small-pit checks using the deployed detector configuration.

The scene is ray cast in the fixed camera pose: rays first meet the upper
ground, the bottom of the opening, or an opaque vertical pit wall.  These
tests exercise depth sampling, ground fitting, gridding and component
filtering together; they are not a substitute for real camera acceptance.
"""

import inspect
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ground.height_map_pothole_detector import HeightMapPotholeDetector  # noqa: E402


IMAGE_SHAPE = (480, 848)
K = np.array([[440.0, 0.0, 423.5],
              [0.0, 440.0, 239.5],
              [0.0, 0.0, 1.0]], dtype=np.float64)
PITCH = math.radians(15.0)
CAMERA_LINK_TO_BASE = np.array([
    [math.cos(PITCH), 0.0, math.sin(PITCH)],
    [0.0, 1.0, 0.0],
    [-math.sin(PITCH), 0.0, math.cos(PITCH)],
])
OPTICAL_TO_CAMERA_LINK = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])
R = CAMERA_LINK_TO_BASE @ OPTICAL_TO_CAMERA_LINK
T = np.array([0.470, -0.010, 0.005], dtype=np.float64)
GROUND_Z = -0.670


@pytest.fixture
def deployment_parameters():
    with (PACKAGE_ROOT / "config" / "pothole_detector.yaml").open() as stream:
        params = yaml.safe_load(stream)["height_map_pothole_detector"]["ros__parameters"]

    # Read the actual deployment file rather than silently testing a separate
    # hand-tuned configuration which the ROS node will never use.
    assert params["resolution"] <= 0.025
    assert params["step"] <= 2
    assert params["min_cell_points"] <= 2
    assert params["low_threshold"] <= 0.04
    assert params["ground_reference_enabled"] is True
    assert params["ground_reference_z"] == pytest.approx(GROUND_Z)
    return params


def make_deployed_detector(params):
    accepted = inspect.signature(HeightMapPotholeDetector.__init__).parameters
    kwargs = {name: value for name, value in params.items() if name in accepted}
    kwargs["random_seed"] = 0
    return HeightMapPotholeDetector(**kwargs)


def render_ground_with_pit(*, center_x=1.5, center_y=0.0, width=0.2,
                           pit_depth=0.05, ground_z=GROUND_Z,
                           noise_std=0.0, seed=0):
    """Return optical-axis depth and masks of physically visible pit faces.

    An opening is square in the base frame. A ray which passes through its
    top must hit either its bottom or the first side wall; distant floor
    cannot be seen through an opaque wall. Non-ground-facing or out-of-range
    rays have no return, as in a depth image.
    """
    v, u = np.indices(IMAGE_SHAPE, dtype=np.float64)
    camera_rays = np.stack(((u - K[0, 2]) / K[0, 0],
                            (v - K[1, 2]) / K[1, 1],
                            np.ones(IMAGE_SHAPE)), axis=-1)
    directions = camera_rays @ R.T
    downward = directions[..., 2] < -1e-9
    ray_parameter = np.full(IMAGE_SHAPE, np.inf, dtype=np.float64)
    ray_parameter[downward] = ((ground_z - T[2]) /
                               directions[..., 2][downward])
    top_x = T[0] + ray_parameter * directions[..., 0]
    top_y = T[1] + ray_parameter * directions[..., 1]
    bottom_seen = np.zeros(IMAGE_SHAPE, dtype=bool)
    wall_seen = np.zeros(IMAGE_SHAPE, dtype=bool)

    if pit_depth > 0.0:
        x_min, x_max = center_x - width / 2.0, center_x + width / 2.0
        y_min, y_max = center_y - width / 2.0, center_y + width / 2.0
        opening = (downward & (top_x > x_min) & (top_x < x_max) &
                   (top_y > y_min) & (top_y < y_max))
        bottom_parameter = np.full(IMAGE_SHAPE, np.inf, dtype=np.float64)
        bottom_parameter[downward] = ((ground_z - pit_depth - T[2]) /
                                      directions[..., 2][downward])
        # The first exit from the rectangular void is the visible opaque
        # side wall. Its parameter is strictly beyond the top intersection.
        exit_parameter = np.full(IMAGE_SHAPE, np.inf, dtype=np.float64)
        for axis, lower, upper in ((0, x_min, x_max), (1, y_min, y_max)):
            component = directions[..., axis]
            moving = np.abs(component) > 1e-9
            boundary = np.where(component >= 0.0, upper, lower)
            candidate = np.full(IMAGE_SHAPE, np.inf, dtype=np.float64)
            candidate[moving] = ((boundary[moving] - T[axis]) /
                                 component[moving])
            exit_parameter = np.minimum(exit_parameter, candidate)

        bottom_seen = opening & (bottom_parameter <= exit_parameter)
        wall_seen = opening & ~bottom_seen
        ray_parameter[opening] = np.minimum(bottom_parameter,
                                            exit_parameter)[opening]

    valid = np.isfinite(ray_parameter) & (ray_parameter > 0.1) & (ray_parameter < 5.0)
    depth = np.zeros(IMAGE_SHAPE, dtype=np.float32)
    depth[valid] = ray_parameter[valid]
    if noise_std > 0.0:
        rng = np.random.RandomState(seed)
        depth[valid] += rng.normal(0.0, noise_std, np.count_nonzero(valid))
    return depth, bottom_seen & valid, wall_seen & valid


@pytest.mark.parametrize("width, center_x", [
    (0.20, 1.30),
    (0.20, 1.50),
    (0.30, 1.30),
    (0.30, 1.50),
    (0.20, 1.5125),  # A half-cell translation must not hide the same pit.
])
def test_five_centimeter_pit_is_detected_from_rendered_depth(
        deployment_parameters, width, center_x):
    detector = make_deployed_detector(deployment_parameters)
    depth, bottom_seen, wall_seen = render_ground_with_pit(
        width=width, center_x=center_x, pit_depth=0.05,
        noise_std=0.001, seed=7)
    assert np.count_nonzero(bottom_seen) > 100
    assert np.count_nonzero(wall_seen) > 0

    detections, *_ = detector.process(depth, K, R, T)

    nearby = [pit for pit in detections
              if abs(pit["x"] - center_x) < width / 2.0 + 0.025
              and abs(pit["y"]) < width / 2.0]
    assert nearby, detector.last_debug
    assert max(pit["max_drop"] for pit in nearby) >= 0.04
    assert max(pit["max_drop"] for pit in nearby) < 0.07
    assert all(pit["z"] == pytest.approx(GROUND_Z, abs=0.015) for pit in nearby)


def test_flat_ground_with_millimeter_depth_noise_has_no_pit(deployment_parameters):
    detector = make_deployed_detector(deployment_parameters)
    depth, _, _ = render_ground_with_pit(pit_depth=0.0, noise_std=0.003, seed=10)

    detections, *_ = detector.process(depth, K, R, T)

    assert detections == []
    assert not detector.last_debug.get("quality_reject_reason"), detector.last_debug
    assert detector.last_ground_plane is not None


def test_two_centimeter_recess_does_not_cross_danger_threshold(deployment_parameters):
    detector = make_deployed_detector(deployment_parameters)
    depth, bottom_seen, _ = render_ground_with_pit(
        width=0.30, pit_depth=0.02, noise_std=0.001, seed=3)
    assert np.count_nonzero(bottom_seen) > 100

    detections, *_ = detector.process(depth, K, R, T)

    assert detections == []
    assert not detector.last_debug.get("quality_reject_reason"), detector.last_debug


def test_lower_floor_cannot_replace_missing_upper_ground_reference(deployment_parameters):
    detector = make_deployed_detector(deployment_parameters)
    upper_depth, _, _ = render_ground_with_pit(pit_depth=0.0)
    detector.process(upper_depth, K, R, T)
    assert detector.last_ground_plane is not None
    # No upper surface remains in view. This is an unavailable reference,
    # not evidence that the lowered floor is a safe continuation. A plane
    # from the preceding healthy frame must not survive the rejection.
    depth, _, _ = render_ground_with_pit(pit_depth=0.0, ground_z=GROUND_Z - 0.05)

    detections, *_ = detector.process(depth, K, R, T)

    assert detections == []
    assert detector.last_debug["valid_cell_count"] >= deployment_parameters["min_valid_cells"]
    assert detector.last_ground_plane is None
    assert "ground_reference" in detector.last_debug.get("quality_reject_reason", "")
