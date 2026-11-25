"""Ray-rendered regressions for upper-edge detection without ROS or hardware."""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from ground.dropoff_detector import GroundRayDropoffDetector  # noqa: E402


K = np.array([[140.0, 0.0, 119.5], [0.0, 140.0, 30.0], [0.0, 0.0, 1.0]])
R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T = np.array([0.0, 0.0, 0.67])
PLANE = (0.0, 0.0, 0.0)


def scene(edge_x=None, drop=0.1, missing=False, plane=PLANE):
    """Intersect camera rays with an upper plane and optional lower half-plane."""
    us, vs = np.meshgrid(np.arange(240), np.arange(160))
    camera_rays = np.stack(((us - K[0, 2]) / K[0, 0],
                            (vs - K[1, 2]) / K[1, 1], np.ones_like(us)), axis=-1)
    rays = camera_rays @ R.T
    a, b, c = plane
    denominator = rays[:, :, 2] - a * rays[:, :, 0] - b * rays[:, :, 1]
    depth = np.zeros(us.shape, dtype=np.float64)
    downward = denominator < -1e-4
    depth[downward] = (c + a * T[0] + b * T[1] - T[2]) / denominator[downward]
    if edge_x is not None:
        lower = downward & (depth * rays[:, :, 0] + T[0] >= edge_x)
        if missing:
            depth[lower] = 0.0
        else:
            depth[lower] = (c - drop + a * T[0] + b * T[1] - T[2]) / denominator[lower]
    depth[(depth > 5.0) | ~np.isfinite(depth)] = 0.0
    return depth.astype(np.float32)


def confirm(detector, depth, plane=PLANE, start=1.0):
    results = []
    for frame in range(detector.confirmation_frames):
        results = detector.process(depth, K, R, T, plane, start + frame / 30.0)
    return results


def test_flat_plane_and_inclined_ground_have_no_dropoff():
    detector = GroundRayDropoffDetector()
    assert confirm(detector, scene()) == []
    assert detector.last_status == "observing"
    plane = (0.08, 0.03, 0.0)
    assert confirm(detector, scene(plane=plane), plane=plane, start=2.0) == []


def test_visible_five_centimeter_drop_uses_upper_edge_not_lower_return():
    detector = GroundRayDropoffDetector()
    detections = confirm(detector, scene(edge_x=1.25, drop=0.05))
    assert len(detections) == 1
    detection = detections[0]
    assert detection["x"] == pytest.approx(1.25, abs=0.025)
    assert detection["drop"] == pytest.approx(0.05, abs=1e-5)
    assert detection["depth_known"] is True
    assert detection["width"] > 0.12
    assert detector.last_status == "suspected_dropoff"


def test_missing_lower_surface_is_only_unknown_suspicion():
    detector = GroundRayDropoffDetector()
    detections = confirm(detector, scene(edge_x=1.25, missing=True))
    assert len(detections) == 1
    assert detections[0]["x"] == pytest.approx(1.25, abs=0.025)
    assert detections[0]["depth_known"] is False
    assert detections[0]["drop"] is None
    assert detector.last_debug["unknown_edges"] == 1


def test_positive_obstacle_and_its_neighboring_invalid_shadow_are_not_dropoff():
    depth = scene()
    depth[65:105, 70:170] = 0.65  # A closer, positive obstacle.
    depth[65:105, 170:172] = 0.0  # Narrow adjacent missing-depth shadow.
    detector = GroundRayDropoffDetector()
    assert confirm(detector, depth) == []


def test_full_depth_loss_and_missing_plane_are_unavailable_not_clear():
    detector = GroundRayDropoffDetector()
    assert confirm(detector, np.zeros((160, 240), dtype=np.float32)) == []
    assert detector.last_status == "unavailable"
    assert detector.last_debug["reason"] == "insufficient_depth"
    assert detector.process(scene(), K, R, T, None) == []
    assert detector.last_status == "unavailable"


def test_random_missing_pixels_and_one_pixel_strip_are_not_edges():
    depth = scene()
    rng = np.random.RandomState(17)
    depth[rng.uniform(size=depth.shape) < 0.005] = 0.0
    depth[105, 50:190] = 0.0
    detector = GroundRayDropoffDetector()
    assert confirm(detector, depth) == []


def test_transition_at_image_border_has_no_near_support():
    # Last safe ground row lies too close to the bottom of the image.
    detector = GroundRayDropoffDetector()
    assert confirm(detector, scene(edge_x=0.75, missing=True)) == []


def test_confirmation_requires_consecutive_frames_and_resets_after_time_gap():
    depth = scene(edge_x=1.25, missing=True)
    detector = GroundRayDropoffDetector()
    assert detector.process(depth, K, R, T, PLANE, 1.0) == []
    assert detector.process(depth, K, R, T, PLANE, 1.03) == []
    assert detector.process(depth, K, R, T, PLANE, 2.0) == []
    assert detector.process(depth, K, R, T, PLANE, 2.03) == []
    assert len(detector.process(depth, K, R, T, PLANE, 2.06)) == 1
    assert detector.process(scene(), K, R, T, PLANE, 2.09) == []
    assert detector.process(depth, K, R, T, PLANE, 2.12) == []
    assert detector.process(depth, K, R, T, PLANE, 2.15) == []
    assert len(detector.process(depth, K, R, T, PLANE, 2.18)) == 1
    assert detector.process(depth, K, R, T, PLANE, 2.17) == []


def test_reset_discards_previous_confirmation():
    detector = GroundRayDropoffDetector()
    depth = scene(edge_x=1.25, missing=True)
    assert len(confirm(detector, depth)) == 1
    detector.reset()
    assert detector.last_status == "unavailable"
    assert detector.process(depth, K, R, T, PLANE, 2.0) == []


def test_unknown_and_measured_evidence_do_not_share_confirmation():
    detector = GroundRayDropoffDetector()
    missing = scene(edge_x=1.25, missing=True)
    measured = scene(edge_x=1.25)
    assert detector.process(missing, K, R, T, PLANE, 1.0) == []
    assert detector.process(missing, K, R, T, PLANE, 1.03) == []
    assert detector.process(measured, K, R, T, PLANE, 1.06) == []
    assert detector.process(measured, K, R, T, PLANE, 1.09) == []
    assert len(detector.process(measured, K, R, T, PLANE, 1.12)) == 1


def mounted_camera_scene(drop=0.05, missing=False, edge_width=0.16):
    """Render with the deployed pose, retaining data outside the height-map ROI."""
    intrinsics = np.array([[425., 0., 423.5], [0., 425., 239.5], [0., 0., 1.]])
    pitch = np.deg2rad(15.)
    rotation = np.array([
        [np.cos(pitch), 0., np.sin(pitch)], [0., 1., 0.],
        [-np.sin(pitch), 0., np.cos(pitch)],
    ]) @ R
    translation = np.array([0.47, -0.01, 0.005])
    us, vs = np.meshgrid(np.arange(848), np.arange(480))
    rays = np.stack(((us - intrinsics[0, 2]) / intrinsics[0, 0],
                     (vs - intrinsics[1, 2]) / intrinsics[1, 1],
                     np.ones_like(us)), axis=-1) @ rotation.T
    depth = np.zeros(us.shape, dtype=np.float64)
    downward = rays[:, :, 2] < -1e-4
    depth[downward] = (-0.67 - translation[2]) / rays[:, :, 2][downward]
    expected_x = translation[0] + depth * rays[:, :, 0]
    expected_y = translation[1] + depth * rays[:, :, 1]
    lower = downward & (expected_x >= 1.65) & (np.abs(expected_y + 0.01) <= edge_width / 2)
    if missing:
        depth[lower] = 0.0
    else:
        depth[lower] = (-0.67 - drop - translation[2]) / rays[:, :, 2][lower]
    depth[(depth > 5.) | ~np.isfinite(depth)] = 0.0
    return depth.astype(np.float32), intrinsics, rotation, translation


@pytest.mark.parametrize("drop", [0.05, 0.10, 0.65])
def test_deployed_mount_detects_short_drop_and_lower_surface_outside_height_roi(drop):
    # A 65 cm lower surface is z=-1.32, outside the existing z_min=-1.20.
    # The upper-edge branch must use the original optical image, not ROI points.
    depth, intrinsics, rotation, translation = mounted_camera_scene(drop=drop)
    detector = GroundRayDropoffDetector()
    output = []
    for frame in range(3):
        output = detector.process(depth, intrinsics, rotation, translation,
                                  (0., 0., -0.67), 1. + frame / 30.)
    assert len(output) == 1
    assert output[0]["x"] == pytest.approx(1.65, abs=0.025)
    assert output[0]["width"] == pytest.approx(0.16, abs=0.02)
    assert output[0]["depth_known"] is True
    assert output[0]["drop"] == pytest.approx(drop, abs=1e-5)


def test_finite_return_beyond_sensor_range_is_unknown_not_measured_low_surface():
    depth, intrinsics, rotation, translation = mounted_camera_scene(missing=True)
    # Real drivers may preserve a finite out-of-range value rather than zero.
    # It has no metric-depth authority and must not produce a fabricated drop.
    depth[depth == 0.0] = 12.0
    detector = GroundRayDropoffDetector()
    for frame in range(3):
        output = detector.process(depth, intrinsics, rotation, translation,
                                  (0., 0., -0.67), 1. + frame / 30.)
    assert len(output) == 1
    assert output[0]["depth_known"] is False
    assert output[0]["drop"] is None


def test_deployed_mount_rejects_discontinuity_narrower_than_twelve_centimeters():
    depth, intrinsics, rotation, translation = mounted_camera_scene(edge_width=0.08)
    detector = GroundRayDropoffDetector()
    for frame in range(3):
        output = detector.process(depth, intrinsics, rotation, translation,
                                  (0., 0., -0.67), 1. + frame / 30.)
    assert output == []


def test_deployed_mount_accepts_twelve_centimeter_edge_with_pixel_quantization():
    depth, intrinsics, rotation, translation = mounted_camera_scene(edge_width=0.12)
    detector = GroundRayDropoffDetector()
    for frame in range(3):
        output = detector.process(depth, intrinsics, rotation, translation,
                                  (0., 0., -0.67), 1. + frame / 30.)
    assert len(output) == 1
    assert output[0]["width"] == pytest.approx(0.12, abs=0.006)
