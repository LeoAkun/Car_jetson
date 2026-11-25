"""ROS message/pipeline regressions, called directly without spinning hardware.

Real nodes load the shipped ROS parameter files. Only TF lookup and outgoing
publisher sinks are substituted; CameraInfo, CvBridge and both algorithms run.
"""

import json
from pathlib import Path
import sys

import numpy as np
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ground.height_map_pothole_detector import HeightMapPotholeNode  # noqa: E402
from test_small_pit_depth_pipeline import (  # noqa: E402
    IMAGE_SHAPE, K, R, T, render_ground_with_pit,
)
from test_dropoff_detector import mounted_camera_scene  # noqa: E402


class CapturedPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    @property
    def latest(self):
        return self.messages[-1]


@pytest.fixture
def make_node(monkeypatch):
    # Keep discovery separate from any vehicle ROS graph. No executor is spun.
    monkeypatch.setenv("ROS_DOMAIN_ID", "231")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    created = []

    def construct(config="pothole_detector.yaml", rotation=R, translation=T,
                  keep_auto_roi=False):
        assert not rclpy.ok(), "Each test must own its ROS context."
        rclpy.init(args=[
            "--ros-args", "--params-file", str(PACKAGE_ROOT / "config" / config),
            "--log-level", "error",
        ])
        node = HeightMapPotholeNode()
        created.append(node)
        # Exercise the configured metric ROI without auto-ROI history changing
        # between sequential frames in these message-interface checks.
        if not keep_auto_roi:
            node.auto_roi_from_camera = False
        node.detector.rng = np.random.RandomState(0)
        monkeypatch.setattr(node, "get_tf_transform", lambda **kwargs: (rotation, translation))
        for name in vars(node).copy():
            if name.endswith("_pub"):
                monkeypatch.setattr(node, name, CapturedPublisher())
        return node

    yield construct

    for node in created:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def set_camera_info(node, intrinsics=K, shape=IMAGE_SHAPE, frame=None):
    info = CameraInfo()
    info.height, info.width = shape
    info.k = np.asarray(intrinsics, dtype=np.float64).ravel().tolist()
    info.header.frame_id = node.camera_frame if frame is None else frame
    info.header.stamp = Time(seconds=1.0).to_msg()
    node.camera_info_callback(info)


def send_depth(node, depth, *, index=0, frame=None, encoding="32FC1"):
    image = node.bridge.cv2_to_imgmsg(np.asarray(depth), encoding=encoding)
    image.header.frame_id = node.camera_frame if frame is None else frame
    image.header.stamp = Time(seconds=1.0 + index / 30.0).to_msg()
    node.depth_callback(image)
    return json.loads(node.status_pub.latest.data)


def cloud_fields(message):
    assert message.point_step == 4 * len(message.fields)
    assert message.row_step == message.point_step * message.width
    assert len(message.data) == message.row_step * message.height
    values = np.frombuffer(message.data, dtype="<f4").reshape((-1, len(message.fields)))
    return {field.name: values[:, index] for index, field in enumerate(message.fields)}


@pytest.mark.parametrize("encoding", ["32FC1", "16UC1"])
def test_deployment_node_emits_small_pit_cloud_and_status(make_node, encoding):
    node = make_node(keep_auto_roi=True)
    assert node.auto_roi_from_camera
    set_camera_info(node)
    depth, _, _ = render_ground_with_pit(center_x=1.5, width=0.20, pit_depth=0.05)
    if encoding == "16UC1":
        depth = np.rint(depth / 0.001).astype(np.uint16)

    status = send_depth(node, depth, encoding=encoding)

    assert status["state"] == "detected_pothole", status
    assert status["pit_count"] >= 1
    cloud = node.pothole_detections_pub.latest
    assert cloud.width > 0
    assert cloud.header.frame_id == "base_link"
    fields = cloud_fields(cloud)
    assert np.max(fields["max_drop"]) == pytest.approx(0.05, abs=0.01)
    assert np.all(np.abs(fields["x"] - 1.5) < 0.20)


def test_wide_near_edge_survives_pit_depth_quality_rejection(make_node):
    depth, intrinsics, rotation, translation = mounted_camera_scene(drop=0.0)
    v, u = np.indices(depth.shape)
    rays = np.stack(((u - intrinsics[0, 2]) / intrinsics[0, 0],
                     (v - intrinsics[1, 2]) / intrinsics[1, 1],
                     np.ones(depth.shape)), axis=-1) @ rotation.T
    expected_x = translation[0] + depth * rays[..., 0]
    depth[expected_x >= 1.20] = 0.0
    valid_ratio = np.mean(depth > 0.0)
    # Most rays see neither lower ground nor a return. Remaining near-side
    # support is still enough to fit the upper plane and locate its edge.
    assert 0.01 < valid_ratio < 0.10
    node = make_node(rotation=rotation, translation=translation, keep_auto_roi=True)
    set_camera_info(node, intrinsics)
    assert valid_ratio < node.detector.min_valid_depth_ratio

    for index in range(3):
        status = send_depth(node, depth, index=index)

    assert status["state"] == "suspected_dropoff", status
    assert status["pit_quality_reject"] == "depth_valid_ratio", status
    assert status["pit_count"] == 0
    assert node.pothole_detections_pub.latest.width == 0
    fields = cloud_fields(node.dropoff_pub.latest)
    assert len(fields["x"]) > 0
    assert np.allclose(fields["x"], 1.20, atol=0.025)
    assert np.all(fields["depth_known"] == 0.0)
    assert np.all(fields["drop"] == -1.0)


def test_unknown_lower_surface_confirms_upper_edge_without_pit_cloud(make_node):
    depth, intrinsics, rotation, translation = mounted_camera_scene(missing=True)
    node = make_node(rotation=rotation, translation=translation)
    set_camera_info(node, intrinsics)

    for index in range(2):
        status = send_depth(node, depth, index=index)
        assert node.dropoff_pub.latest.width == 0
        assert status["state"] == "observing", status
    status = send_depth(node, depth, index=2)

    assert status["state"] == "suspected_dropoff", status
    assert status["edge_count"] >= 1
    assert node.pothole_detections_pub.latest.width == 0
    fields = cloud_fields(node.dropoff_pub.latest)
    assert len(fields["x"]) > 0
    assert np.allclose(fields["x"], 1.65, atol=0.025)
    assert np.all(fields["depth_known"] == 0.0)
    assert np.all(fields["drop"] == -1.0)
    assert np.allclose(fields["z"], -0.67, atol=0.01)


def test_visible_lower_surface_outside_grid_height_roi_still_measures_drop(make_node):
    depth, intrinsics, rotation, translation = mounted_camera_scene(drop=0.65)
    node = make_node(rotation=rotation, translation=translation)
    set_camera_info(node, intrinsics)
    assert -0.67 - 0.65 < node.detector.z_min

    for index in range(3):
        status = send_depth(node, depth, index=index)

    assert status["state"] == "suspected_dropoff", status
    fields = cloud_fields(node.dropoff_pub.latest)
    assert len(fields["x"]) > 0
    assert np.all(fields["depth_known"] == 1.0)
    assert np.allclose(fields["drop"], 0.65, atol=0.01)
    assert np.allclose(fields["z"], -0.67, atol=0.01)


def test_full_depth_loss_clears_clouds_and_restarts_confirmation(make_node):
    depth, intrinsics, rotation, translation = mounted_camera_scene(missing=True)
    node = make_node(rotation=rotation, translation=translation)
    set_camera_info(node, intrinsics)
    for index in range(3):
        send_depth(node, depth, index=index)
    assert node.dropoff_pub.latest.width > 0

    status = send_depth(node, np.zeros_like(depth), index=3)

    assert status["state"] == "unavailable", status
    assert node.dropoff_pub.latest.width == 0
    assert node.pothole_detections_pub.latest.width == 0
    assert node.detector.last_ground_plane is None
    for index in range(4, 6):
        status = send_depth(node, depth, index=index)
        assert node.dropoff_pub.latest.width == 0
        assert status["state"] == "observing", status
    assert send_depth(node, depth, index=6)["state"] == "suspected_dropoff"


@pytest.mark.parametrize("mismatch, reason", [
    ("shape", "depth_camera_info_shape_mismatch"),
    ("frame", "depth_camera_info_frame_mismatch"),
])
def test_depth_camera_info_mismatch_is_unavailable_and_clears_previous_pit(
        make_node, mismatch, reason):
    node = make_node()
    set_camera_info(node)
    depth, _, _ = render_ground_with_pit(width=0.20, pit_depth=0.05)
    send_depth(node, depth)
    assert node.pothole_detections_pub.latest.width > 0

    if mismatch == "shape":
        status = send_depth(node, depth[:-1], index=1)
    else:
        status = send_depth(node, depth, index=1, frame="different_depth_optical_frame")

    assert status["state"] == "unavailable", status
    assert status["reason"] == reason
    assert node.pothole_detections_pub.latest.width == 0
    assert node.dropoff_pub.latest.width == 0
    assert node.detector.last_ground_plane is None


def test_depth_timeout_clears_confirmed_hazard_and_publishes_unavailable(make_node):
    depth, intrinsics, rotation, translation = mounted_camera_scene(missing=True)
    node = make_node(rotation=rotation, translation=translation)
    set_camera_info(node, intrinsics)
    for index in range(3):
        send_depth(node, depth, index=index)
    assert node.dropoff_pub.latest.width > 0

    node.last_depth_received = node.get_clock().now() - Duration(seconds=node.depth_timeout + 0.1)
    node.check_depth_timeout()

    status = json.loads(node.status_pub.latest.data)
    assert status["state"] == "unavailable", status
    assert status["reason"] == "depth_timeout"
    assert node.dropoff_pub.latest.width == 0
    assert node.pothole_detections_pub.latest.width == 0
    assert node.detector.last_ground_plane is None


def test_indoor_config_uses_raised_model_plane_and_five_centimeter_pit(make_node):
    node = make_node("pothole_detector_indoor.yaml")
    assert node.detector.ground_reference_z == pytest.approx(-0.52)
    assert node.roi_ground_z == pytest.approx(-0.52)
    assert node.detector.x_min == pytest.approx(0.90)
    assert node.detector.x_max == pytest.approx(2.0)
    assert node.publish_debug_images_enabled
    set_camera_info(node)
    depth, _, _ = render_ground_with_pit(
        ground_z=-0.52, center_x=1.4, width=0.20, pit_depth=0.05)

    status = send_depth(node, depth)

    assert status["state"] == "detected_pothole", status
    assert node.pothole_detections_pub.latest.width > 0
    assert node.detector.last_ground_plane[2] == pytest.approx(-0.52, abs=0.005)
    assert node.pit_bottom_mask_image_pub.latest.encoding == "mono8"
    fields = cloud_fields(node.pothole_detections_pub.latest)
    assert np.max(fields["max_drop"]) == pytest.approx(0.05, abs=0.01)
