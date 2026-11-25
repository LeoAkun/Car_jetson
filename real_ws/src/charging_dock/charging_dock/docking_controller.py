# Copyright 2026 akun
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""
Static Stage 4 lidar/controller integration.

This node is deliberately dry-run only: it evaluates docking geometry and
safety conditions, publishes diagnostics, and always sends a zero Twist.
"""

from dataclasses import dataclass
import math

from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String

from charging_dock.rear_lidar_processor import (
    LidarObservation,
    RearLidarProcessor,
)


@dataclass
class StaticDockResult:
    state: str
    reason: str
    in_stop_window: bool
    obstacle: bool
    observation: LidarObservation


class DockingStaticCore:
    """Fail-closed state evaluation independent of ROS."""

    def __init__(self, processor, stop_min_m=0.45, stop_max_m=0.47,
                 expected_angle_deg=0.0, angle_tolerance_deg=5.0,
                 minimum_quality=0.0, obstacle_range_m=1.0,
                 target_roi_min_deg=-30.0, target_roi_max_deg=30.0,
                 target_intrusion_margin_m=0.15):
        self.processor = processor
        self.stop_min_m = float(stop_min_m)
        self.stop_max_m = float(stop_max_m)
        self.expected_angle_deg = float(expected_angle_deg)
        self.angle_tolerance_deg = abs(float(angle_tolerance_deg))
        self.minimum_quality = float(minimum_quality)
        self.obstacle_range_m = float(obstacle_range_m)
        self.target_roi_min_deg = float(target_roi_min_deg)
        self.target_roi_max_deg = float(target_roi_max_deg)
        self.target_intrusion_margin_m = abs(
            float(target_intrusion_margin_m))

    def evaluate(self, message, storage_stamp_ns):
        observation = self.processor.process(message, storage_stamp_ns)
        obstacle = self._has_non_target_obstacle(message, observation)
        if obstacle:
            return StaticDockResult(
                'DRY_RUN_OBSTACLE', 'non_target_obstacle', False, True,
                observation)
        if not observation.valid:
            state = {
                'stale_timestamp': 'DRY_RUN_STALE_SCAN',
                'unexpected_frame': 'DRY_RUN_WRONG_FRAME',
                'target_jump': 'DRY_RUN_TARGET_JUMP',
                'warming_up': 'DRY_RUN_WARMING_UP',
            }.get(observation.reason, 'DRY_RUN_NO_TARGET')
            return StaticDockResult(
                state, observation.reason, False, False, observation)
        if observation.quality < self.minimum_quality:
            return StaticDockResult(
                'DRY_RUN_LOW_QUALITY', 'low_quality', False, False,
                observation)
        yaw = observation.base_link_yaw_deg
        angle_error = None if yaw is None else (
            (yaw - self.expected_angle_deg + 90.0) % 180.0 - 90.0)
        if angle_error is None or abs(angle_error) > self.angle_tolerance_deg:
            return StaticDockResult(
                'DRY_RUN_ANGLE_OUT_OF_RANGE', 'angle_out_of_range', False,
                False, observation)
        distance = observation.base_link_plane_normal_distance_m
        in_window = (
            distance is not None
            and self.stop_min_m <= distance <= self.stop_max_m)
        return StaticDockResult(
            'DRY_RUN_STOP_WINDOW' if in_window else 'DRY_RUN_TARGET_VALID',
            'stop_window' if in_window else 'target_valid', in_window, False,
            observation)

    def _has_non_target_obstacle(self, message, observation):
        """Detect close returns outside the target or in front of its plane."""
        if not message.ranges:
            return False
        target_range = observation.distance_m
        angle = float(message.angle_min)
        for value in message.ranges:
            value = float(value)
            degrees = math.degrees(angle)
            if (self.target_roi_min_deg <= degrees <= self.target_roi_max_deg):
                if (math.isfinite(value) and value > 0.0
                        and target_range is not None
                        and value < target_range
                        - self.target_intrusion_margin_m):
                    return True
                angle += float(message.angle_increment)
                continue
            if (math.isfinite(value) and value > 0.0
                    and value < self.obstacle_range_m):
                return True
            angle += float(message.angle_increment)
        return False


class DockingController(Node):
    """ROS adapter for static lidar integration; never commands motion."""

    def __init__(self):
        super().__init__('docking_controller')
        self.declare_parameter('scan_topic', '/laser/scan_back')
        self.declare_parameter('state_topic', '/charging_dock/state')
        self.declare_parameter('pose_topic', '/charging_dock/lidar_pose')
        self.declare_parameter('quality_topic', '/charging_dock/lidar_quality')
        self.declare_parameter('command_topic', '/cmd_vel_dock')
        self.declare_parameter('roi_min_deg', -30.0)
        self.declare_parameter('roi_max_deg', 30.0)
        self.declare_parameter('expected_range_m', 2.5)
        self.declare_parameter('stop_distance_min_m', 0.45)
        self.declare_parameter('stop_distance_max_m', 0.47)
        self.declare_parameter('expected_angle_deg', 0.0)
        self.declare_parameter('angle_tolerance_deg', 5.0)
        self.declare_parameter('minimum_quality', 0.0)
        self.declare_parameter('obstacle_range_m', 1.0)
        self.declare_parameter('target_intrusion_margin_m', 0.15)
        self.declare_parameter('sensor_x_m', -0.43)
        self.declare_parameter('sensor_y_m', -0.28)
        self.declare_parameter('sensor_yaw_deg', -135.0)
        self.declare_parameter('maximum_message_age_sec', 0.20)
        self.declare_parameter('stable_frames', 10)
        p = self.get_parameter
        self.processor = RearLidarProcessor(
            p('roi_min_deg').value, p('roi_max_deg').value,
            expected_range_m=p('expected_range_m').value,
            stable_frames=p('stable_frames').value,
            maximum_age_sec=p('maximum_message_age_sec').value,
            expected_frames=('rear_laser',), sensor_x_m=p('sensor_x_m').value,
            sensor_y_m=p('sensor_y_m').value,
            sensor_yaw_deg=p('sensor_yaw_deg').value)
        self.core = DockingStaticCore(
            self.processor, p('stop_distance_min_m').value,
            p('stop_distance_max_m').value, p('expected_angle_deg').value,
            p('angle_tolerance_deg').value, p('minimum_quality').value,
            p('obstacle_range_m').value, p('roi_min_deg').value,
            p('roi_max_deg').value,
            p('target_intrusion_margin_m').value)
        self.state_pub = self.create_publisher(
            String, p('state_topic').value, 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, p('pose_topic').value, 10)
        self.quality_pub = self.create_publisher(
            Float32MultiArray, p('quality_topic').value, 10)
        self.command_pub = self.create_publisher(
            Twist, p('command_topic').value, 10)
        self.create_subscription(
            LaserScan, p('scan_topic').value, self._scan_callback, 10)
        self.create_timer(0.05, self._zero_command)

    def _zero_command(self):
        self.command_pub.publish(Twist())

    def _scan_callback(self, message):
        now_ns = self.get_clock().now().nanoseconds
        result = self.core.evaluate(message, now_ns)
        state = String()
        state.data = (
            f'{result.state} reason={result.reason} '
            f'in_stop_window={result.in_stop_window}')
        self.state_pub.publish(state)
        quality = Float32MultiArray()
        obs = result.observation
        quality.data = [float(obs.quality), float(obs.target_points),
                        float(obs.fit_residual_m or 0.0),
                        float(obs.base_link_plane_normal_distance_m or 0.0),
                        float(obs.base_link_yaw_deg or 0.0),
                        1.0 if obs.center_available else 0.0]
        self.quality_pub.publish(quality)
        if not obs.valid:
            return
        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(
            obs.base_link_plane_normal_distance_m or 0.0)
        pose.pose.position.y = float(obs.lateral_center_m or 0.0)
        yaw = math.radians(float(obs.base_link_yaw_deg or 0.0))
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        self.pose_pub.publish(pose)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
