#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
map → odom 周期性变换发布节点
用途：持续发布 map 到 odom 的变换，可通过参数动态调整
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class MapToOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('map_to_odom_broadcaster')

        # 创建动态变换广播器（非静态）
        self.tf_broadcaster = TransformBroadcaster(self)

        # 可调参数
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw_deg', 0.0)          # 偏航角，单位：度
        self.declare_parameter('publish_rate', 10.0)    # 发布频率 Hz，默认 10Hz

        # 获取初始值
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value
        self.yaw_rad = math.radians(self.get_parameter('yaw_deg').value)
        self.rate = self.get_parameter('publish_rate').value

        # 创建定时器，周期发布变换
        self.timer_period = 1.0 / self.rate
        self.timer = self.create_timer(self.timer_period, self.broadcast_transform)

        self.get_logger().info(
            f"开始周期发布 map → odom 变换 "
            f"({self.x:.3f}, {self.y:.3f}, {self.z:.3f}), yaw={self.get_parameter('yaw_deg').value:.1f}° "
            f"@ {self.rate} Hz"
        )

    def broadcast_transform(self):
        """
        每周期发布一次 map → odom 变换
        """
        t = TransformStamped()

        # 使用当前时间戳（动态变换需要新鲜时间戳）
        t.header.stamp = self.get_clock().now().to_msg()

        # 父 → 子
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        # 平移
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        # 旋转（只绕 Z 轴）
        cy = math.cos(self.yaw_rad * 0.5)
        sy = math.sin(self.yaw_rad * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy

        # 广播变换
        self.tf_broadcaster.sendTransform(t)

        # 可选：每隔一段时间打印一次日志，避免刷屏
        if int(self.get_clock().now().nanoseconds / 1e9) % 10 == 0:
            self.get_logger().debug(f"广播 map→odom: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw_rad):.1f}°")


def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"节点异常退出: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()