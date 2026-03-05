#!/usr/bin/env python3
"""
临时脚本：过滤 /laser/scan_merge_laser 话题中距离传感器1cm以内的点
合并后的scan是在laser_link坐标系下，两个传感器有偏移，
所以需要计算每个点到传感器的实际距离来过滤。

用法: python3 laser_filter.py
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

# 过滤阈值：距离传感器小于此值的点会被删除
MIN_DIST_TO_SENSOR = 0.05  # 1cm

# 两个激光雷达在 laser_link 坐标系下的安装位置 (来自 params.yaml)
SENSOR_POSITIONS = [
    (0.44, 0.28),    # laser1 (front)
    (-0.44, -0.28),  # laser2 (back)
]

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class LaserFilter(Node):
    def __init__(self):
        super().__init__('laser_filter')
        self.sub = self.create_subscription(
            LaserScan, '/laser/scan_merge_laser', self.callback, SENSOR_QOS)
        self.pub = self.create_publisher(
            LaserScan, '/laser/scan_merge_laser_filtered', SENSOR_QOS)
        self.get_logger().info(
            f'激光过滤节点启动，过滤距离传感器 < {MIN_DIST_TO_SENSOR}m 的点')

    def callback(self, msg: LaserScan):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        new_ranges = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and r > 0:
                # 计算该点在 laser_link 坐标系下的 x, y
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                # 检查到每个传感器的距离
                too_close = False
                for sx, sy in SENSOR_POSITIONS:
                    dist = math.hypot(x - sx, y - sy)
                    if dist < MIN_DIST_TO_SENSOR:
                        too_close = True
                        break
                new_ranges.append(float('inf') if too_close else r)
            else:
                new_ranges.append(r)
            angle += msg.angle_increment

        filtered.ranges = new_ranges
        filtered.intensities = list(msg.intensities)
        self.pub.publish(filtered)


def main():
    rclpy.init()
    node = LaserFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
