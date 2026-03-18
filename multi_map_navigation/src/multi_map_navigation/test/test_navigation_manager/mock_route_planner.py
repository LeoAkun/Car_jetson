#!/usr/bin/env python3
"""
模拟/test/route_planner/pub_new_path服务
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from multi_map_navigation_msgs.msg import Waypoint, WaypointList, StartEndGraph
from multi_map_navigation_msgs.srv import PubNewPath
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
import time
import threading

class MockRoutePlannerNode(Node):
    def __init__(self):
        super().__init__('route_planner')

        self.get_logger().info("[MOCK] Mock Route Planner 已启动")

        # 服务（模拟 pub_new_path）
        self.pub_new_path_srv = self.create_service(
            PubNewPath,
            '/test/route_planner/pub_new_path',
            self.mock_pub_new_path_callback
        )
        # 启动后自动模拟一次路径发布（可选）
        self.publisher_ = self.create_publisher(WaypointList, '/test/waypoint_list', 10)

    def mock_pub_new_path_callback(self, request, response):
        """模拟 pub_new_path 服务调用"""
        self.get_logger().info("[MOCK] 收到 pub_new_path 请求")
        self.get_logger().info(f"[MOCK] 已走过的点数: {len(request.points)}")

        # 发布新路径
        # 填充waypoint_list路径消息
        waypoint_list_msg = WaypointList()
        waypoint_list_msg.header = Header()
        waypoint_list_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_list_msg.header.frame_id = 'map'

        # 生成任务ID
        waypoint_list_msg.task_id = f'task_{self.get_clock().now().to_msg().sec}'
        waypoint_list_msg.path = "path1" # 路线名称
        
        # 填充waypoint航点消息
        A = self.create_waypoint(name = "A", id_int = 0, x = 0.0, y = 0.0, map_name="map1", w_type=1)
        B1 = self.create_waypoint(name = "B1", id_int = 1, x = 1.0, y = 2.0,  map_name="map1", w_type=1) 
        B2 = self.create_waypoint(name = "B2", id_int = 1, x = 1.0, y = 2.0,  map_name="map1", w_type=1)
        M = self.create_waypoint(name = "M", id_int = 2, x = 2.0, y = 3.0,  map_name="map1", w_type=4, next_map_name="map2", next_x=0.0, next_yaw=0.0)
        C1 = self.create_waypoint(name = "C1", id_int = 3, x = 2.0, y = 2.0,  map_name="map2", w_type=1)
        C2 = self.create_waypoint(name = "C2", id_int = 3, x = 2.0, y = 2.0,  map_name="map2", w_type=1)
        D = self.create_waypoint(name = "D", id_int = 4, x = 2.0, y = 3.0,  map_name="map2", w_type=1)
        waypoints = [B1, A, B2, M, C1, D]
        waypoint_list_msg.waypoints = waypoints

        # 路点数量
        waypoint_list_msg.total_waypoints = len(waypoints)
        
        # 起始地图名称
        waypoint_list_msg.start_map_name = waypoint_list_msg.waypoints[0].map_name 
        
        # 路线数量
        waypoint_list_msg.total_path = 1 
        print(f'msg: {waypoint_list_msg}')
        self.publisher_.publish(waypoint_list_msg)
        self.get_logger().info(f'已发布包含航点列表')

        response.success = True
        response.message = "[MOCK] 路径已重新规划并下发"
        return response

    def create_waypoint(self, name, id_int, x, y, map_name="map1", w_type=1, next_map_name="map2", next_x=0.0, next_y=0.0, next_yaw=0.0) -> Waypoint:
        """快速构造 Waypoint 消息的辅助函数"""
        wp = Waypoint()
        wp.name = name
        wp.id = id_int
        wp.map_name = map_name
        wp.x = x
        wp.y = y
        wp.yaw = 0.0
        wp.lng  = 0.0               # ← 加上
        wp.lat  = 0.0               # ← 加上
        wp.type = w_type  # 1=正常, 4=地图切换
        wp.tolerance = 0.5
        if w_type == 1:
            # 即使不用的字段也初始化一下，避免某些版本的 ROS 报错
            wp.next_map_name = ""
            wp.next_x = 0.0
            wp.next_y = 0.0
            wp.next_yaw = 0.0
        else:
            wp.next_map_name = next_map_name
            wp.next_x = next_x
            wp.next_y = next_y
            wp.next_yaw = next_yaw
        return wp


def main(args=None):
    rclpy.init(args=args)
    node = MockRoutePlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()