#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from multi_map_navigation_msgs.msg import StartEndGraph, Waypoint, Edge
from multi_map_navigation_msgs.srv import PubNewPath
from std_msgs.msg import Header
import time

class TestGraphPublisher(Node):
    def __init__(self):
        super().__init__('test_graph_publisher')
        self.publisher_ = self.create_publisher(StartEndGraph, '/start_end_graph', 10)

    def create_waypoint(self, name, id_int, x, y, yaw, map_name="map1", w_type=1, next_map_name="map2", next_x=0.0, next_y=0.0, next_yaw=0.0, lng=0.0, lat=0.0)->Waypoint:
        """快速构造 Waypoint 消息的辅助函数"""
        wp = Waypoint()
        wp.name = name
        wp.id = id_int
        wp.map_name = map_name
        wp.x = x
        wp.y = y
        wp.yaw = yaw
        wp.type = w_type  # 1=正常, 4=地图切换
        wp.tolerance = 0.5
        wp.lng=lng
        wp.lat=lat
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
    
    def create_edge(self, start_node_id, end_node_id, weight)-> Edge:
        e = Edge()
        e.start_node_id = start_node_id
        e.end_node_id = end_node_id
        e.weight = weight
        return e

    def publish_once_1(self):
        msg = StartEndGraph()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # 1. 构造节点 (Nodes)
        self.start = self.create_waypoint(name = "start", id_int = 0, x = -127.0, y = 430.0, yaw=1.74, map_name="map2", w_type=1)
        self.N1 = self.create_waypoint(name = "N1", id_int = 1, x = -101.0, y = 401.0, yaw=1.57,  map_name="map2", w_type=1)
        self.N2 = self.create_waypoint(name = "N2", id_int = 2, x = -92.0, y = 350.0,  yaw=1.74, map_name="map2", w_type=1)
        self.N3 = self.create_waypoint(name = "N3", id_int = 3, x = -109.0, y = 339.0,  yaw=1.74, map_name="map2", w_type=4)
        self.end = self.create_waypoint(name = "end", id_int = 4, x = -71.6, y = 237.0,  yaw=1.57, map_name="map1", w_type=1)
        
        
        msg.nodes = [self.start, self.N1, self.N2, self.N3, self.end]

        # 2. 设置任务的起点和终点
        msg.start = self.N1
        msg.end = self.start

        # 3. 构造边 (Edges)
        e1 = self.create_edge(start_node_id=0 , end_node_id=1, weight=5.3)
        e2 = self.create_edge(start_node_id=1 , end_node_id=2, weight=6.5)
        e3 = self.create_edge(start_node_id=2 , end_node_id=4, weight=13.3)
        e4 = self.create_edge(start_node_id=0 , end_node_id=3, weight=11.8)
        e5 = self.create_edge(start_node_id=3 , end_node_id=4, weight=12.5)


        msg.edges = [e1, e2, e3, e4, e5]

        # 4. 执行发布
        self.publisher_.publish(msg)
        self.get_logger().info('成功发布 StartEndGraph 消息！')

            
def main(args=None):
    rclpy.init(args=args)
    node = TestGraphPublisher()
    try:
    
        node.publish_once_1()
        # time.sleep(2)
        # node.call_service([node.C3, node.C2])
        #node.call_service([node.A1, node.A4])
        # time.sleep(5)
        # node.call_service([node.A4, node.M1])
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()