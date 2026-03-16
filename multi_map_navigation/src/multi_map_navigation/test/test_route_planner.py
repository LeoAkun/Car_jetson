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
       
        # 发布路径的客户端
        self.pub_new_path_client = self.create_client(
            PubNewPath, '/route_planner/pub_new_path',
        )
        self.get_logger().info('等待路径服务...')
        self.pub_new_path_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('发布路径服务已连接')

    def create_waypoint(self, name, id_int, x, y, map_name="map1", w_type=1, next_map_name="map2", next_x=0.0, next_y=0.0, next_yaw=0.0):
        """快速构造 Waypoint 消息的辅助函数"""
        wp = Waypoint()
        wp.name = name
        wp.id = id_int
        wp.map_name = map_name
        wp.x = x
        wp.y = y
        wp.yaw = 0.0
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

    def publish_once(self):
        msg = StartEndGraph()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # 1. 构造节点 (Nodes)
        # 我们模拟一个简单的 Y 字型图
        p_start = self.create_waypoint(name = "start", id_int = 0, x = 0.0, y = 0.0, map_name="map1", w_type=1)
        p_a = self.create_waypoint(name = "2教送货点", id_int = 1, x = 1.0, y = 2.0,  map_name="map1", w_type=1)
        p_b = self.create_waypoint(name = "map1路过点1", id_int = 2, x = 2.0, y = 2.0,  map_name="map1", w_type=1)
        p_c = self.create_waypoint(name = "map1路过点2", id_int = 3, x = 2.0, y = 3.0,  map_name="map1", w_type=1)
        p_d = self.create_waypoint(name = "地图切换点", id_int = 4, x = 2.0, y = 4.0,  map_name="map1", w_type=4, next_map_name="map2", next_x=0.0, next_yaw=0.0)
        p_e = self.create_waypoint(name = "map2路过点1", id_int = 5, x = 2.0, y = 2.0,  map_name="map2", w_type=1)
        p_f = self.create_waypoint(name = "map2路过点2", id_int = 6, x = 2.0, y = 3.0,  map_name="map2", w_type=1)
        p_end = self.create_waypoint(name = "end", id_int = 7, x = 3.0, y = 4.0,  map_name="map2", w_type=1)

        
        msg.nodes = [p_start, p_a, p_b, p_c, p_d, p_e, p_f, p_end]

        # 2. 设置任务的起点和终点
        msg.start = p_start
        msg.end = p_end

        # 3. 构造边 (Edges)
        # 边1: start -> 2教送货点 (权重 2.0)
        e1 = Edge()
        e1.start_node_id = 0
        e1.end_node_id = 1
        e1.weight = 1.0

        # 边2: 2教送货点 -> map1路过点1 (权重 2.5)
        e2 = Edge()
        e2.start_node_id = 1
        e2.end_node_id = 2
        e2.weight = 1.0

        # 边3: 2教送货点 -> map1路过点2 (权重 2.5)
        e3 = Edge()
        e3.start_node_id = 1
        e3.end_node_id = 3
        e3.weight = 1.0


        # 边4: map1路过点1 -> 地图切换点 (权重 2.5)
        e4 = Edge()
        e4.start_node_id = 2
        e4.end_node_id = 4
        e4.weight = 1.0

        
        # 边5: map1路过点2 -> 地图切换点 (权重 2.5)
        e5 = Edge()
        e5.start_node_id = 3
        e5.end_node_id = 4
        e5.weight = 2.0

        # 边6: 地图切换点 -> map2路过点1 (这是一条捷径，权重 5.0)
        e6 = Edge()
        e6.start_node_id = 4
        e6.end_node_id = 5
        e6.weight = 1.0

        # 边7: 地图切换点 -> map2路过点2 (这是一条捷径，权重 5.0)
        e7 = Edge()
        e7.start_node_id = 4
        e7.end_node_id = 6
        e7.weight = 1.0

        # 边8: map2路过点1 -> end (这是一条捷径，权重 4.0)
        e8 = Edge()
        e8.start_node_id = 5
        e8.end_node_id = 7
        e8.weight = 1.0

        # 边9: map2路过点2 -> end (这是一条捷径，权重 4.0)
        e9 = Edge()
        e9.start_node_id = 6
        e9.end_node_id = 7
        e9.weight = 1.0

        msg.edges = [e1, e2, e3, e4, e5, e6, e7, e8, e9]

        # 4. 执行发布
        self.publisher_.publish(msg)
        self.get_logger().info('成功发布 StartEndGraph 消息！')

    def call_service(self):
        req = PubNewPath.Request()
        
        # ... (构造 waypoints 的代码保持不变)
        p_start = self.create_waypoint(name = "start", id_int = 0, x = 0.0, y = 0.0, map_name="map1", w_type=1)
        p_a = self.create_waypoint(name = "2教送货点", id_int = 1, x = 1.0, y = 2.0,  map_name="map1", w_type=1)
        p_b = self.create_waypoint(name = "map1路过点1", id_int = 2, x = 2.0, y = 2.0,  map_name="map1", w_type=1)
        p_d = self.create_waypoint(name = "地图切换点", id_int = 4, x = 2.0, y = 4.0,  map_name="map1", w_type=4, next_map_name="map2", next_x=0.0, next_yaw=0.0)
        p_e = self.create_waypoint(name = "map2路过点1", id_int = 5, x = 2.0, y = 2.0,  map_name="map2", w_type=1)
        
        # p_start = self.create_waypoint(name = "start", id_int = 0, x = 0.0, y = 0.0, map_name="map1", w_type=1)
        # p_a = self.create_waypoint(name = "2教送货点", id_int = 1, x = 1.0, y = 2.0,  map_name="map1", w_type=1)
        # p_b = self.create_waypoint(name = "map1路过点1", id_int = 2, x = 2.0, y = 2.0,  map_name="map1", w_type=1)
        # p_d = self.create_waypoint(name = "地图切换点", id_int = 4, x = 2.0, y = 4.0,  map_name="map1", w_type=4, next_map_name="map2", next_x=0.0, next_yaw=0.0)
        # p_e = self.create_waypoint(name = "map2路过点1", id_int = 5, x = 2.0, y = 2.0,  map_name="map2", w_type=1)
        
        points = [p_start, p_a, p_b, p_d, p_e]
        req.path_name = "path1"
        req.points = points
        
        self.get_logger().info(f'正在请求切换路径，当前路径: {req.path_name}')
        
        # 异步调用
        future = self.pub_new_path_client.call_async(req)
        
        # --- 修改部分：使用 rclpy.spin_until_future_complete ---
        # 这允许 ROS 2 在等待结果的同时处理后台数据
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            try:
                response = future.result()
                self.get_logger().info('--- 服务调用成功 ---')
                self.get_logger().info(f'是否成功: {response.success}')
                self.get_logger().info(f'反馈消息: {response.message}')
            except Exception as e:
                self.get_logger().error(f'服务调用产生异常: {e}')
        else:
            self.get_logger().error('服务调用超时！')
            
def main(args=None):
    rclpy.init(args=args)
    node = TestGraphPublisher()
    try:
        node.publish_once()
        time.sleep(5)
        # node.call_service()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()