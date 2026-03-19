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

    def create_waypoint(self, name, id_int, x, y, map_name="map1", w_type=1, next_map_name="map2", next_x=0.0, next_y=0.0, next_yaw=0.0, lng=0.0, lat=0.0):
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

    def publish_once_1(self):
        msg = StartEndGraph()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # 1. 构造节点 (Nodes)
        self.A = self.create_waypoint(name = "A", id_int = 0, x = 0.0, y = 0.0, map_name="map1", w_type=1)
        self.B1 = self.create_waypoint(name = "B1", id_int = 1, x = 1.0, y = 2.0,  map_name="map1", w_type=1)
        self.B2 = self.create_waypoint(name = "B2", id_int = 2, x = 2.0, y = 2.0,  map_name="map1", w_type=1)
        self.M = self.create_waypoint(name = "M", id_int = 3, x = 2.0, y = 3.0,  map_name="map1", w_type=4, next_map_name="map2", next_x=0.0, next_yaw=0.0)
        self.C1 = self.create_waypoint(name = "C1", id_int = 4, x = 2.0, y = 4.0,  map_name="map1", w_type=4)
        self.C2 = self.create_waypoint(name = "C2", id_int = 5, x = 2.0, y = 2.0,  map_name="map2", w_type=1)
        self.D = self.create_waypoint(name = "D", id_int = 6, x = 2.0, y = 3.0,  map_name="map2", w_type=1)
        
        msg.nodes = [self.A, self.B1, self.B2, self.M, self.C1, self.C2, self.D]

        # 2. 设置任务的起点和终点
        msg.start = self.A
        msg.end = self.D

        # 3. 构造边 (Edges)
        e_A_B1 = Edge()
        e_A_B1.start_node_id = 0
        e_A_B1.end_node_id = 1
        e_A_B1.weight = 1.0

        e_A_B2 = Edge()
        e_A_B2.start_node_id = 0
        e_A_B2.end_node_id = 2
        e_A_B2.weight = 1.0

        e_B1_M = Edge()
        e_B1_M.start_node_id = 1
        e_B1_M.end_node_id = 3
        e_B1_M.weight = 1.0

        e_B2_M = Edge()
        e_B2_M.start_node_id = 2
        e_B2_M.end_node_id = 3
        e_B2_M.weight = 1.0

        e_M_C1 = Edge()
        e_M_C1.start_node_id = 3
        e_M_C1.end_node_id = 4
        e_M_C1.weight = 1.0

        e_M_C2 = Edge()
        e_M_C2.start_node_id = 3
        e_M_C2.end_node_id = 5
        e_M_C2.weight = 1.0

        e_C1_D = Edge()
        e_C1_D.start_node_id = 4
        e_C1_D.end_node_id = 6
        e_C1_D.weight = 1.0

        e_C2_D = Edge()
        e_C2_D.start_node_id = 5
        e_C2_D.end_node_id = 6
        e_C2_D.weight = 1.0

        msg.edges = [e_A_B1, e_A_B2, e_B1_M, e_B2_M, e_M_C1, e_M_C2, e_C1_D, e_C2_D]

        # 4. 执行发布
        self.publisher_.publish(msg)
        self.get_logger().info('成功发布 StartEndGraph 消息！')


    def publish_once_2(self):
        msg = StartEndGraph()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # 1. 构造节点 (Nodes)
        self.A1 = self.create_waypoint(name = "A1", id_int = 0, x = 0.0, y = 0.0, map_name="map1", w_type=1, lng=1.0, lat=4.0)
        self.A2 = self.create_waypoint(name = "A2", id_int = 1, x = 0.0, y = 0.0, map_name="map1", w_type=1, lng=2.0, lat=5.0)
        self.A3 = self.create_waypoint(name = "A3", id_int = 2, x = 0.0, y = 0.0, map_name="map1", w_type=1, lng=4.0, lat=5.0)
        self.A4 = self.create_waypoint(name = "A4", id_int = 3, x = 0.0, y = 0.0, map_name="map1", w_type=1, lng=3.0, lat=4.0)
        self.M1 = self.create_waypoint(name = "M1", id_int = 4, x = 2.0, y = 3.0,  map_name="map1", w_type=4, next_map_name="map2", next_x=0.0, next_yaw=0.0, lng=5.0, lat=4.0)
        self.B1 = self.create_waypoint(name = "B1", id_int = 5, x = 1.0, y = 2.0,  map_name="map2", w_type=1, lng=6.0, lat=5.0)
        self.B2 = self.create_waypoint(name = "B2", id_int = 6, x = 2.0, y = 2.0,  map_name="map2", w_type=1, lng=7.0, lat=5.0)
        self.M2 = self.create_waypoint(name = "M2", id_int = 7, x = 4.0, y = 5.0,  map_name="map1", w_type=4, next_map_name="map3", next_x=0.0, next_yaw=0.0, lng=5.0, lat=3.0)
        self.C1 = self.create_waypoint(name = "C1", id_int = 8, x = 2.0, y = 4.0,  map_name="map3", w_type=1, lng=6.0, lat=1.0)
        self.C2 = self.create_waypoint(name = "C2", id_int = 9, x = 2.0, y = 2.0,  map_name="map3", w_type=1, lng=7.0, lat=2.0)
        self.C3 = self.create_waypoint(name = "C3", id_int = 10, x = 2.0, y = 2.0,  map_name="map3", w_type=1, lng=7.0, lat=1.0)
        
        msg.nodes = [self.A1, self.A2, self.A3, self.A4, self.B1, self.B2, self.M1, self.M2,self.C1, self.C2, self.C3]

        # 2. 设置任务的起点和终点
        # msg.start = self.A1
        # msg.end = self.C3
        msg.start = self.C3
        msg.end = self.A1

        # 3. 构造边 (Edges)
        e_A1_A2 = Edge()
        e_A1_A2.start_node_id = 0
        e_A1_A2.end_node_id = 1
        e_A1_A2.weight = 3.0

        e_A2_A3 = Edge()
        e_A2_A3.start_node_id = 1
        e_A2_A3.end_node_id = 2
        e_A2_A3.weight = 2.0

        e_A3_A4 = Edge()
        e_A3_A4.start_node_id = 2
        e_A3_A4.end_node_id = 3
        e_A3_A4.weight = 4.0

        e_A1_A4 = Edge()
        e_A1_A4.start_node_id = 0
        e_A1_A4.end_node_id = 3
        e_A1_A4.weight = 1.0

        e_A2_A4 = Edge()
        e_A2_A4.start_node_id = 1
        e_A2_A4.end_node_id = 3
        e_A2_A4.weight = 1.0

        e_A3_M1 = Edge()
        e_A3_M1.start_node_id = 2
        e_A3_M1.end_node_id = 4
        e_A3_M1.weight = 2.0

        e_A4_M1 = Edge()
        e_A4_M1.start_node_id = 3
        e_A4_M1.end_node_id = 4
        e_A4_M1.weight = 6.0

        e_A1_M1 = Edge()
        e_A1_M1.start_node_id = 0
        e_A1_M1.end_node_id = 4
        e_A1_M1.weight = 10.0

        e_A1_M2 = Edge()
        e_A1_M2.start_node_id = 0
        e_A1_M2.end_node_id = 7
        e_A1_M2.weight = 15.0

        e_M1_M2 = Edge()
        e_M1_M2.start_node_id = 4
        e_M1_M2.end_node_id = 7
        e_M1_M2.weight = 15.0

        e_M1_B1 = Edge()
        e_M1_B1.start_node_id = 4
        e_M1_B1.end_node_id = 5
        e_M1_B1.weight = 3.0

        e_M1_B2 = Edge()
        e_M1_B2.start_node_id = 4
        e_M1_B2.end_node_id = 6
        e_M1_B2.weight = 4.0  

        e_B1_B2 = Edge()
        e_B1_B2.start_node_id = 5
        e_B1_B2.end_node_id = 6
        e_B1_B2.weight = 1.0 
        
        e_M2_C1 = Edge()
        e_M2_C1.start_node_id = 7
        e_M2_C1.end_node_id = 8
        e_M2_C1.weight = 6.0 

        e_C1_C2 = Edge()
        e_C1_C2.start_node_id = 8
        e_C1_C2.end_node_id = 9
        e_C1_C2.weight = 1.0 

        e_C1_C3 = Edge()
        e_C1_C3.start_node_id = 8
        e_C1_C3.end_node_id = 10
        e_C1_C3.weight = 2.0 

        e_C2_C3 = Edge()
        e_C2_C3.start_node_id = 9
        e_C2_C3.end_node_id = 10
        e_C2_C3.weight = 2.0 

        e_C2_B2 = Edge()
        e_C2_B2.start_node_id = 9
        e_C2_B2.end_node_id = 6
        e_C2_B2.weight = 1.0 

        msg.edges = [e_A1_A2, e_A1_A4, e_A1_M1, e_A1_M2, e_A2_A3, e_A2_A4, e_A3_A4, e_A3_M1, e_A4_M1, e_M1_M2, e_M1_B2, e_M1_B1, e_B1_B2, e_C2_B2, e_M2_C1, e_C1_C2, e_C1_C3, e_C2_C3, ]

        # 4. 执行发布
        self.publisher_.publish(msg)
        self.get_logger().info('成功发布 StartEndGraph 消息！')

    def call_service(self, point_list):
        '''
        function:
        param: @ point_list: 上一次规划的路径中, 机器人已经走过的航点
        '''
        req = PubNewPath.Request()
        
        points = point_list
        req.points = points
        
        
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
        node.publish_once_2()
        time.sleep(2)
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