"""
虚拟发布航点信息测试脚本
"""
import rclpy
from rclpy.node import Node
from multi_map_navigation_msgs.msg import WaypointList, Waypoint
from std_msgs.msg import Header

class WaypointListPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_list_publisher')
        # 发布到 /waypoint_list 话题
        self.publisher_ = self.create_publisher(WaypointList, '/waypoint_list_test', 10)
        
        # 定时发布
        self.timer = self.create_timer(2.0, self.publish_callback)
        self.get_logger().info('5个普通点航点列表发布器已启动...')

    def publish_callback(self):
        msg = WaypointList()
        
        # 1. 设置 Header [cite: 1]
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # 2. 设置任务元数据 [cite: 1]
        msg.task_id = "test_task_5_points"
        msg.path = "path_test_sequence111"
        msg.start_map_name = "map_office"
        msg.total_path = 1
        
        # 3. 创建 5 个普通导航点 (Type 1) [cite: 1, 2]
        waypoints = []
        positions = [
            (1.0, 1.0), (2.0, 2.0), (3.0, 3.0), (4.0, 4.0), (5.0, 5.0)
        ]
        
        for i, (pos_x, pos_y) in enumerate(positions):
            wp = Waypoint()
            wp.id = i + 1              # 航点ID从1开始 [cite: 3]
            wp.name = f"point_{i+1}"    # 航点名称 [cite: 3]
            wp.map_name = "map_office" # 所属地图 [cite: 3]
            wp.x = float(pos_x)        # X坐标 [cite: 4]
            wp.y = float(pos_y)        # Y坐标 [cite: 4]
            wp.yaw = 0.0               # 偏航角 [cite: 5]
            wp.type = 1                # 1=正常导航点 [cite: 5]
            wp.tolerance = 0.5         # 容差 [cite: 6]
            waypoints.append(wp)
            
        msg.waypoints = waypoints      # 赋值航点数组 [cite: 1]
        msg.total_waypoints = len(waypoints) # 设置总数 [cite: 2]
        
        # 4. 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f'已发布包含 {msg.total_waypoints} 个普通点的航点列表')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointListPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()