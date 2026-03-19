#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String

class Publisher(Node):
    def __init__(self):
        super().__init__('gps_mock_publisher')

        self.robot_state_pub = self.create_publisher(
            String,
            '/robot_state',
            10
        )
        
        # 每秒发布一次 (1Hz)
        self.timer = self.create_timer(1.0, self.publish_state_data)
        self.get_logger().info('robot 模拟发布器已启动...')

    def publish_state_data(self):
        msg = String()
        msg.data = "running"
        self.robot_state_pub.publish(msg)
        self.get_logger().debug(f'机器人状态: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()