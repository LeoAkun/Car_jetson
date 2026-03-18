import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from multi_map_navigation_msgs.msg import WaypointList, Waypoint
class SimpleSubscriber(Node):
    def __init__(self):
        # 初始化节点名称为 'simple_subscriber'
        super().__init__('simple_subscriber')
        
        # 创建订阅者
        # 参数：消息类型, 话题名称, 回调函数, 队列深度(QoS)
        self.subscription = self.create_subscription(
            WaypointList,
            '/test/waypoint_list',
            self.listener_callback,
            10)
        self.get_logger().info('订阅者节点已启动，正在等待消息...')

    def listener_callback(self, msg):
        # 收到消息后的处理逻辑
        self.get_logger().info(f'我听到了: "{msg}"')

def main(args=None):
    rclpy.init(args=args)
    
    subscriber_node = SimpleSubscriber()
    
    try:
        # 循环检查消息
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点并关闭
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()