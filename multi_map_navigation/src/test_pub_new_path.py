"""
虚假路径规划服务，打印请求星期，默认返回失败
"""


import rclpy
from rclpy.node import Node
from multi_map_navigation_msgs.srv import PubNewPath

class MockRoutePlanner(Node):
    def __init__(self):
        super().__init__('mock_route_planner')
        # 创建服务服务端
        self.srv = self.create_service(
            PubNewPath, 
            '/route_planner/pub_new_path_test', 
            self.pub_new_path_callback
        )
        self.get_logger().info("❌ 模拟路径规划服务已启动，收到请求将返回失败。")

    def pub_new_path_callback(self, request, response):
            """
            处理 PubNewPath 请求的回调函数
            """
            self.get_logger().warn("---------------------------------------")
            self.get_logger().warn("📢 收到路径重规划请求！")
            
            # 移除末尾的 [cite: 7]，保持纯净的 Python 语法
            self.get_logger().info(f"路径名称 (path_name): {request.path_name}")
            self.get_logger().info(f"已走过的点数量 (points count): {len(request.points)}")
            
            for idx, wp in enumerate(request.points):
                self.get_logger().info(f"  [{idx}] ID: {wp.id}, Name: {wp.name}, Pose: ({wp.x:.2f}, {wp.y:.2f})")
            
            # 构造响应：按照要求返回失败
            response.success = False
            response.message = "模拟测试：手动触发的规划失败响应"
            
            self.get_logger().error(f"⚠️ 已发送失败响应: {response.message}")
            self.get_logger().warn("---------------------------------------")
            
            return response

def main(args=None):
    rclpy.init(args=args)
    node = MockRoutePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()