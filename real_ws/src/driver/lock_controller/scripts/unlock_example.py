#!/usr/bin/env python3
"""
锁控制器客户端示例 - 开锁
"""
import rclpy
from rclpy.node import Node
from lock_controller.srv import UnlockCommand
import sys


class LockClient(Node):
    def __init__(self):
        super().__init__('lock_client')
        self.client = self.create_client(UnlockCommand, 'unlock')

        # 等待服务
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待unlock服务启动...')

    def unlock(self, board_address, lock_address):
        """
        开锁
        :param board_address: 板地址 (1-32)
        :param lock_address: 锁地址 (1-24, 0=全部锁)
        :return: 是否成功
        """
        request = UnlockCommand.Request()
        request.board_address = board_address
        request.lock_address = lock_address

        self.get_logger().info(f'发送开锁命令: 板{board_address}, 锁{lock_address}')

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'结果: {"成功" if response.success else "失败"}')
            self.get_logger().info(f'消息: {response.message}')
            return response.success
        else:
            self.get_logger().error('服务调用失败')
            return False


def main(args=None):
    rclpy.init(args=args)

    # 解析命令行参数
    if len(sys.argv) < 3:
        print("用法: ros2 run lock_controller unlock_example.py <板地址> <锁地址>")
        print("示例: ros2 run lock_controller unlock_example.py 1 1")
        print("      ros2 run lock_controller unlock_example.py 1 0  # 0表示全部锁")
        sys.exit(1)

    board_addr = int(sys.argv[1])
    lock_addr = int(sys.argv[2])

    client = LockClient()
    client.unlock(board_addr, lock_addr)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
