#!/usr/bin/env python3
"""
锁控制器客户端示例 - 查询状态
"""
import rclpy
from rclpy.node import Node
from lock_controller.srv import QueryLockStatus
import sys


class StatusQueryClient(Node):
    def __init__(self):
        super().__init__('status_query_client')
        self.client = self.create_client(QueryLockStatus, 'query_status')

        # 等待服务
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待query_status服务启动...')

    def query_status(self, board_address, lock_address):
        """
        查询锁状态
        :param board_address: 板地址 (1-32)
        :param lock_address: 锁地址 (1-24, 0=查询所有锁)
        :return: 是否成功
        """
        request = QueryLockStatus.Request()
        request.board_address = board_address
        request.lock_address = lock_address

        if lock_address == 0:
            self.get_logger().info(f'查询板{board_address}的所有锁状态...')
        else:
            self.get_logger().info(f'查询板{board_address}锁{lock_address}的状态...')

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                if lock_address == 0:
                    # 显示所有锁状态
                    self.get_logger().info(f'板{board_address}锁状态:')
                    for i, locked in enumerate(response.all_lock_states):
                        status = "🔒 已锁" if locked else "🔓 已开"
                        self.get_logger().info(f'  锁{i+1}: {status}')
                else:
                    # 显示单个锁状态
                    status = "🔒 已锁" if response.is_locked else "🔓 已开"
                    self.get_logger().info(f'板{board_address}锁{lock_address}: {status}')
            else:
                self.get_logger().error(f'查询失败: {response.message}')
            return response.success
        else:
            self.get_logger().error('服务调用失败')
            return False


def main(args=None):
    rclpy.init(args=args)

    # 解析命令行参数
    if len(sys.argv) < 3:
        print("用法: ros2 run lock_controller query_status_example.py <板地址> <锁地址>")
        print("示例: ros2 run lock_controller query_status_example.py 1 1")
        print("      ros2 run lock_controller query_status_example.py 1 0  # 0表示查询所有锁")
        sys.exit(1)

    board_addr = int(sys.argv[1])
    lock_addr = int(sys.argv[2])

    client = StatusQueryClient()
    client.query_status(board_addr, lock_addr)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
