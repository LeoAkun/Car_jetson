#!/usr/bin/env python3
"""
锁控制器客户端示例 - 订阅锁状态变化
"""
import rclpy
from rclpy.node import Node
from lock_controller.msg import LockStatus, AllLockStatus


class LockStatusSubscriber(Node):
    def __init__(self):
        super().__init__('lock_status_subscriber')

        # 订阅单个锁状态
        self.lock_status_sub = self.create_subscription(
            LockStatus,
            'lock_status',
            self.lock_status_callback,
            10)

        # 订阅所有锁状态
        self.all_lock_status_sub = self.create_subscription(
            AllLockStatus,
            'all_lock_status',
            self.all_lock_status_callback,
            10)

        self.get_logger().info('开始监听锁状态变化...')
        self.get_logger().info('按Ctrl+C退出')

    def lock_status_callback(self, msg):
        """单个锁状态变化回调"""
        status = "🔒 已锁" if msg.is_locked else "🔓 已开"
        self.get_logger().info(
            f'[单锁状态变化] 板{msg.board_address} 锁{msg.lock_address}: {status}')

    def all_lock_status_callback(self, msg):
        """所有锁状态回调"""
        self.get_logger().info(f'[全部锁状态] 板{msg.board_address}:')
        for i, locked in enumerate(msg.lock_states):
            status = "🔒 已锁" if locked else "🔓 已开"
            self.get_logger().info(f'  锁{i+1}: {status}')


def main(args=None):
    rclpy.init(args=args)

    subscriber = LockStatusSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info('用户中断，退出...')

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
