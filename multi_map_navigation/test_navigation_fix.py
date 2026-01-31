#!/usr/bin/env python3
"""
测试导航管理器的修复
验证：
1. 导航结果判断逻辑（正确识别失败状态）
2. waypoint_list 空值检查
"""

import rclpy
from rclpy.node import Node
from multi_map_navigation_msgs.msg import WaypointList, Waypoint
from std_msgs.msg import String
import time
import sys


class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')

        # 发布航点列表
        self.waypoint_list_pub = self.create_publisher(
            WaypointList,
            '/waypoint_list',
            10
        )

        # 订阅机器人状态
        self.robot_state_sub = self.create_subscription(
            String,
            '/robot_state',
            self.robot_state_callback,
            10
        )

        self.robot_state = 'unknown'
        self.test_count = 0

    def robot_state_callback(self, msg):
        """机器人状态回调"""
        self.robot_state = msg.data
        self.get_logger().info(f'机器人状态: {self.robot_state}')

    def create_test_waypoint(self, x, y, yaw, map_name, wp_type=1):
        """创建测试航点"""
        waypoint = Waypoint()
        waypoint.id = self.test_count
        waypoint.x = x
        waypoint.y = y
        waypoint.yaw = yaw
        waypoint.map_name = map_name
        waypoint.type = wp_type
        return waypoint

    def test_case_1_normal_navigation(self):
        """测试用例1: 正常导航（3个航点）"""
        self.get_logger().info('='*60)
        self.get_logger().info('测试用例1: 正常导航序列')
        self.get_logger().info('='*60)

        msg = WaypointList()
        msg.task_id = "test_task_1"  # 字符串类型
        msg.total_waypoints = 3
        msg.start_map_name = 'map1'

        # 添加3个航点
        msg.waypoints.append(self.create_test_waypoint(1.0, 0.0, 0.0, 'map1'))
        msg.waypoints.append(self.create_test_waypoint(2.0, 0.0, 0.0, 'map1'))
        msg.waypoints.append(self.create_test_waypoint(3.0, 0.0, 0.0, 'map1'))

        self.waypoint_list_pub.publish(msg)
        self.get_logger().info('已发布3个航点，观察导航过程...')
        self.test_count += 1

        return 30  # 等待30秒

    def test_case_2_single_waypoint(self):
        """测试用例2: 单个航点"""
        self.get_logger().info('='*60)
        self.get_logger().info('测试用例2: 单个航点导航')
        self.get_logger().info('='*60)

        msg = WaypointList()
        msg.task_id = "test_task_2"  # 字符串类型
        msg.total_waypoints = 1
        msg.start_map_name = 'map1'

        msg.waypoints.append(self.create_test_waypoint(1.5, 0.5, 0.0, 'map1'))

        self.waypoint_list_pub.publish(msg)
        self.get_logger().info('已发布1个航点')
        self.test_count += 1

        return 20

    def test_case_3_invalid_map(self):
        """测试用例3: 无效地图名称（可能导致导航失败）"""
        self.get_logger().info('='*60)
        self.get_logger().info('测试用例3: 测试失败场景 - 无效地图')
        self.get_logger().info('='*60)

        msg = WaypointList()
        msg.task_id = "test_task_3"  # 字符串类型
        msg.total_waypoints = 1
        msg.start_map_name = 'invalid_map'

        msg.waypoints.append(self.create_test_waypoint(100.0, 100.0, 0.0, 'invalid_map'))

        self.waypoint_list_pub.publish(msg)
        self.get_logger().info('已发布无效地图航点，应触发失败处理逻辑')
        self.test_count += 1

        return 15

    def test_case_4_multiple_in_sequence(self):
        """测试用例4: 连续发送多个任务"""
        self.get_logger().info('='*60)
        self.get_logger().info('测试用例4: 连续任务序列')
        self.get_logger().info('='*60)

        for i in range(2):
            msg = WaypointList()
            msg.task_id = f"test_task_{4+i}"  # 字符串类型
            msg.total_waypoints = 2
            msg.start_map_name = 'map1'

            msg.waypoints.append(self.create_test_waypoint(1.0 + i*0.5, 0.0, 0.0, 'map1'))
            msg.waypoints.append(self.create_test_waypoint(2.0 + i*0.5, 0.0, 0.0, 'map1'))

            self.waypoint_list_pub.publish(msg)
            self.get_logger().info(f'已发布任务 {4+i}')
            self.test_count += 1
            time.sleep(2)

        return 25

    def check_fix_effectiveness(self):
        """检查修复是否有效"""
        self.get_logger().info('='*60)
        self.get_logger().info('检查修复效果')
        self.get_logger().info('='*60)

        checks = {
            '✓ 导航失败时不再误判为成功': '观察日志中是否还有"成功到达导航目标"后崩溃的情况',
            '✓ waypoint_list为None时有防护': '观察日志中是否出现"航点列表为空，忽略目标到达事件"',
            '✓ 状态码正确识别': '观察日志中是否显示正确的状态码（非3表示失败）',
            '✓ 不再出现TypeError崩溃': '观察进程是否稳定运行，不会因为NoneType错误退出'
        }

        self.get_logger().info('验证点：')
        for check, description in checks.items():
            self.get_logger().info(f'  {check}: {description}')

        self.get_logger().info('')
        self.get_logger().info('预期行为：')
        self.get_logger().info('  1. 导航失败时显示: "导航目标失败，状态码: X"')
        self.get_logger().info('  2. 如果waypoint_list为空: "航点列表为空，忽略目标到达事件"')
        self.get_logger().info('  3. 不再出现: TypeError: \'NoneType\' object is not subscriptable')
        self.get_logger().info('')


def main():
    rclpy.init()

    tester = NavigationTester()

    print('\n' + '='*60)
    print('导航管理器修复验证脚本')
    print('='*60)
    print('\n这个脚本会测试以下修复：')
    print('1. 导航结果判断逻辑（正确识别失败状态）')
    print('2. waypoint_list 空值检查防护')
    print('\n请确保已启动导航系统：')
    print('  ros2 launch multi_map_navigation multi_map_navigation.launch.py')
    print('\n' + '='*60)
    print('\n选择测试模式：')
    print('  1. 完整测试套件（推荐）')
    print('  2. 仅显示验证点（不发送任务）')
    print('  3. 快速测试（单个航点）')
    print('  0. 退出')

    try:
        choice = input('\n请输入选择 [1-3]: ').strip()

        if choice == '0':
            print('退出测试')
            return

        if choice == '1':
            # 完整测试套件
            print('\n开始完整测试套件...\n')
            time.sleep(1)

            # 显示验证点
            tester.check_fix_effectiveness()
            time.sleep(2)

            # 运行测试用例
            wait_times = []
            wait_times.append(tester.test_case_1_normal_navigation())
            time.sleep(wait_times[0])

            wait_times.append(tester.test_case_2_single_waypoint())
            time.sleep(wait_times[1])

            wait_times.append(tester.test_case_3_invalid_map())
            time.sleep(wait_times[2])

            wait_times.append(tester.test_case_4_multiple_in_sequence())
            time.sleep(wait_times[3])

            print('\n' + '='*60)
            print('测试完成！请检查上述日志输出：')
            print('='*60)
            print('✓ 确认没有 TypeError 崩溃')
            print('✓ 确认失败时有正确的状态码日志')
            print('✓ 确认航点列表为空时有防护日志')
            print('='*60)

        elif choice == '2':
            # 仅显示验证点
            tester.check_fix_effectiveness()

        elif choice == '3':
            # 快速测试
            print('\n开始快速测试...\n')
            tester.check_fix_effectiveness()
            time.sleep(1)
            wait_time = tester.test_case_2_single_waypoint()
            time.sleep(wait_time)
            print('\n快速测试完成！')

        else:
            print('无效选择')

    except KeyboardInterrupt:
        print('\n\n测试被中断')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
