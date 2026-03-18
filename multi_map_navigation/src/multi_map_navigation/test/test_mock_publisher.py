#!/usr/bin/env python3
"""
模拟数据发布节点 - 用于测试status_reporter的远程驾驶请求功能
发布话题：
- /vehicle_status (速度)
- /robot_state (任务状态)
- /sensing/gnss/pose_with_covariance (GPS位置)
- /battery_status (电池状态)
"""

import rclpy
from rclpy.node import Node
from yunle_msgs.msg import VehicleStatus, Battery
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import time
import sys


class MockDataPublisher(Node):
    def __init__(self):
        super().__init__('mock_data_publisher')

        # 发布车辆状态
        self.vehicle_status_pub = self.create_publisher(
            VehicleStatus,
            '/vehicle_status',
            10
        )

        # 发布机器人状态
        self.robot_state_pub = self.create_publisher(
            String,
            '/robot_state',
            10
        )

        # 发布GPS位置
        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/sensing/gnss/pose_with_covariance',
            10
        )

        # 发布电池状态
        self.battery_pub = self.create_publisher(
            Battery,
            '/battery_status',
            10
        )

        # 创建定时器，每秒发布一次数据
        self.timer = self.create_timer(1.0, self.publish_data)

        # 模拟数据
        self.cur_speed = 0.0
        self.task_status = "idle"
        self.running_time = 0
        self.test_mode = None

        # GPS数据（北京某位置）
        self.gps_lat = 39.9042  # 纬度
        self.gps_lng = 116.4074  # 经度
        self.gps_alt = 50.0     # 海拔

        # 电池数据
        self.battery_capacity = 85.0  # 电量百分比
        self.battery_voltage = 48.2   # 电压
        self.battery_ampere = 12.5    # 电流

        self.get_logger().info('模拟数据发布节点已启动')

    def show_menu(self):
        """显示测试菜单"""
        print("\n" + "="*70)
        print("          Status Reporter 远程驾驶请求功能测试")
        print("="*70)
        print("\n请选择测试场景：\n")
        print("  1. 正常触发场景 (速度=0 + 任务running，30秒后触发)")
        print("  2. 速度恢复场景 (速度=0 → 非0，重置计时器)")
        print("  3. 任务空闲场景 (速度=0 + 任务idle，不触发)")
        print("  4. 交替变化场景 (速度和状态交替变化)")
        print("  5. 快速触发场景 (配置改为10秒测试)")
        print("  6. 持续运行场景 (模拟正常行驶)")
        print("  0. 退出")
        print("\n" + "="*70)

    def setup_test_mode_1(self):
        """场景1: 正常触发 - 速度=0 + 任务running"""
        print("\n[场景1] 正常触发场景")
        print("- 速度: 0.0 km/h")
        print("- 任务状态: running")
        print("- GPS: 固定位置")
        print("- 电池: 85%")
        print("- 预期: 30秒后触发远程驾驶请求并关闭进程\n")

        self.cur_speed = 0.0
        self.task_status = "running"
        self.running_time = 0
        self.test_mode = 1

    def setup_test_mode_2(self):
        """场景2: 速度恢复 - 速度从0变为非0"""
        print("\n[场景2] 速度恢复场景")
        print("- 前15秒: 速度=0.0 km/h")
        print("- 后15秒: 速度=10.0 km/h")
        print("- GPS: 固定位置")
        print("- 电池: 85%")
        print("- 预期: 不触发请求，计时器重置\n")

        self.cur_speed = 0.0
        self.task_status = "running"
        self.running_time = 0
        self.test_mode = 2

    def setup_test_mode_3(self):
        """场景3: 任务空闲 - 速度=0 + 任务idle"""
        print("\n[场景3] 任务空闲场景")
        print("- 速度: 0.0 km/h")
        print("- 任务状态: idle")
        print("- GPS: 固定位置")
        print("- 电池: 85%")
        print("- 预期: 不触发请求\n")

        self.cur_speed = 0.0
        self.task_status = "idle"
        self.running_time = 0
        self.test_mode = 3

    def setup_test_mode_4(self):
        """场景4: 交替变化 - 速度和状态交替变化"""
        print("\n[场景4] 交替变化场景")
        print("- 速度和任务状态每10秒变化一次")
        print("- GPS: 固定位置")
        print("- 电池: 85%")
        print("- 预期: 不触发请求，计时器不断重置\n")

        self.cur_speed = 0.0
        self.task_status = "running"
        self.running_time = 0
        self.test_mode = 4

    def setup_test_mode_5(self):
        """场景5: 快速触发 - 配置改为10秒"""
        print("\n[场景5] 快速触发场景")
        print("- 速度: 0.0 km/h")
        print("- 任务状态: running")
        print("- GPS: 固定位置")
        print("- 电池: 85%")
        print("- 预期: 10秒后触发（需先在mqtt_config.yaml中设置speed_threshold_time: 10）\n")

        self.cur_speed = 0.0
        self.task_status = "running"
        self.running_time = 0
        self.test_mode = 5

    def setup_test_mode_6(self):
        """场景6: 持续运行 - 模拟正常行驶"""
        print("\n[场景6] 持续运行场景")
        print("- 速度: 15.0 km/h (恒定)")
        print("- 任务状态: running")
        print("- GPS: 模拟移动")
        print("- 电池: 模拟放电")
        print("- 预期: 不触发请求\n")

        self.cur_speed = 15.0
        self.task_status = "running"
        self.running_time = 0
        self.test_mode = 6

    def publish_data(self):
        """发布模拟数据"""
        if self.test_mode is None:
            return

        self.running_time += 1

        # 根据测试模式调整数据
        if self.test_mode == 2:
            # 场景2: 15秒后速度恢复
            if self.running_time == 16:
                self.cur_speed = 10.0
                print(f"\n[T={self.running_time}s] 速度恢复: 0.0 → 10.0 km/h")

        elif self.test_mode == 4:
            # 场景4: 每10秒交替变化
            if self.running_time % 10 == 1 and self.running_time > 1:
                if self.cur_speed == 0.0:
                    self.cur_speed = 10.0
                    self.task_status = "idle"
                    print(f"\n[T={self.running_time}s] 速度变为10.0, 任务变为idle")
                else:
                    self.cur_speed = 0.0
                    self.task_status = "running"
                    print(f"\n[T={self.running_time}s] 速度变为0.0, 任务变为running")

        elif self.test_mode == 6:
            # 场景6: 模拟GPS移动和电池放电
            # GPS缓慢向北移动
            self.gps_lat += 0.0001
            self.gps_lng += 0.0001

            # 电池缓慢放电
            if self.running_time % 10 == 0 and self.battery_capacity > 20:
                self.battery_capacity -= 0.5
                self.battery_voltage -= 0.01

        # 发布车辆状态
        vehicle_msg = VehicleStatus()
        vehicle_msg.cur_speed = self.cur_speed
        self.vehicle_status_pub.publish(vehicle_msg)

        # 发布机器人状态
        robot_msg = String()
        robot_msg.data = self.task_status
        self.robot_state_pub.publish(robot_msg)

        # 发布GPS数据
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'map'
        gps_msg.latitude = self.gps_lat
        gps_msg.longitude = self.gps_lng
        gps_msg.altitude = self.gps_alt
        self.gps_pub.publish(gps_msg)

        # 发布电池数据
        battery_msg = Battery()
        battery_msg.capacity = self.battery_capacity
        battery_msg.voltage = self.battery_voltage
        battery_msg.ampere = self.battery_ampere
        battery_msg.charge_status = 0  # 0=放电, 1=充电
        self.battery_pub.publish(battery_msg)

        # 显示当前状态
        status_display = f'[{self.running_time}s] 速度={self.cur_speed}km/h, 任务={self.task_status}, ' \
                        f'GPS=({self.gps_lat:.4f}, {self.gps_lng:.4f}), 电量={self.battery_capacity:.1f}%'

        # 每10秒或关键时间点打印
        if self.running_time % 10 == 0 or self.running_time in [15, 16, 30]:
            print(status_display)

        # 场景1和5的关键提示
        if self.test_mode in [1, 5] and self.running_time == 30:
            print("\n" + "!"*70)
            print("已达到30秒，应该已经触发远程驾驶请求！")
            print("!"*70 + "\n")

        elif self.test_mode == 5 and self.running_time == 10:
            print("\n" + "!"*70)
            print("已达到10秒，应该已经触发远程驾驶请求！")
            print("!"*70 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = MockDataPublisher()

    try:
        # 显示菜单
        node.show_menu()

        # 获取用户选择
        try:
            choice = input("\n请输入选项 (0-6): ").strip()
            choice = int(choice)

            if choice == 0:
                print("\n退出测试程序")
                return

            elif choice == 1:
                node.setup_test_mode_1()
                print("开始测试... (按Ctrl+C停止)\n")

            elif choice == 2:
                node.setup_test_mode_2()
                print("开始测试... (按Ctrl+C停止)\n")

            elif choice == 3:
                node.setup_test_mode_3()
                print("开始测试... (按Ctrl+C停止)\n")

            elif choice == 4:
                node.setup_test_mode_4()
                print("开始测试... (按Ctrl+C停止)\n")

            elif choice == 5:
                node.setup_test_mode_5()
                print("提示: 确保已设置 speed_threshold_time: 10")
                print("开始测试... (按Ctrl+C停止)\n")

            elif choice == 6:
                node.setup_test_mode_6()
                print("开始测试... (按Ctrl+C停止)\n")

            else:
                print("\n无效选项，退出")
                return

        except ValueError:
            print("\n无效输入，退出")
            return
        except KeyboardInterrupt:
            print("\n\n用户取消")
            return

        # 运行节点
        rclpy.spin(node)

    except KeyboardInterrupt:
        print('\n\n停止发布...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
