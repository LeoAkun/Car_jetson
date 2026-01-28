#!/usr/bin/env python3
"""
进程管理器
开发者B - 导航模块

管理ROS2启动进程的生命周期(重定位、LIO-SAM、Navigation2)
"""

import subprocess, os
import time
import signal
import psutil
from typing import Optional, Dict
import rclpy
from rclpy.node import Node

class ProcessManager(Node):
    def __init__(self):
        super().__init__('process_manager')

        # 声明参数
        self.declare_parameter('startup_delay', 2.0)
        self.declare_parameter('shutdown_timeout', 10.0)
        self.declare_parameter('health_check_interval', 1.0)

        # 获取参数
        self.startup_delay = self.get_parameter('startup_delay').value
        self.shutdown_timeout = self.get_parameter('shutdown_timeout').value
        self.health_check_interval = self.get_parameter('health_check_interval').value

        # 进程跟踪
        self.processes: Dict[str, subprocess.Popen] = {}

        self.get_logger().info('进程管理器已初始化')

    def launch_relocalization(self, map_name: str) -> bool:
        """
        启动重定位服务进程

        参数:
            map_name: 用于重定位的地图名称

        返回:
            启动成功返回True，否则返回False
        """
        try:
            self.get_logger().info(f'正在为地图启动重定位: {map_name}')

            # 构建启动命令
            cmd = [
                'ros2', 'launch',
                're_localization',
                'run.real_launch.py',
                f'map_name:={map_name}'
            ]

            # 启动进程
            process = subprocess.Popen(
                cmd,
                # stdout=subprocess.PIPE,
                # stderr=subprocess.PIPE,
                # preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
                start_new_session=True # 创建新的进程组
            )

            self.processes['re_localization'] = process

            # 等待启动
            time.sleep(self.startup_delay)

            # 检查进程是否正在运行
            if self.is_process_running('re_localization'):
                self.get_logger().info('重定位服务启动成功')
                return True
            else:
                self.get_logger().error('重定位服务启动失败')
                return False

        except Exception as e:
            self.get_logger().error(f'启动重定位失败: {e}')
            return False
    
    def launch_nav2_init_pose(self) -> bool:
        """
        启动 nav2_init_pose 进程

        返回:
            启动成功返回True，否则返回False
        """
        try:

            # 构建启动命令
            cmd = [
                'ros2', 'run',
                'nav2_init_pose',
                'nav2_init_pose'
            ]

            # 启动进程
            process = subprocess.Popen(
                cmd,
                # stdout=subprocess.PIPE,
                # stderr=subprocess.PIPE,
                # preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
                start_new_session=True # 创建新的进程组
            )

            self.processes['nav2_init_pose'] = process

            # 等待启动
            time.sleep(self.startup_delay)

            # 检查进程是否正在运行
            if self.is_process_running('nav2_init_pose'):
                self.get_logger().info('nav2_init_pose启动成功')
                return True
            else:
                self.get_logger().error('nav2_init_pose启动失败')
                return False

        except Exception as e:
            self.get_logger().error(f'启动nav2_init_pose失败: {e}')
            return False
        pass

    def launch_liosam(self) -> bool:
        """
        启动LIO-SAM进程

        返回:
            启动成功返回True，否则返回False
        """
        try:
            self.get_logger().info('正在启动LIO-SAM')

            # 构建启动命令
            cmd = [
                'ros2', 'launch',
                'lio_sam',
                'run.real_launch.py'
            ]

            # 启动进程
            process = subprocess.Popen(
                cmd,
                # stdout=subprocess.PIPE,
                # stderr=subprocess.PIPE,
                # preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
                start_new_session=True # 创建新的进程组
            )

            self.processes['liosam'] = process

            # 等待启动
            time.sleep(self.startup_delay)

            # 检查进程是否正在运行
            if self.is_process_running('liosam'):
                self.get_logger().info('LIO-SAM启动成功')
                return True
            else:
                self.get_logger().error('LIO-SAM启动失败')
                return False

        except Exception as e:
            self.get_logger().error(f'启动LIO-SAM失败: {e}')
            return False

    def launch_navigation2(self, map_name: str) -> bool:
        """
        启动Navigation2进程

        参数:
            map_name: 用于导航的地图名称

        返回:
            启动成功返回True，否则返回False
        """
        try:
            self.get_logger().info(f'正在为地图启动Navigation2: {map_name}')

            # 构建启动命令
            cmd = [
                'ros2', 'launch',
                'nav2',
                'run.real_launch.py',
                f'map:=/home/akun/workspace/Car_jetson/utils/src/pcd2pgm/pgm/real/{map_name}.yaml'
            ]

            # 启动进程
            process = subprocess.Popen(
                cmd,
                # stdout=subprocess.PIPE,
                # stderr=subprocess.PIPE,
                # preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
                start_new_session=True # 创建新的进程组
            )

            self.processes['navigation2'] = process

            # 等待启动
            time.sleep(self.startup_delay)

            # 检查进程是否正在运行
            if self.is_process_running('navigation2'):
                self.get_logger().info('Navigation2启动成功')
                return True
            else:
                self.get_logger().error('Navigation2启动失败')
                return False

        except Exception as e:
            self.get_logger().error(f'启动Navigation2失败: {e}')
            return False

    def shutdown_process(self, process_name: str) -> bool:
        """
        优雅地关闭特定进程

        参数:
            process_name: 要关闭的进程名称

        返回:
            关闭成功返回True，否则返回False
        """
        if process_name not in self.processes:
            self.get_logger().warn(f'未找到进程 {process_name}')
            return True

        process = self.processes[process_name]

        try:
            self.get_logger().info(f'正在关闭进程组: {process_name}')

            # 获取进程组ID（PGID）
            pgid = os.getpgid(process.pid)

            # 向整个进程组发送 SIGTERM
            os.killpg(pgid, signal.SIGTERM)

            # 等待进程终止
            start_time = time.time()
            while time.time() - start_time < self.shutdown_timeout:
                if process.poll() is not None:
                    self.get_logger().info(f'{process_name} 及其子进程已关闭')
                    del self.processes[process_name]
                    return True
                time.sleep(0.5)

            # 如果仍在运行则强制终止
            self.get_logger().warn(f'{process_name} 未能优雅终止，正在强制终止')
            os.killpg(pgid, signal.SIGKILL)
            process.wait()

            del self.processes[process_name]
            return True

        except Exception as e:
            self.get_logger().error(f'关闭 {process_name} 失败: {e}')
            return False

    def shutdown_all_processes(self) -> bool:
        """
        关闭所有托管进程

        返回:
            所有进程关闭成功返回True，否则返回False
        """
        self.get_logger().info('正在关闭所有进程')

        # 按相反顺序关闭: navigation2 -> liosam -> relocalization
        shutdown_order = ['navigation2', 'liosam', 'nav2_init_pose', 're_localization']

        all_success = True
        for process_name in shutdown_order:
            if not self.shutdown_process(process_name):
                all_success = False

        return all_success

    def is_process_running(self, process_name: str) -> bool:
        """
        检查进程是否正在运行

        参数:
            process_name: 要检查的进程名称

        返回:
            进程正在运行返回True，否则返回False
        """
        if process_name not in self.processes:
            return False

        process = self.processes[process_name]
        return process.poll() is None

    def get_process_status(self) -> Dict[str, bool]:
        """
        获取所有托管进程的状态

        返回:
            将进程名称映射到运行状态的字典
        """
        return {
            name: self.is_process_running(name)
            for name in ['re_localization', 'liosam', 'nav2_init_pose', 'navigation2']
        }

    def wait_for_clean_shutdown(self, timeout: float = 10.0) -> bool:
        """
        等待所有进程完全关闭

        参数:
            timeout: 最大等待时间（秒）

        返回:
            所有进程关闭返回True，超时返回False
        """
        start_time = time.time()

        while time.time() - start_time < timeout:
            status = self.get_process_status()
            if not any(status.values()):
                self.get_logger().info('所有进程已完全关闭')
                return True
            time.sleep(0.5)

        self.get_logger().warn('等待完全关闭超时')
        return False

    def destroy_node(self):
        """节点销毁时的清理"""
        self.shutdown_all_processes()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ProcessManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
