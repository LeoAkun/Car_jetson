#!/usr/bin/env python3
"""
进程管理器ROS2节点
开发者B - 导航模块

通过ROS2服务接口管理ROS2启动进程的生命周期(重定位、LIO-SAM、Navigation2)
"""

import subprocess
import os
import time
import signal
import psutil
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from multi_map_navigation_msgs.srv import StartProcess, ShutdownProcess, GetProcessStatus
import yaml


class ProcessManagerNode(Node):
    def __init__(self):
        super().__init__('process_manager')

        # 加载配置
        self.declare_parameter('config_file', '/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/config/navigation_config.yaml')
        config_file = self.get_parameter('config_file').value

        with open(config_file) as f:
            config = yaml.safe_load(f)

        pm_params = config.get('process_manager', {}).get('ros__parameters', {})
        self.startup_delay = pm_params.get('startup_delay', 2.0)
        self.max_startup_time = pm_params.get('max_startup_time', 15.0)
        self.health_check_interval = pm_params.get('health_check_interval', 1.0)
        self.shutdown_timeout = pm_params.get('shutdown_timeout', 10.0)

        map_params = config.get('maps', {}).get('ros__parameters', {})
        self.map_dir = map_params.get('map_directory', "/home/akun/workspace/Car_jetson/multi_map_navigation/src/map")

        # 进程跟踪
        self.processes = {}

        # 创建服务
        self.start_process_srv = self.create_service(
            StartProcess,
            '/test/process_manager/start_process',
            self.start_process_callback
        )

        self.shutdown_process_srv = self.create_service(
            ShutdownProcess,
            '/test/process_manager/shutdown_process',
            self.shutdown_process_callback
        )

        self.get_process_status_srv = self.create_service(
            GetProcessStatus,
            '/test/process_manager/get_status',
            self.get_process_status_callback
        )

        self.get_logger().info('进程管理器节点已初始化')

    def start_process_callback(self, request, response):
        """
        启动进程服务回调
        """
        process_name = request.process_name
        map_name = request.map_name

        self.get_logger().info(f'收到启动进程请求: {process_name}, map: {map_name}')

        try:
            if process_name == 're_localization':
                success = self.launch_relocalization(map_name)
                response.success = success
                response.message = '重定位启动成功' if success else '重定位启动失败'

            elif process_name == 'nav2_init_pose':
                success = self.launch_nav2_init_pose(map_name)
                response.success = success
                response.message = 'nav2_init_pose启动成功' if success else 'nav2_init_pose启动失败'

            elif process_name == 'liosam':
                success = self.launch_liosam()
                response.success = success
                response.message = 'LIO-SAM启动成功' if success else 'LIO-SAM启动失败'

            elif process_name == 'navigation2':
                success = self.launch_navigation2(map_name)
                response.success = success
                response.message = 'Navigation2启动成功' if success else 'Navigation2启动失败'

            else:
                response.success = False
                response.message = f'未知的进程名称: {process_name}'

        except Exception as e:
            response.success = False
            response.message = f'启动进程时发生异常: {str(e)}'
            self.get_logger().error(f'启动进程异常: {e}')

        return response

    def shutdown_process_callback(self, request, response):
        """
        关闭进程服务回调
        """
        process_name = request.process_name

        self.get_logger().info(f'收到关闭进程请求: {process_name}')

        try:
            success = self.shutdown_process(process_name)
            response.success = success
            response.message = f'{process_name}关闭成功' if success else f'{process_name}关闭失败'

        except Exception as e:
            response.success = False
            response.message = f'关闭进程时发生异常: {str(e)}'
            self.get_logger().error(f'关闭进程异常: {e}')

        return response

    def get_process_status_callback(self, request, response):
        """
        获取进程状态服务回调
        """
        response.re_localization_running = self.is_process_running('re_localization')
        self.get_logger().debug(f're_localization运行状态: {response.re_localization_running}')

        response.liosam_running = self.is_process_running('liosam')
        self.get_logger().debug(f'liosam运行状态: {response.liosam_running}')

        response.nav2_init_pose_running = self.is_process_running('nav2_init_pose')
        self.get_logger().debug(f'nav2_init_pose运行状态: {response.nav2_init_pose_running}')

        response.navigation2_running = self.is_process_running('navigation2')
        self.get_logger().debug(f'navigation2运行状态: {response.navigation2_running}')

        response.message = '进程状态查询成功'

        return response

    def launch_relocalization(self, map_name: str) -> bool:
        """启动重定位服务进程"""
        try:
            self.get_logger().info(f'正在为地图启动重定位: {map_name}')

            cmd = ['bash', '-c', 'echo "Mock process started"; sleep 999999']

            process = subprocess.Popen(
                cmd,
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            self.processes['re_localization'] = process
            time.sleep(self.startup_delay)

            if self.is_process_running('re_localization'):
                self.get_logger().info('重定位服务启动成功')
                return True
            else:
                self.get_logger().error('重定位服务启动失败')
                return False

        except Exception as e:
            self.get_logger().error(f'启动重定位失败: {e}')
            return False

    def launch_nav2_init_pose(self, map_name: str) -> bool:
        """启动nav2_init_pose进程"""
        try:
            cmd = [
                'bash', '-c', 'echo "Mock process started"; sleep 999999'
            ]

            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"
            env["RCUTILS_LOGGING_USE_STDOUT"] = "1"

            process = subprocess.Popen(
                cmd,
                start_new_session=True,
                env=env,
                stdout=None,
                stderr=None
            )

            self.processes['nav2_init_pose'] = process
            time.sleep(self.startup_delay)

            if self.is_process_running('nav2_init_pose'):
                self.get_logger().info('nav2_init_pose启动成功')
                return True
            else:
                self.get_logger().error('nav2_init_pose启动失败')
                return False

        except Exception as e:
            self.get_logger().error(f'启动nav2_init_pose失败: {e}')
            return False

    def launch_liosam(self) -> bool:
        """启动LIO-SAM进程"""
        try:
            self.get_logger().info('正在启动LIO-SAM')

            cmd = [
                'bash', '-c', 'echo "Mock process started"; sleep 999999'
            ]

            process = subprocess.Popen(
                cmd,
                start_new_session=True
            )

            self.processes['liosam'] = process
            time.sleep(self.startup_delay)

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
        """启动Navigation2进程"""
        try:
            self.get_logger().info(f'正在为地图启动Navigation2: {map_name}')

            cmd = [
                'bash', '-c', 'echo "Mock process started"; sleep 999999'
            ]

            process = subprocess.Popen(
                cmd,
                start_new_session=True
            )

            self.processes['navigation2'] = process
            time.sleep(self.startup_delay)

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
        """优雅地关闭特定进程"""
        if process_name not in self.processes:
            self.get_logger().warning(f'未找到进程 {process_name}')
            return True

        process = self.processes[process_name]

        try:
            self.get_logger().info(f'正在关闭进程组: {process_name}')

            pgid = os.getpgid(process.pid)
            os.killpg(pgid, signal.SIGTERM)

            start_time = time.time()
            while time.time() - start_time < self.shutdown_timeout:
                if process.poll() is not None:
                    self.get_logger().info(f'{process_name} 及其子进程已关闭')
                    del self.processes[process_name]
                    return True
                time.sleep(0.5)

            self.get_logger().warning(f'{process_name} 未能优雅终止，正在强制终止')
            os.killpg(pgid, signal.SIGKILL)
            process.wait()

            del self.processes[process_name]
            return True

        except Exception as e:
            self.get_logger().error(f'关闭 {process_name} 失败: {e}')
            return False

    def is_process_running(self, process_name: str) -> bool:
        """检查进程是否正在运行"""
        if process_name not in self.processes:
            return False

        process = self.processes[process_name]
        return process.poll() is None

def main(args=None):
    rclpy.init(args=args)
    node = ProcessManagerNode()

    # 使用多线程executor以支持并发服务调用
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 清理所有进程
        node.get_logger().info('关闭所有进程...')
        for process_name in list(node.processes.keys()):
            node.shutdown_process(process_name)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
