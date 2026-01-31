#!/usr/bin/env python3
"""
进程管理器
开发者B - 导航模块

管理ROS2启动进程的生命周期(重定位、LIO-SAM、Navigation2)
"""

import subprocess
import os
import time
import signal
import psutil
import logging
from typing import Optional, Dict
import yaml

class ProcessManager:
    def __init__(self):
        """
        初始化进程管理器

        参数:
            startup_delay: 进程启动后的等待时间（秒）
            shutdown_timeout: 进程关闭超时时间（秒）
        """

        with open("/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/config/navigation_config.yaml") as f:
            config = yaml.safe_load(f)
            
        pm_params = config.get('process_manager', {}).get('ros__parameters', {})
        self.startup_delay = pm_params.get('startup_delay', 2.0)
        self.max_startup_time = pm_params.get('max_startup_time', 15.0)
        self.health_check_interval = pm_params.get('health_check_interval', 1.0)
        self.shutdown_timeout = pm_params.get('shutdown_timeout', 10.0)
        
        map_params = config.get('maps', {}).get('ros__parameters', {})
        self.map_dir = map_params.get('map_directory', "/home/akun/workspace/Car_jetson/multi_map_navigation/src/map")
        

        # 进程跟踪
        self.processes: Dict[str, subprocess.Popen] = {}

        # 配置日志
        self.logger = logging.getLogger(__name__)

        self.logger.info('进程管理器已初始化')

    def launch_relocalization(self, map_name: str) -> bool:
        """
        启动重定位服务进程

        参数:
            map_name: 用于重定位的地图名称

        返回:
            启动成功返回True，否则返回False
        """
        try:
            self.logger.info(f'正在为地图启动重定位: {map_name}')

            # 构建启动命令
            cmd = [
                'ros2', 'launch',
                're_localization',
                'run.real_launch.py',
                f'map_path:={self.map_dir}/{map_name}/{map_name}_clean.pcd'
            ]

            # 启动进程
            process = subprocess.Popen(
                cmd,
                start_new_session=True,  # 创建新的进程组
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            self.processes['re_localization'] = process

            # 等待启动
            time.sleep(self.startup_delay)

            # 检查进程是否正在运行
            if self.is_process_running('re_localization'):
                self.logger.info('重定位服务启动成功')
                return True
            else:
                self.logger.error('重定位服务启动失败')
                return False

        except Exception as e:
            self.logger.error(f'启动重定位失败: {e}')
            return False

    def launch_nav2_init_pose(self, map_name: str) -> bool:
        """
        启动 nav2_init_pose 进程

        参数:
            map_name: 地图名称（用于获取CSV文件）

        返回:
            启动成功返回True，否则返回False
        """
        try:
            # 构建启动命令
            cmd = [
                'ros2', 'run',
                'nav2_init_pose',
                'nav2_init_pose',
                '--ros-args',
                '-p', f'map_csv:={self.map_dir}/{map_name}/{map_name}.csv',
                '--log-level', 'info'  # 确保日志级别足够
            ]

            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"                 # 让 print() 立即输出
            env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"  # 让 ROS2 日志立即输出
            env["RCUTILS_LOGGING_USE_STDOUT"] = "1"       # 强制使用 stdout 而不是 stderr

            # 启动进程
            process = subprocess.Popen(
                cmd,
                start_new_session=True,  # 创建新的进程组
                env=env,            # <--- 注入环境变量
                stdout=None,        # 继承父进程 stdout (默认行为)
                stderr=None         # 继承父进程 stderr
            )

            self.processes['nav2_init_pose'] = process

            # 等待启动
            time.sleep(self.startup_delay)

            # 检查进程是否正在运行
            if self.is_process_running('nav2_init_pose'):
                self.logger.info('nav2_init_pose启动成功')
                return True
            else:
                self.logger.error('nav2_init_pose启动失败')
                return False

        except Exception as e:
            self.logger.error(f'启动nav2_init_pose失败: {e}')
            return False

    def launch_liosam(self) -> bool:
        """
        启动LIO-SAM进程

        返回:
            启动成功返回True，否则返回False
        """
        try:
            self.logger.info('正在启动LIO-SAM')

            # 构建启动命令
            cmd = [
                'ros2', 'launch',
                'lio_sam',
                'run.real_launch.py'
            ]

            # 启动进程
            process = subprocess.Popen(
                cmd,
                start_new_session=True  # 创建新的进程组
            )

            self.processes['liosam'] = process

            # 等待启动
            time.sleep(self.startup_delay)

            # 检查进程是否正在运行
            if self.is_process_running('liosam'):
                self.logger.info('LIO-SAM启动成功')
                return True
            else:
                self.logger.error('LIO-SAM启动失败')
                return False

        except Exception as e:
            self.logger.error(f'启动LIO-SAM失败: {e}')
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
            self.logger.info(f'正在为地图启动Navigation2: {map_name}')

            # 构建启动命令
            cmd = [
                'ros2', 'launch',
                'nav2',
                'run.real_launch.py',
                f'map:={self.map_dir}/{map_name}/{map_name}_clean.yaml'
            ]

            # 启动进程
            process = subprocess.Popen(
                cmd,
                start_new_session=True  # 创建新的进程组
            )

            self.processes['navigation2'] = process

            # 等待启动
            time.sleep(self.startup_delay)

            # 检查进程是否正在运行
            if self.is_process_running('navigation2'):
                self.logger.info('Navigation2启动成功')
                return True
            else:
                self.logger.error('Navigation2启动失败')
                return False

        except Exception as e:
            self.logger.error(f'启动Navigation2失败: {e}')
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
            self.logger.warning(f'未找到进程 {process_name}')
            return True

        process = self.processes[process_name]

        try:
            self.logger.info(f'正在关闭进程组: {process_name}')

            # 获取进程组ID（PGID）
            pgid = os.getpgid(process.pid)

            # 向整个进程组发送 SIGTERM
            os.killpg(pgid, signal.SIGTERM)

            # 等待进程终止
            start_time = time.time()
            while time.time() - start_time < self.shutdown_timeout:
                if process.poll() is not None:
                    self.logger.info(f'{process_name} 及其子进程已关闭')
                    del self.processes[process_name]
                    return True
                time.sleep(0.5)

            # 如果仍在运行则强制终止
            self.logger.warning(f'{process_name} 未能优雅终止，正在强制终止')
            os.killpg(pgid, signal.SIGKILL)
            process.wait()

            del self.processes[process_name]
            return True

        except Exception as e:
            self.logger.error(f'关闭 {process_name} 失败: {e}')
            return False

    def shutdown_all_processes(self) -> bool:
        """
        关闭所有托管进程

        返回:
            所有进程关闭成功返回True，否则返回False
        """
        self.logger.info('正在关闭所有进程')

        # 按相反顺序关闭: navigation2 -> liosam -> nav2_init_pose -> re_localization
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
                self.logger.info('所有进程已完全关闭')
                return True
            time.sleep(0.5)

        self.logger.warning('等待完全关闭超时')
        return False

    def cleanup(self):
        """清理资源"""
        self.shutdown_all_processes()


