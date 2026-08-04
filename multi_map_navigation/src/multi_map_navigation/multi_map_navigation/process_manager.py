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
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
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
        self.max_startup_time = pm_params.get('max_startup_time', 60.0)
        self.health_check_interval = pm_params.get('health_check_interval', 1.0)
        self.health_failure_threshold = pm_params.get('health_failure_threshold', 3)
        self.lifecycle_query_timeout = pm_params.get('lifecycle_query_timeout', 1.0)
        self.shutdown_timeout = pm_params.get('shutdown_timeout', 10.0)

        self.required_process_patterns = {
            'navigation2': tuple(pm_params.get(
                'navigation2_required_processes',
                ['component_container_isolated'])),
            'liosam': tuple(pm_params.get(
                'liosam_required_processes',
                [
                    'lio_sam_imuPreintegration',
                    'lio_sam_imageProjection',
                    'lio_sam_featureExtraction',
                    'lio_sam_mapOptimization',
                ])),
        }
        self.required_nodes = {
            'navigation2': tuple(pm_params.get(
                'navigation2_required_nodes',
                [
                    '/nav2_container',
                    '/controller_server',
                    '/smoother_server',
                    '/planner_server',
                    '/behavior_server',
                    '/bt_navigator',
                    '/waypoint_follower',
                    '/velocity_smoother',
                    '/map_server',
                    '/lifecycle_manager_navigation',
                    '/lifecycle_manager_localization',
                ])),
            'liosam': tuple(pm_params.get(
                'liosam_required_nodes',
                [
                    '/lio_sam_imuPreintegration',
                    '/lio_sam_imageProjection',
                    '/lio_sam_featureExtraction',
                    '/lio_sam_mapOptimization',
                ])),
        }
        self.required_services = {
            'navigation2': tuple(pm_params.get(
                'navigation2_required_services',
                [
                    '/navigate_to_pose/_action/send_goal',
                    '/follow_waypoints/_action/send_goal',
                ])),
        }
        self.required_lifecycle_nodes = tuple(pm_params.get(
            'navigation2_required_lifecycle_nodes',
            [
                '/map_server',
                '/controller_server',
                '/smoother_server',
                '/planner_server',
                '/behavior_server',
                '/bt_navigator',
                '/waypoint_follower',
                '/velocity_smoother',
            ]))

        map_params = config.get('maps', {}).get('ros__parameters', {})
        self.map_dir = map_params.get('map_directory', "/home/akun/workspace/Car_jetson/multi_map_navigation/src/map")

        # 进程跟踪
        self.processes = {}
        self.process_states = {}
        self.health_failures = {}
        self.fault_reasons = {}

        self.lifecycle_callback_group = ReentrantCallbackGroup()
        self.lifecycle_clients = {
            node_name: self.create_client(
                GetState,
                f'{node_name}/get_state',
                callback_group=self.lifecycle_callback_group
            )
            for node_name in self.required_lifecycle_nodes
        }

        self.fault_pub = self.create_publisher(
            String,
            '/process_manager/fault',
            10
        )

        # 创建服务
        self.start_process_srv = self.create_service(
            StartProcess,
            '/process_manager/start_process',
            self.start_process_callback
        )

        self.shutdown_process_srv = self.create_service(
            ShutdownProcess,
            '/process_manager/shutdown_process',
            self.shutdown_process_callback
        )

        self.get_process_status_srv = self.create_service(
            GetProcessStatus,
            '/process_manager/get_status',
            self.get_process_status_callback
        )

        self.health_timer = self.create_timer(
            self.health_check_interval,
            self.health_check_callback
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

        failed = [
            f'{name}: {reason}' for name, reason in self.fault_reasons.items()
            if self.process_states.get(name) == 'FAILED'
        ]
        response.message = '; '.join(failed) if failed else '进程状态查询成功'

        return response

    def launch_relocalization(self, map_name: str) -> bool:
        """启动重定位服务进程"""
        try:
            self.get_logger().info(f'正在为地图启动重定位: {map_name}')

            cmd = [
                'ros2', 'launch',
                're_localization',
                'run.real_launch.py',
                f'map_path:={self.map_dir}/{map_name}/{map_name}_raw.pcd'
            ]

            process = subprocess.Popen(
                cmd,
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            self.processes['re_localization'] = process
            self._mark_starting('re_localization')
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
                'ros2', 'run',
                'nav2_init_pose',
                'nav2_init_pose',
                '--ros-args',
                '-p', f'map_csv:={self.map_dir}/{map_name}/{map_name}.csv',
                '--log-level', 'info'
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
            self._mark_starting('nav2_init_pose')
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
                'ros2', 'launch',
                'lio_sam',
                'run.real_launch.py'
            ]

            process = subprocess.Popen(
                cmd,
                start_new_session=True
            )

            self.processes['liosam'] = process
            self._mark_starting('liosam')

            if self.wait_for_process_healthy('liosam'):
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
                'ros2', 'launch',
                'nav2',
                'run.real_launch.py',
                f'map:={self.map_dir}/{map_name}/{map_name}_clean.yaml'
            ]

            # 【调试插桩】
            # ==========================================
            # 【新增逻辑】：创建一个专属的日志文件来捕捉 Nav2 的输出
            log_file_path = "/home/akun/workspace/Car_jetson/multi_map_navigation/nav2.log"
            self.nav2_log_file = open(log_file_path, 'w')
            self.get_logger().info(f'Nav2 的详细调试日志将保存在: {log_file_path}')
            # 使用 stdbuf 强制关闭 Linux 缓冲，保证日志实时写入文件
            # 如果不加 stdbuf，ROS 2 的日志可能会憋在内存里迟迟不写入文件
            stdbuf_cmd = ['stdbuf', '-o0', '-e0'] + cmd
            process = subprocess.Popen(
                stdbuf_cmd,               # 使用去缓冲的命令
                start_new_session=True,
                stdout=self.nav2_log_file, # 标准输出重定向到文件
                stderr=subprocess.STDOUT   # 错误输出合并到标准输出，一起存入文件
            )
            # ==========================================

            # process = subprocess.Popen(
            #     cmd,
            #     start_new_session=True
            # )

            self.processes['navigation2'] = process
            self._mark_starting('navigation2')

            if self.wait_for_process_healthy('navigation2'):
                self.get_logger().info('Navigation2启动成功')
                return True
            else:
                self.get_logger().error('Navigation2启动失败')
                return False

        except Exception as e:
            self.get_logger().error(f'启动Navigation2失败: {e}')
            return False

    def shutdown_process(self, process_name: str, preserve_fault: bool = False) -> bool:
        """优雅地关闭特定进程"""
        if process_name not in self.processes:
            self.get_logger().warning(f'未找到进程 {process_name}')
            return True

        process = self.processes[process_name]
        self.process_states[process_name] = 'STOPPING'

        try:
            self.get_logger().info(f'正在关闭进程组: {process_name}')

            pgid = os.getpgid(process.pid)
            os.killpg(pgid, signal.SIGTERM)

            start_time = time.time()
            while time.time() - start_time < self.shutdown_timeout:
                if process.poll() is not None:
                    self.get_logger().info(f'{process_name} 及其子进程已关闭')
                    self._forget_process(process_name, preserve_fault)
                    return True
                time.sleep(0.5)

            self.get_logger().warning(f'{process_name} 未能优雅终止，正在强制终止')
            os.killpg(pgid, signal.SIGKILL)
            process.wait()

            self._forget_process(process_name, preserve_fault)
            return True

        except ProcessLookupError:
            self._forget_process(process_name, preserve_fault)
            return True
        except Exception as e:
            self.get_logger().error(f'关闭 {process_name} 失败: {e}')
            return False

    def is_process_running(self, process_name: str) -> bool:
        """Return the last confirmed health, tolerating brief graph discovery loss."""
        if process_name not in self.processes:
            return False

        healthy, reason, immediate_failure = self.probe_process_health(process_name)
        self.update_process_health(process_name, healthy, reason, immediate_failure)
        return self.process_states.get(process_name) in ('HEALTHY', 'DEGRADED')

    def _mark_starting(self, process_name: str):
        self.process_states[process_name] = 'STARTING'
        self.health_failures[process_name] = 0
        self.fault_reasons.pop(process_name, None)

    def _forget_process(self, process_name: str, preserve_fault: bool = False):
        self.processes.pop(process_name, None)
        if preserve_fault:
            self.process_states[process_name] = 'FAILED'
        else:
            self.process_states[process_name] = 'STOPPED'
            self.health_failures.pop(process_name, None)
            self.fault_reasons.pop(process_name, None)

    def _running_process_commands(self, process) -> list:
        commands = []
        try:
            root = psutil.Process(process.pid)
            for candidate in [root] + root.children(recursive=True):
                if not candidate.is_running() or candidate.status() == psutil.STATUS_ZOMBIE:
                    continue
                try:
                    commands.append(' '.join(candidate.cmdline()))
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return []
        return commands

    def _current_node_names(self) -> set:
        return {
            f'{namespace.rstrip("/")}/{name}' if namespace != '/' else f'/{name}'
            for name, namespace in self.get_node_names_and_namespaces()
        }

    def _current_service_names(self) -> set:
        return {name for name, _ in self.get_service_names_and_types()}

    def _lifecycle_nodes_active(self):
        clients = getattr(self, 'lifecycle_clients', {})
        if not clients:
            return True, ''

        unavailable = [
            node_name for node_name, client in clients.items()
            if not client.service_is_ready()
        ]
        if unavailable:
            return False, f'lifecycle services unavailable: {", ".join(unavailable)}'

        futures = {}
        for node_name, client in clients.items():
            try:
                futures[node_name] = client.call_async(GetState.Request())
            except Exception as exc:
                return False, f'lifecycle query failed for {node_name}: {exc}'

        deadline = time.monotonic() + self.lifecycle_query_timeout
        while time.monotonic() < deadline and not all(
            future.done() for future in futures.values()
        ):
            time.sleep(0.01)

        inactive = []
        for node_name, future in futures.items():
            if not future.done():
                future.cancel()
                inactive.append(f'{node_name}=timeout')
                continue
            try:
                result = future.result()
            except Exception as exc:
                inactive.append(f'{node_name}=error({exc})')
                continue
            if result is None or result.current_state.id != State.PRIMARY_STATE_ACTIVE:
                label = 'no response' if result is None else result.current_state.label
                inactive.append(f'{node_name}={label}')

        if inactive:
            return False, f'lifecycle nodes not active: {", ".join(inactive)}'
        return True, ''

    def probe_process_health(self, process_name: str):
        """Check the launch process, required child executables, and ROS nodes."""
        process = self.processes.get(process_name)
        if process is None:
            return False, 'launch process is not tracked', True
        if process.poll() is not None:
            return False, f'launch process exited with code {process.returncode}', True

        commands = self._running_process_commands(process)
        missing_processes = [
            pattern for pattern in self.required_process_patterns.get(process_name, ())
            if not any(pattern in command for command in commands)
        ]
        if missing_processes:
            return False, f'missing child processes: {", ".join(missing_processes)}', False

        required_nodes = self.required_nodes.get(process_name, ())
        if required_nodes:
            current_nodes = self._current_node_names()
            missing_nodes = [node for node in required_nodes if node not in current_nodes]
            if missing_nodes:
                return False, f'missing ROS nodes: {", ".join(missing_nodes)}', False

        required_services = getattr(self, 'required_services', {}).get(process_name, ())
        if required_services:
            current_services = self._current_service_names()
            missing_services = [
                service for service in required_services if service not in current_services
            ]
            if missing_services:
                return False, f'missing ROS services: {", ".join(missing_services)}', False

        if process_name == 'navigation2':
            lifecycle_active, lifecycle_reason = self._lifecycle_nodes_active()
            if not lifecycle_active:
                return False, lifecycle_reason, False

        return True, '', False

    def update_process_health(
        self, process_name: str, healthy: bool, reason: str,
        immediate_failure: bool = False
    ):
        previous_state = self.process_states.get(process_name, 'STOPPED')
        if previous_state in ('STOPPING', 'STOPPED'):
            return

        if healthy:
            self.process_states[process_name] = 'HEALTHY'
            self.health_failures[process_name] = 0
            self.fault_reasons.pop(process_name, None)
            return

        failures = self.health_failures.get(process_name, 0) + 1
        self.health_failures[process_name] = failures
        confirmed = immediate_failure or failures >= self.health_failure_threshold
        if not confirmed:
            self.process_states[process_name] = 'DEGRADED'
            return

        self.process_states[process_name] = 'FAILED'
        self.fault_reasons[process_name] = reason
        if previous_state != 'FAILED':
            self.get_logger().error(f'{process_name}健康检查失败: {reason}')
            message = String()
            message.data = f'{process_name}: {reason}'
            self.fault_pub.publish(message)

    def health_check_callback(self):
        for process_name in list(self.processes):
            if self.process_states.get(process_name) in ('STARTING', 'STOPPING', 'STOPPED'):
                continue
            healthy, reason, immediate = self.probe_process_health(process_name)
            self.update_process_health(process_name, healthy, reason, immediate)
            if self.process_states.get(process_name) == 'FAILED':
                self.shutdown_process(process_name, preserve_fault=True)

    def wait_for_process_healthy(self, process_name: str) -> bool:
        deadline = time.monotonic() + self.max_startup_time
        while time.monotonic() < deadline:
            healthy, reason, immediate = self.probe_process_health(process_name)
            if healthy:
                self.update_process_health(process_name, True, '')
                return True
            if immediate:
                self.update_process_health(process_name, False, reason, True)
                return False
            time.sleep(0.25)

        healthy, reason, immediate = self.probe_process_health(process_name)
        if healthy:
            self.update_process_health(process_name, True, '')
            return True
        timeout_reason = f'startup health check timed out: {reason}'
        self.update_process_health(process_name, False, timeout_reason, True)
        return False

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
        if rclpy.ok():
            node.get_logger().info('关闭所有进程...')
        for process_name in list(node.processes.keys()):
            node.shutdown_process(process_name)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
