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
import threading
import uuid
import math
import fcntl
import psutil
import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from multi_map_navigation_msgs.msg import (
    LioCorrectionEvent,
    LioReadiness,
    LocalizationSession,
)
from multi_map_navigation_msgs.srv import StartProcess, ShutdownProcess, GetProcessStatus
from re_localization.srv import GetLocalizationReadiness
import yaml


GUI_ENVIRONMENT_KEYS = (
    'DISPLAY',
    'WAYLAND_DISPLAY',
    'XAUTHORITY',
    'XDG_RUNTIME_DIR',
    'DBUS_SESSION_BUS_ADDRESS',
)


class ProcessManagerNode(Node):
    def __init__(self):
        super().__init__('process_manager')
        self._acquire_manager_instance_lock()
        self.external_process_guard_enabled = True

        # 加载配置
        self.declare_parameter('config_file', '/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/config/navigation_config.yaml')
        config_file = self.get_parameter('config_file').value

        with open(config_file) as f:
            config = yaml.safe_load(f)

        pm_params = config.get('process_manager', {}).get('ros__parameters', {})
        self.startup_delay = pm_params.get('startup_delay', 2.0)
        self.max_startup_time = pm_params.get('max_startup_time', 60.0)
        self.nav2_max_startup_time = pm_params.get(
            'nav2_max_startup_time', 150.0)
        self.health_check_interval = pm_params.get('health_check_interval', 1.0)
        self.health_failure_threshold = pm_params.get('health_failure_threshold', 3)
        self.liosam_recovery_grace_sec = max(
            0.0, float(pm_params.get('liosam_recovery_grace_sec', 5.0)))
        self.liosam_stable_correction_count_required = max(
            1, int(pm_params.get('liosam_stable_correction_count', 10)))
        self.liosam_stable_translation_m = max(
            0.0, float(pm_params.get(
                'liosam_stable_translation_m', 0.05)))
        self.liosam_stable_yaw_rad = math.radians(max(
            0.0, float(pm_params.get('liosam_stable_yaw_deg', 1.0))))
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
        # A ros2 launch process starts a separate process group.  Keep the
        # group id captured at Popen time because the launch parent can exit
        # before one of its children.  Looking the pgid up from the parent at
        # shutdown time is therefore too late.
        self.process_pgids = {}
        self.process_maps = {}
        self.process_started_at = {}
        self.process_generations = {}
        self.process_instance_ids = {}
        self.process_cancel_events = {}
        self.process_dependencies = {}
        self._lifecycle_generation_counter = 0
        self.process_lifecycle_lock = threading.RLock()
        self.start_operation_lock = threading.Lock()
        self._shutdown_in_progress = set()
        self._shutdown_operations = {}
        self._manager_shutting_down = False
        self.active_localization_session = ''
        self.active_map = ''
        self.localization_session_start_ns = 0
        self.process_states = {}
        self.health_failures = {}
        self.fault_reasons = {}
        self.confirmed_healthy_processes = set()
        self.liosam_ready = False
        self.liosam_ever_ready = False
        self.liosam_not_ready_since = None
        self.liosam_stable_correction_count = 0
        self.liosam_stability_ready = False
        self.liosam_last_correction_epoch = 0

        # One reliable writer publishes readiness transitions in order. Keep
        # this callback serialized so executor scheduling cannot apply an older
        # False after a newer True from the same LIO instance.
        self.liosam_ready_callback_group = MutuallyExclusiveCallbackGroup()
        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.liosam_ready_sub = self.create_subscription(
            LioReadiness,
            '/lio_sam/imu_preintegration_readiness',
            self.liosam_ready_callback,
            ready_qos,
            callback_group=self.liosam_ready_callback_group,
        )
        correction_qos = QoSProfile(depth=100)
        correction_qos.reliability = ReliabilityPolicy.RELIABLE
        self.liosam_correction_sub = self.create_subscription(
            LioCorrectionEvent,
            '/lio_sam/correction_event',
            self.liosam_correction_callback,
            correction_qos,
            callback_group=self.liosam_ready_callback_group,
        )

        self.lifecycle_callback_group = ReentrantCallbackGroup()
        self.lifecycle_clients = {
            node_name: self.create_client(
                GetState,
                f'{node_name}/get_state',
                callback_group=self.lifecycle_callback_group
            )
            for node_name in self.required_lifecycle_nodes
        }
        self.relocalization_readiness_client = self.create_client(
            GetLocalizationReadiness, '/pcl_localization/get_readiness',
            callback_group=self.lifecycle_callback_group)
        self.relocalization_state_client = self.create_client(
            GetState, '/pcl_localization/get_state',
            callback_group=self.lifecycle_callback_group)
        self.relocalization_change_client = self.create_client(
            ChangeState, '/pcl_localization/change_state',
            callback_group=self.lifecycle_callback_group)

        self.fault_pub = self.create_publisher(
            String,
            '/process_manager/fault',
            10
        )

        session_qos = QoSProfile(depth=1)
        session_qos.reliability = ReliabilityPolicy.RELIABLE
        session_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.localization_session_pub = self.create_publisher(
            LocalizationSession,
            '/localization_session',
            session_qos,
        )

        # Long startup health checks run inside the start service callback.
        # A non-blocking operation lock serializes starts without leaving later
        # requests queued; status and stop remain responsive in another group.
        self.start_callback_group = ReentrantCallbackGroup()
        self.shutdown_callback_group = ReentrantCallbackGroup()
        self.status_callback_group = ReentrantCallbackGroup()
        self.health_callback_group = MutuallyExclusiveCallbackGroup()

        # 创建服务
        self.start_process_srv = self.create_service(
            StartProcess,
            '/process_manager/start_process',
            self.start_process_callback,
            callback_group=self.start_callback_group,
        )

        self.shutdown_process_srv = self.create_service(
            ShutdownProcess,
            '/process_manager/shutdown_process',
            self.shutdown_process_callback,
            callback_group=self.shutdown_callback_group,
        )

        self.get_process_status_srv = self.create_service(
            GetProcessStatus,
            '/process_manager/get_status',
            self.get_process_status_callback,
            callback_group=self.status_callback_group,
        )

        self.health_timer = self.create_timer(
            self.health_check_interval,
            self.health_check_callback,
            callback_group=self.health_callback_group,
        )

        self.get_logger().info('进程管理器节点已初始化')

    def _acquire_manager_instance_lock(self):
        """Allow only one process manager to own navigation subprocesses."""
        lock_path = '/tmp/multi_map_navigation_process_manager.lock'
        lock_file = open(lock_path, 'a+', encoding='utf-8')
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            lock_file.close()
            raise RuntimeError(
                '另一个 process_manager 已持有导航进程所有权锁') from exc
        lock_file.seek(0)
        lock_file.truncate()
        lock_file.write(f'{os.getpid()}\n')
        lock_file.flush()
        self._manager_lock_file = lock_file

    def liosam_ready_callback(self, message: LioReadiness):
        ready = bool(message.ready)
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if ('liosam' not in self.processes or
                    self.process_states.get('liosam') in
                    ('STOPPING', 'STOPPED', 'FAILED')):
                return
            expected_instance_id = self.process_instance_ids.get('liosam')
            expected_generation = self.process_generations.get('liosam')
            if (not expected_instance_id or
                    message.instance_id != expected_instance_id or
                    message.generation != expected_generation):
                self.get_logger().warning(
                    '忽略非当前LIO-SAM实例的就绪消息: '
                    f'instance={message.instance_id or "<empty>"}, '
                    f'generation={message.generation}, '
                    f'expected_instance={expected_instance_id or "<empty>"}, '
                    f'expected_generation={expected_generation}')
                return
            if ready:
                self.liosam_ever_ready = True
                self.liosam_not_ready_since = None
            elif (getattr(self, 'liosam_ever_ready', False) and
                  getattr(self, 'liosam_not_ready_since', None) is None):
                self.liosam_not_ready_since = time.monotonic()
            if not ready:
                self.liosam_stable_correction_count = 0
                self.liosam_stability_ready = False
                self.liosam_last_correction_epoch = 0
            self.liosam_ready = ready

    def liosam_correction_callback(self, message: LioCorrectionEvent):
        """Require consecutive trustworthy scan corrections before Nav2 starts."""
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if ('liosam' not in self.processes or
                    self.process_states.get('liosam') in
                    ('STOPPING', 'STOPPED', 'FAILED')):
                return
            expected_instance_id = self.process_instance_ids.get('liosam')
            expected_generation = self.process_generations.get('liosam')
            if (not expected_instance_id or
                    message.instance_id != expected_instance_id or
                    int(message.generation) != expected_generation):
                return
            if message.source != LioCorrectionEvent.SOURCE_SCAN_TO_MAP:
                return
            epoch = int(message.correction_epoch)
            if epoch <= self.liosam_last_correction_epoch:
                return
            self.liosam_last_correction_epoch = epoch

            # Runtime localization faults remain the responsibility of the
            # correction/fault path. Once startup stability is proven, a
            # single later medium correction must not make process health flap.
            if self.liosam_stability_ready:
                return

            stable = (
                bool(message.accepted)
                and not bool(message.degenerate)
                and abs(float(message.translation_residual)) <=
                self.liosam_stable_translation_m
                and abs(float(message.yaw_residual)) <=
                self.liosam_stable_yaw_rad
            )
            if not stable:
                self.liosam_stable_correction_count = 0
                return

            self.liosam_stable_correction_count += 1
            if (self.liosam_stable_correction_count >=
                    self.liosam_stable_correction_count_required):
                self.liosam_stability_ready = True
                self.get_logger().info(
                    'LIO-SAM扫描匹配稳定门槛已满足: '
                    f'{self.liosam_stable_correction_count}/'
                    f'{self.liosam_stable_correction_count_required}')

    def _populate_existing_start_response(
        self, process_name: str, map_name: str, response
    ) -> bool:
        """Return an idempotent result for an already tracked component."""
        with self.process_lifecycle_lock:
            if (process_name not in self.processes and
                    process_name not in self.process_generations):
                return False

            state = self.process_states.get(process_name)
            if process_name in self._shutdown_in_progress:
                response.success = False
                response.busy = True
                response.message = f'{process_name}正在关闭'
                return True
            if state == 'FAILED':
                response.success = False
                response.busy = False
                response.message = self.fault_reasons.get(
                    process_name, f'{process_name}处于FAILED状态')
                return True

            tracked_map = self.process_maps.get(process_name, '')
            compatible = (
                process_name == 'liosam' or
                tracked_map == (map_name or '')
            )
            if process_name == 're_localization':
                compatible = (
                    compatible and
                    self.active_map == map_name and
                    bool(self.active_localization_session) and
                    self.localization_session_start_ns > 0
                )

            dependency_valid = True
            if process_name == 'nav2_init_pose':
                dependency = self.process_dependencies.get(process_name)
                dependency_valid = (
                    dependency is not None and
                    len(dependency) == 6 and
                    dependency[0] == 're_localization' and
                    self._localization_dependency_matches_locked(
                        *dependency[1:])
                )
            elif process_name == 'navigation2':
                dependency = self.process_dependencies.get(process_name)
                dependency_valid = (
                    dependency is not None and
                    len(dependency) == 3 and
                    dependency[0] == 'liosam' and
                    self.processes.get('liosam') is dependency[1] and
                    self.process_generations.get('liosam') == dependency[2] and
                    self.process_cancel_events.get('liosam') is not None and
                    not self.process_cancel_events['liosam'].is_set() and
                    self.process_states.get('liosam') == 'HEALTHY'
                )

            if not dependency_valid:
                response.success = False
                response.busy = False
                response.message = (
                    f'{process_name}绑定的依赖实例已停止或被替换')
                return True

            if not compatible and state in ('HEALTHY', 'DEGRADED'):
                response.success = False
                response.busy = False
                response.message = (
                    f'{process_name}已绑定其他地图: '
                    f'active={tracked_map}, requested={map_name or ""}')
                return True

            if state == 'HEALTHY' and compatible:
                response.success = True
                response.busy = False
                response.message = f'{process_name}已由相同地图的事务启动'
                if process_name == 're_localization':
                    response.instance_id = self.active_localization_session
                    response.active_map = self.active_map
                    response.session_start_ns = (
                        self.localization_session_start_ns)
                return True

            response.success = False
            response.busy = True
            if compatible:
                response.message = (
                    f'{process_name}正在处理，当前状态为{state or "UNKNOWN"}')
            else:
                response.message = (
                    f'{process_name}正在处理其他地图事务: '
                    f'active={tracked_map}, requested={map_name or ""}, '
                    f'state={state or "UNKNOWN"}')
            return True

    def start_process_callback(self, request, response):
        """
        启动进程服务回调
        """
        process_name = request.process_name
        map_name = request.map_name
        response.instance_id = ''
        response.active_map = ''
        response.session_start_ns = 0
        response.busy = False

        self.get_logger().info(f'收到启动进程请求: {process_name}, map: {map_name}')

        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if self._manager_shutting_down:
                response.success = False
                response.busy = True
                response.message = '进程管理器正在关闭'
                return response
        if not self.start_operation_lock.acquire(blocking=False):
            response.success = False
            response.busy = True
            response.message = '另一个导航子进程启动请求正在处理'
            return response

        known_processes = {
            're_localization', 'nav2_init_pose', 'liosam', 'navigation2'}
        admitted_generation = None
        try:
            with self.process_lifecycle_lock:
                if self._manager_shutting_down:
                    response.success = False
                    response.busy = True
                    response.message = '进程管理器正在关闭'
                    return response
            if process_name in known_processes:
                admitted_generation = self._begin_start_admission(
                    process_name)
                if admitted_generation is None:
                    if not self._populate_existing_start_response(
                            process_name, map_name, response):
                        response.success = False
                        response.busy = True
                        response.message = (
                            f'{process_name}启动许可暂不可用')
                    return response

            if process_name == 're_localization':
                success = self.launch_relocalization(map_name)
                with self.process_lifecycle_lock:
                    response.success = (
                        success and self.process_states.get(process_name) == 'HEALTHY')
                    if response.success:
                        response.message = '重定位启动成功'
                        response.instance_id = self.active_localization_session
                        response.active_map = self.active_map
                        response.session_start_ns = self.localization_session_start_ns
                    else:
                        response.message = self.fault_reasons.get(
                            process_name, '重定位启动失败或已被取消')

            elif process_name == 'nav2_init_pose':
                initial_pose = (
                    request.initial_pose if request.use_initial_pose else None)
                success = self.launch_nav2_init_pose(map_name, initial_pose)
                response.success = success
                response.message = 'nav2_init_pose启动成功' if success else 'nav2_init_pose启动失败'
                if success:
                    response.instance_id = self.active_localization_session
                    response.active_map = self.active_map
                    response.session_start_ns = self.localization_session_start_ns

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
        finally:
            if admitted_generation is not None:
                self._finish_start_admission(
                    process_name, admitted_generation)
            self.start_operation_lock.release()

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
        Return a cached lifecycle snapshot without running health probes.

        Health probes can query multiple lifecycle services and take seconds.
        Keeping them out of this callback makes status usable while another
        service callback is waiting for a process to become healthy.
        """
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            tracked = set(self.processes)
            states = dict(self.process_states)
            faults = dict(self.fault_reasons)
            process_maps = dict(self.process_maps)
            active_map = self.active_map
            session_id = self.active_localization_session
            session_start_ns = self.localization_session_start_ns

        response.re_localization_running = 're_localization' in tracked
        self.get_logger().debug(f're_localization运行状态: {response.re_localization_running}')

        response.liosam_running = 'liosam' in tracked
        self.get_logger().debug(f'liosam运行状态: {response.liosam_running}')

        response.nav2_init_pose_running = 'nav2_init_pose' in tracked
        self.get_logger().debug(f'nav2_init_pose运行状态: {response.nav2_init_pose_running}')

        response.navigation2_running = 'navigation2' in tracked
        self.get_logger().debug(f'navigation2运行状态: {response.navigation2_running}')

        required_names = (
            're_localization',
            'nav2_init_pose',
            'liosam',
            'navigation2',
        )
        failed_names = [
            name for name, state in states.items() if state == 'FAILED']
        details = [
            f'{name}: {faults.get(name, "FAILED (reason unavailable)")}'
            for name in failed_names
        ]
        details.extend(
            f'{name}: {state}' for name, state in states.items()
            if state in ('STARTING', 'STOPPING', 'DEGRADED')
        )
        response.message = '; '.join(details) if details else '进程状态查询成功'
        state_values = set(states.values())
        if failed_names:
            response.stack_state = 'FAILED'
        elif 'STOPPING' in state_values:
            response.stack_state = 'STOPPING'
        elif 'STARTING' in state_values:
            response.stack_state = 'STARTING'
        elif (all(name in tracked for name in required_names) and
              all(states.get(name) == 'HEALTHY'
                  for name in required_names)):
            response.stack_state = 'RUNNING'
        elif not any(name in tracked for name in required_names):
            response.stack_state = 'STOPPED'
        else:
            response.stack_state = 'PARTIAL'
        response.active_map = (
            process_maps.get('navigation2')
            or active_map
            or '')
        response.localization_session_id = session_id
        response.session_start_ns = session_start_ns

        return response

    def _ensure_process_tracking(self):
        """Initialize lifecycle fields for nodes constructed with ``__new__``.

        Several unit tests intentionally bypass ``Node.__init__``.  Keeping
        the compatibility initialization here also makes lifecycle helpers
        safe to call while recovering an older in-memory manager instance.
        """
        if not hasattr(self, 'processes'):
            self.processes = {}
        if not hasattr(self, 'process_pgids'):
            self.process_pgids = {}
        if not hasattr(self, 'process_maps'):
            self.process_maps = {}
        if not hasattr(self, 'process_started_at'):
            self.process_started_at = {}
        if not hasattr(self, 'process_generations'):
            self.process_generations = {}
        if not hasattr(self, 'process_instance_ids'):
            self.process_instance_ids = {}
        if not hasattr(self, 'process_cancel_events'):
            self.process_cancel_events = {}
        if not hasattr(self, 'process_dependencies'):
            self.process_dependencies = {}
        if not hasattr(self, '_lifecycle_generation_counter'):
            self._lifecycle_generation_counter = 0
        if not hasattr(self, 'process_lifecycle_lock'):
            self.process_lifecycle_lock = threading.RLock()
        if not hasattr(self, 'start_operation_lock'):
            self.start_operation_lock = threading.Lock()
        if not hasattr(self, '_shutdown_in_progress'):
            self._shutdown_in_progress = set()
        if not hasattr(self, '_shutdown_operations'):
            self._shutdown_operations = {}
        if not hasattr(self, '_manager_shutting_down'):
            self._manager_shutting_down = False
        if not hasattr(self, 'active_localization_session'):
            self.active_localization_session = ''
        if not hasattr(self, 'active_map'):
            self.active_map = ''
        if not hasattr(self, 'localization_session_start_ns'):
            self.localization_session_start_ns = 0
        if not hasattr(self, 'process_states'):
            self.process_states = {}
        if not hasattr(self, 'health_failures'):
            self.health_failures = {}
        if not hasattr(self, 'fault_reasons'):
            self.fault_reasons = {}
        if not hasattr(self, 'confirmed_healthy_processes'):
            self.confirmed_healthy_processes = set()
        if not hasattr(self, 'external_process_guard_enabled'):
            # Unit-test instances created with __new__ opt in explicitly.
            self.external_process_guard_enabled = False
        if not hasattr(self, 'liosam_stable_correction_count_required'):
            self.liosam_stable_correction_count_required = 10
        if not hasattr(self, 'liosam_stable_translation_m'):
            self.liosam_stable_translation_m = 0.05
        if not hasattr(self, 'liosam_stable_yaw_rad'):
            self.liosam_stable_yaw_rad = math.radians(1.0)
        if not hasattr(self, 'liosam_stable_correction_count'):
            self.liosam_stable_correction_count = 0
        if not hasattr(self, 'liosam_stability_ready'):
            self.liosam_stability_ready = False
        if not hasattr(self, 'liosam_last_correction_epoch'):
            self.liosam_last_correction_epoch = 0

    def _instance_matches_locked(
        self, process_name: str, expected_process=None,
        expected_generation=None
    ) -> bool:
        if expected_process is not None:
            if self.processes.get(process_name) is not expected_process:
                return False
        if expected_generation is not None:
            if self.process_generations.get(process_name) != expected_generation:
                return False
        return True

    def _begin_start_admission(self, process_name: str) -> int:
        """Register a cancellable start before any launch preparation."""
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if self._manager_shutting_down:
                return None
            if process_name in self._shutdown_in_progress:
                return None
            if process_name in self.processes:
                return None
            if process_name in self.process_generations:
                return None
            self._lifecycle_generation_counter += 1
            generation = self._lifecycle_generation_counter
            self.process_generations[process_name] = generation
            self.process_cancel_events[process_name] = threading.Event()
            self.process_states[process_name] = 'STARTING'
            self.health_failures[process_name] = 0
            self.fault_reasons.pop(process_name, None)
            return generation

    def begin_shutdown(self):
        """Cancel every admitted start before executor workers are drained."""
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            self._manager_shutting_down = True
            for process_name, cancel_event in self.process_cancel_events.items():
                cancel_event.set()
                if self.process_states.get(process_name) == 'STARTING':
                    self.process_states[process_name] = 'STOPPING'

    def _finish_start_admission(
        self, process_name: str, expected_generation: int
    ):
        """Release an admission token that never became a tracked process."""
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if self.processes.get(process_name) is not None:
                return
            if (self.process_generations.get(process_name) !=
                    expected_generation):
                return
            self.process_generations.pop(process_name, None)
            self.process_instance_ids.pop(process_name, None)
            self.process_cancel_events.pop(process_name, None)
            if self.process_states.get(process_name) in ('STARTING', 'STOPPING'):
                self.process_states[process_name] = 'STOPPED'

    def _generation_for_process(self, process_name: str, process) -> int:
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if self.processes.get(process_name) is not process:
                return None
            return self.process_generations.get(process_name)

    def _tracked_instance(self, process_name: str):
        """Return the current process and generation as one locked snapshot."""
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            return (
                self.processes.get(process_name),
                self.process_generations.get(process_name),
            )

    def _start_was_cancelled(
        self, process_name: str, expected_generation
    ) -> bool:
        if expected_generation is None:
            return False
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if self.process_generations.get(process_name) != expected_generation:
                return True
            cancel_event = self.process_cancel_events.get(process_name)
            return cancel_event is None or cancel_event.is_set()

    def _wait_for_start_poll(
        self, process_name: str, expected_generation, timeout: float
    ) -> bool:
        """Wait for one health poll interval; return false when cancelled."""
        if expected_generation is None:
            time.sleep(timeout)
            return True
        with self.process_lifecycle_lock:
            if self.process_generations.get(process_name) != expected_generation:
                return False
            cancel_event = self.process_cancel_events.get(process_name)
        if cancel_event is None:
            return False
        return not cancel_event.wait(timeout)

    @staticmethod
    def _command_matches_process(process_name: str, command: str) -> bool:
        if process_name == 're_localization':
            return (
                'pcl_localization_node' in command
                or ('ros2 launch re_localization run.real_launch.py'
                    in command)
            )
        if process_name == 'nav2_init_pose':
            return (
                '/nav2_init_pose' in command
                or 'ros2 run nav2_init_pose nav2_init_pose' in command
            )
        if process_name == 'liosam':
            return any(name in command for name in (
                'lio_sam_imuPreintegration',
                'lio_sam_imageProjection',
                'lio_sam_featureExtraction',
                'lio_sam_mapOptimization',
            ))
        if process_name == 'navigation2':
            return (
                '__node:=nav2_container' in command
                or 'ros2 launch nav2 run.real_launch.py' in command
            )
        return False

    def _find_untracked_processes(self, process_name: str) -> list:
        """Find matching live processes outside every group owned here."""
        self._ensure_process_tracking()
        tracked_pgids = {
            pgid for pgid in self.process_pgids.values() if pgid is not None}
        matches = []
        for candidate in psutil.process_iter(['pid', 'cmdline', 'status']):
            try:
                if candidate.info['pid'] == os.getpid():
                    continue
                if candidate.info.get('status') == psutil.STATUS_ZOMBIE:
                    continue
                try:
                    pgid = os.getpgid(candidate.info['pid'])
                except (ProcessLookupError, PermissionError):
                    continue
                if pgid in tracked_pgids:
                    continue
                command = ' '.join(candidate.info.get('cmdline') or ())
                if self._command_matches_process(process_name, command):
                    matches.append((candidate.info['pid'], command))
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return matches

    def _live_process_group_members(self, pgid: int) -> set:
        """Return every non-zombie PID that still belongs to ``pgid``."""
        if pgid is None:
            return set()

        members = set()
        for candidate in psutil.process_iter(['pid', 'status']):
            try:
                pid = candidate.info['pid']
                if os.getpgid(pid) != pgid:
                    continue
                status = candidate.info.get('status') or candidate.status()
                if status != psutil.STATUS_ZOMBIE:
                    members.add(pid)
            except (ProcessLookupError, psutil.NoSuchProcess):
                continue
            except (PermissionError, psutil.AccessDenied):
                # A group member that cannot be inspected must be treated as
                # alive.  Otherwise shutdown could incorrectly report clean.
                try:
                    if os.getpgid(candidate.pid) == pgid:
                        members.add(candidate.pid)
                except (ProcessLookupError, PermissionError):
                    continue
        return members

    @staticmethod
    def _parent_is_running(process) -> bool:
        try:
            return process.poll() is None
        except (AttributeError, OSError):
            # Test doubles and an uninspectable Popen are handled
            # conservatively: never overwrite their tracking entry.
            return True

    def _has_live_tracked_instance(self, process_name: str) -> bool:
        self._ensure_process_tracking()
        process = self.processes.get(process_name)
        if process is None:
            return False

        pgid = self.process_pgids.get(process_name)
        if pgid is not None and self._live_process_group_members(pgid):
            return True
        return self._parent_is_running(process)

    def _capture_process_group(self, process) -> int:
        """Capture the new session's pgid immediately after ``Popen``."""
        pid = getattr(process, 'pid', None)
        if pid is None:
            # Compatibility for lightweight Popen doubles in existing tests.
            return None
        try:
            return os.getpgid(pid)
        except OSError:
            # ``start_new_session=True`` makes pid == pgid.  Retain that id
            # even if a short-lived launch parent exits before this lookup.
            return pid

    def _spawn_tracked_process(
        self, process_name: str, map_name: str, command: list, **popen_kwargs
    ):
        """Start and atomically register one process-group instance."""
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if self._manager_shutting_down:
                self.get_logger().info(
                    f'取消 {process_name} 启动: 进程管理器正在关闭')
                return None
            if process_name in self._shutdown_in_progress:
                self.get_logger().info(
                    f'取消 {process_name} 启动: 当前关闭事务尚未结束')
                return None
            if (process_name in self.processes and
                    self.process_states.get(process_name) == 'STOPPING'):
                self.get_logger().warning(
                    f'拒绝启动 {process_name}: 当前实例仍在关闭')
                return None
            pending_generation = self.process_generations.get(process_name)
            pending_cancel = self.process_cancel_events.get(process_name)
            if (process_name not in self.processes and
                    pending_generation is not None and
                    (pending_cancel is None or pending_cancel.is_set())):
                self.get_logger().info(
                    f'取消 {process_name} 启动: '
                    f'generation={pending_generation}')
                return None

            if self._has_live_tracked_instance(process_name):
                pgid = self.process_pgids.get(process_name)
                active_map = self.process_maps.get(process_name, '')
                self.get_logger().warning(
                    f'拒绝重复启动 {process_name}: '
                    f'已有存活进程组 pgid={pgid}, map={active_map}'
                )
                return None

            if self.external_process_guard_enabled:
                untracked = self._find_untracked_processes(process_name)
                if untracked:
                    pids = [pid for pid, _command in untracked]
                    reason = (
                        f'拒绝启动 {process_name}: 发现未纳管残留进程 '
                        f'pid={pids}，请先执行完整导航栈清理')
                    self.get_logger().error(reason)
                    self._record_start_failure(process_name, reason)
                    return None

            # A previously tracked group is now completely gone.  Remove its
            # active handle before registering a replacement; FAILED state and
            # fault reason are retained until _mark_starting resets the retry.
            if process_name in self.processes:
                self.processes.pop(process_name, None)
                self.process_pgids.pop(process_name, None)
                self.process_maps.pop(process_name, None)
                self.process_started_at.pop(process_name, None)
                self.process_generations.pop(process_name, None)
                self.process_instance_ids.pop(process_name, None)
                self.process_cancel_events.pop(process_name, None)
                self.process_dependencies.pop(process_name, None)
                self.confirmed_healthy_processes.discard(process_name)

            generation = self.process_generations.get(process_name)
            if generation is None:
                self._lifecycle_generation_counter += 1
                generation = self._lifecycle_generation_counter

            instance_id = ''
            if process_name == 'liosam':
                instance_id = uuid.uuid4().hex
                launch_env = dict(
                    popen_kwargs.get('env') or os.environ)
                launch_env['LIO_SAM_INSTANCE_ID'] = instance_id
                launch_env['LIO_SAM_GENERATION'] = str(generation)
                popen_kwargs['env'] = launch_env

            process = subprocess.Popen(command, **popen_kwargs)
            pgid = self._capture_process_group(process)
            if process_name not in self.process_generations:
                self.process_generations[process_name] = generation
                self.process_cancel_events[process_name] = threading.Event()
            if instance_id:
                self.process_instance_ids[process_name] = instance_id
            self.processes[process_name] = process
            self.process_pgids[process_name] = pgid
            self.process_maps[process_name] = map_name or ''
            self.process_started_at[process_name] = time.monotonic()
            self._mark_starting(process_name)
            self.get_logger().info(
                f'已登记进程组: {process_name}, pid='
                f'{getattr(process, "pid", None)}, pgid={pgid}, '
                f'map={map_name or ""}, generation={generation}'
            )
            return process, generation

    def _record_start_failure(
        self, process_name: str, reason: str, expected_generation=None
    ):
        """Record a launch failure without erasing an earlier root cause."""
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if (expected_generation is not None and
                    self.process_generations.get(process_name) !=
                    expected_generation):
                return
            if self.process_states.get(process_name) != 'FAILED':
                if self.process_states.get(process_name) in (None, 'STOPPED'):
                    self.process_states[process_name] = 'STARTING'
                    self.health_failures[process_name] = 0
                self.update_process_health(
                    process_name, False, reason, True,
                    expected_generation=expected_generation)

    def _rollback_failed_launch(
        self, process_name: str, reason: str, expected_process=None,
        expected_generation=None
    ):
        """Record startup failure and clean only that launch's process group."""
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if not self._instance_matches_locked(
                    process_name, expected_process, expected_generation):
                self.get_logger().warning(
                    f'忽略 {process_name} 旧启动实例的迟到回滚: '
                    f'generation={expected_generation}')
                return False
        self._record_start_failure(
            process_name, reason, expected_generation=expected_generation)
        if process_name in self.processes:
            return self.shutdown_process(
                process_name,
                preserve_fault=True,
                expected_process=expected_process,
                expected_generation=expected_generation,
            )
        return True

    def _localization_session_now_ns(self):
        """Use the ROS clock domain used by sensor message headers."""
        try:
            return int(self.get_clock().now().nanoseconds)
        except (AttributeError, RuntimeError):
            return time.time_ns()

    def _publish_localization_session(self, active: bool):
        """Publish the generation that may create the authoritative map TF."""
        publisher = getattr(self, 'localization_session_pub', None)
        if publisher is None:
            return
        message = LocalizationSession()
        try:
            message.header.stamp = self.get_clock().now().to_msg()
        except (AttributeError, RuntimeError):
            pass
        message.header.frame_id = 'map'
        message.session_id = self.active_localization_session
        message.map_id = self.active_map
        message.session_start_ns = self.localization_session_start_ns
        message.active = bool(active)
        publisher.publish(message)

    @staticmethod
    def _call_relocalization_start_service(client, request, deadline, cancelled):
        if cancelled() or time.monotonic() >= deadline:
            return None
        try:
            if not client.service_is_ready():
                return None
            future = client.call_async(request)
            while not cancelled():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                if future.done():
                    return future.result()
                time.sleep(min(0.02, remaining))
            future.cancel()
        except Exception:
            # A service may disappear while the process is starting or stopping.
            return None
        return None

    def _wait_for_relocalization_ready(
        self, process, generation, session_id, map_name, session_start_ns
    ) -> tuple[bool, str]:
        """Drive managed lifecycle transitions and verify the current map target."""
        deadline = time.monotonic() + self.max_startup_time
        identity = (session_id, map_name, session_start_ns)
        transitions = {
            State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
            State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE,
        }
        attempts = dict.fromkeys(transitions, 0)
        reason = '等待重定位就绪服务'

        def cancelled():
            return (self._start_was_cancelled('re_localization', generation) or
                    not self._parent_is_running(process))

        def call(client, request, timeout):
            return self._call_relocalization_start_service(
                client, request, min(deadline, time.monotonic() + timeout), cancelled)

        while time.monotonic() < deadline and not cancelled():
            readiness = call(
                self.relocalization_readiness_client,
                GetLocalizationReadiness.Request(), 1.0)
            if readiness is None:
                reason = '重定位就绪服务无响应'
            elif (readiness.session_id, readiness.map_id,
                  readiness.session_start_ns) != identity:
                reason = (
                    '重定位就绪服务会话不匹配: '
                    f'session={readiness.session_id}, map={readiness.map_id}, '
                    f'start={readiness.session_start_ns}')
            else:
                result = call(self.relocalization_state_client, GetState.Request(), 1.0)
                if result is None:
                    reason = '重定位生命周期状态服务无响应'
                else:
                    state = result.current_state
                    if state.id == State.PRIMARY_STATE_ACTIVE and readiness.ready:
                        if not cancelled():
                            return True, ''
                        break
                    reason = f'重定位状态={state.label}: {readiness.message}'
                    if state.id == State.PRIMARY_STATE_FINALIZED:
                        return False, reason
                    if state.id in transitions:
                        if attempts[state.id] >= 2:
                            return False, f'{reason}（生命周期切换已尝试两次）'
                        if self.relocalization_change_client.service_is_ready():
                            request = ChangeState.Request()
                            request.transition.id = transitions[state.id]
                            attempts[state.id] += 1
                            # A lost reply is ambiguous. Re-read GetState before retrying.
                            call(self.relocalization_change_client, request, 12.0)
                        else:
                            reason = f'{reason}（生命周期切换服务未发现）'

            remaining = deadline - time.monotonic()
            if remaining > 0 and not self._wait_for_start_poll(
                    're_localization', generation, min(0.2, remaining)):
                break

        if self._start_was_cancelled('re_localization', generation):
            return False, '重定位启动已被关闭请求取消'
        if not self._parent_is_running(process):
            return False, f'重定位启动进程已退出: code={process.returncode}'
        return False, f'等待重定位就绪超时: {reason}'

    def launch_relocalization(self, map_name: str) -> bool:
        """启动重定位服务进程"""
        process = None
        generation = None
        try:
            self.get_logger().info(f'正在为地图启动重定位: {map_name}')

            self._ensure_process_tracking()
            with self.process_lifecycle_lock:
                if self._has_live_tracked_instance('re_localization'):
                    self.get_logger().warning(
                        '拒绝重复启动 re_localization: '
                        f'已有会话 {self.active_localization_session}, '
                        f'map={self.active_map}'
                    )
                    return False
                session_id = str(uuid.uuid4())
                session_start_ns = self._localization_session_now_ns()

            cmd = [
                'ros2', 'launch',
                're_localization',
                'run.real_launch.py',
                'autostart:=false',
                f'map_path:={self.map_dir}/{map_name}/{map_name}_raw.pcd',
                f'localization_session_id:={session_id}',
                f'map_id:={map_name}',
                f'session_start_ns:={session_start_ns}',
            ]

            spawned = self._spawn_tracked_process(
                're_localization',
                map_name,
                cmd,
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            if spawned is None:
                return False
            process, generation = spawned
            with self.process_lifecycle_lock:
                cancel_event = self.process_cancel_events.get(
                    're_localization')
                if (generation is None or
                        not self._instance_matches_locked(
                            're_localization', process, generation) or
                        cancel_event is None or cancel_event.is_set()):
                    return False
                self.active_localization_session = session_id
                self.active_map = map_name
                self.localization_session_start_ns = session_start_ns
                self._publish_localization_session(True)
            ready, reason = self._wait_for_relocalization_ready(
                process, generation, session_id, map_name, session_start_ns)
            if (ready and self._parent_is_running(process) and
                    self._commit_start_healthy('re_localization', generation)):
                self.get_logger().info(
                    f'重定位服务已激活且地图就绪: map={map_name}, session={session_id}')
                return True
            if self._start_was_cancelled('re_localization', generation):
                self.get_logger().info('重定位启动已被关闭请求取消')
                return False
            self._rollback_failed_launch(
                're_localization', reason or '重定位服务未能提交就绪状态',
                process, generation)
            return False

        except Exception as e:
            self.get_logger().error(f'启动重定位失败: {e}')
            if (process is not None and not self._start_was_cancelled(
                    're_localization', generation)):
                self._rollback_failed_launch(
                    're_localization', f'启动重定位异常: {e}',
                    process, generation)
            elif process is None:
                self._record_start_failure(
                    're_localization', f'启动重定位异常: {e}')
            return False

    @staticmethod
    def _pose_yaw(pose) -> float:
        q = pose.orientation
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            q.x, q.y, q.z, q.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('initial pose contains non-finite values')
        if not 0.5 <= norm_sq <= 1.5:
            raise ValueError('initial pose quaternion is invalid')
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _localization_dependency_matches_locked(
        self, process, generation, session_id, map_name, session_start_ns
    ) -> bool:
        """Validate the exact relocalization generation behind init-pose."""
        cancel_event = self.process_cancel_events.get('re_localization')
        return (
            process is not None and
            generation is not None and
            self.processes.get('re_localization') is process and
            self.process_generations.get('re_localization') == generation and
            cancel_event is not None and
            not cancel_event.is_set() and
            self.process_states.get('re_localization') == 'HEALTHY' and
            bool(session_id) and
            self.active_localization_session == session_id and
            self.active_map == map_name and
            self.localization_session_start_ns == session_start_ns
        )

    def launch_nav2_init_pose(self, map_name: str, initial_pose=None) -> bool:
        """启动nav2_init_pose进程"""
        process = None
        generation = None
        relocalization_process = None
        relocalization_generation = None
        try:
            self._ensure_process_tracking()
            with self.process_lifecycle_lock:
                relocalization_process = self.processes.get(
                    're_localization')
                relocalization_generation = self.process_generations.get(
                    're_localization')
                session_id = self.active_localization_session
                session_map = self.active_map
                session_start_ns = self.localization_session_start_ns
                dependency_valid = (
                    self._localization_dependency_matches_locked(
                        relocalization_process,
                        relocalization_generation,
                        session_id,
                        map_name,
                        session_start_ns,
                    )
                )
            if not dependency_valid:
                reason = (
                    'nav2_init_pose缺少健康且匹配的重定位实例: '
                    f'requested_map={map_name}, active_map={session_map}'
                )
                self.get_logger().error(reason)
                self._record_start_failure('nav2_init_pose', reason)
                return False

            cmd = [
                'ros2', 'run',
                'nav2_init_pose',
                'nav2_init_pose',
                '--ros-args',
                '-p', f'map_csv:={self.map_dir}/{map_name}/{map_name}.csv',
                '-p', f'map_yaml:={self.map_dir}/{map_name}/{map_name}_clean.yaml',
                '-p', f'localization_session_id:={session_id}',
                '-p', f'map_name:={map_name}',
                '-p', f'session_start_ns:={session_start_ns}',
                '--log-level', 'info'
            ]
            if initial_pose is not None:
                if initial_pose.header.frame_id != 'map':
                    raise ValueError(
                        'map-switch initial pose frame must be map')
                pose = initial_pose.pose
                initial_yaw = self._pose_yaw(pose)
                cmd[cmd.index('--log-level'):cmd.index('--log-level')] = [
                    '-p', 'use_initial_pose:=true',
                    '-p', f'initial_x:={pose.position.x}',
                    '-p', f'initial_y:={pose.position.y}',
                    '-p', f'initial_yaw:={initial_yaw}',
                ]

            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"
            env["RCUTILS_LOGGING_USE_STDOUT"] = "1"

            spawned = self._spawn_tracked_process(
                'nav2_init_pose',
                map_name,
                cmd,
                start_new_session=True,
                env=env,
                stdout=None,
                stderr=None
            )
            if spawned is None:
                return False
            process, generation = spawned
            with self.process_lifecycle_lock:
                own_cancel = self.process_cancel_events.get(
                    'nav2_init_pose')
                dependency_valid = (
                    self._instance_matches_locked(
                        'nav2_init_pose', process, generation) and
                    own_cancel is not None and
                    not own_cancel.is_set() and
                    self._localization_dependency_matches_locked(
                        relocalization_process,
                        relocalization_generation,
                        session_id,
                        map_name,
                        session_start_ns,
                    )
                )
                if dependency_valid:
                    self.process_dependencies['nav2_init_pose'] = (
                        're_localization',
                        relocalization_process,
                        relocalization_generation,
                        session_id,
                        map_name,
                        session_start_ns,
                    )
            if not dependency_valid:
                self._rollback_failed_launch(
                    'nav2_init_pose',
                    '重定位实例在nav2_init_pose启动期间停止或被替换',
                    process,
                    generation,
                )
                return False
            if not self._wait_for_start_poll(
                    'nav2_init_pose', generation, self.startup_delay):
                self.get_logger().info('nav2_init_pose启动已被关闭请求取消')
                return False

            process_running = self.is_process_running(
                'nav2_init_pose', process, generation)
            with self.process_lifecycle_lock:
                own_cancel = self.process_cancel_events.get(
                    'nav2_init_pose')
                dependency_valid = (
                    self._instance_matches_locked(
                        'nav2_init_pose', process, generation) and
                    own_cancel is not None and
                    not own_cancel.is_set() and
                    self._localization_dependency_matches_locked(
                        relocalization_process,
                        relocalization_generation,
                        session_id,
                        map_name,
                        session_start_ns,
                    )
                )
            if process_running and dependency_valid:
                self.get_logger().info('nav2_init_pose启动成功')
                return True
            else:
                if self._start_was_cancelled(
                        'nav2_init_pose', generation):
                    self.get_logger().info(
                        'nav2_init_pose启动已被关闭请求取消')
                    return False
                if not dependency_valid:
                    reason = '重定位实例在nav2_init_pose启动期间停止或被替换'
                else:
                    reason = 'nav2_init_pose启动健康检查失败'
                self.get_logger().error('nav2_init_pose启动失败')
                self._rollback_failed_launch(
                    'nav2_init_pose',
                    reason,
                    process,
                    generation,
                )
                return False

        except Exception as e:
            self.get_logger().error(f'启动nav2_init_pose失败: {e}')
            if (process is not None and not self._start_was_cancelled(
                    'nav2_init_pose', generation)):
                self._rollback_failed_launch(
                    'nav2_init_pose', f'启动nav2_init_pose异常: {e}',
                    process, generation)
            elif process is None:
                self._record_start_failure(
                    'nav2_init_pose', f'启动nav2_init_pose异常: {e}')
            return False

    def launch_liosam(self) -> bool:
        """启动LIO-SAM进程"""
        process = None
        generation = None
        try:
            self.get_logger().info('正在启动LIO-SAM')

            cmd = [
                'ros2', 'launch',
                'lio_sam',
                'run.real_launch.py'
            ]

            spawned = self._spawn_tracked_process(
                'liosam',
                '',
                cmd,
                start_new_session=True
            )
            if spawned is None:
                return False
            process, generation = spawned

            if (self.wait_for_process_healthy(
                    'liosam', generation) and
                    not self._start_was_cancelled('liosam', generation)):
                self.get_logger().info('LIO-SAM启动成功')
                return True
            else:
                if self._start_was_cancelled('liosam', generation):
                    self.get_logger().info('LIO-SAM启动已被关闭请求取消')
                    return False
                self.get_logger().error('LIO-SAM启动失败')
                self._rollback_failed_launch(
                    'liosam', 'LIO-SAM启动健康检查失败',
                    process, generation)
                return False

        except Exception as e:
            self.get_logger().error(f'启动LIO-SAM失败: {e}')
            if (process is not None and not self._start_was_cancelled(
                    'liosam', generation)):
                self._rollback_failed_launch(
                    'liosam', f'启动LIO-SAM异常: {e}',
                    process, generation)
            elif process is None:
                self._record_start_failure(
                    'liosam', f'启动LIO-SAM异常: {e}')
            return False

    def launch_navigation2(self, map_name: str) -> bool:
        """启动Navigation2进程"""
        process = None
        generation = None
        liosam_process = None
        liosam_generation = None
        nav2_log_file = None
        try:
            self.get_logger().info(f'正在为地图启动Navigation2: {map_name}')
            self._ensure_process_tracking()

            with self.process_lifecycle_lock:
                liosam_process = self.processes.get('liosam')
                liosam_generation = self.process_generations.get('liosam')
                liosam_cancel = self.process_cancel_events.get('liosam')
                liosam_state = self.process_states.get('liosam')
                liosam_available = (
                    liosam_process is not None and
                    liosam_generation is not None and
                    liosam_cancel is not None and
                    not liosam_cancel.is_set() and
                    liosam_state == 'HEALTHY'
                )
            if not liosam_available:
                reason = (
                    '拒绝启动 Navigation2: 缺少可用且受管的 LIO-SAM 实例')
                self.get_logger().error(reason)
                self._record_start_failure('navigation2', reason)
                return False

            cmd = [
                'ros2', 'launch',
                'nav2',
                'run.real_launch.py',
                f'map:={self.map_dir}/{map_name}/{map_name}_clean.yaml'
            ]

            # 【调试插桩】
            # ==========================================
            # 【新增逻辑】：创建一个专属的日志文件来捕捉 Nav2 的输出
            monitor_root = os.environ.get('CAR_JETSON_MONITOR_ROOT')
            if not monitor_root:
                run_pointer = (
                    '/home/akun/workspace/Car_jetson/'
                    'navigation_monitoring/current_run_dir'
                )
                try:
                    with open(run_pointer, encoding='utf-8') as pointer_file:
                        monitor_root = pointer_file.read().strip()
                except OSError:
                    monitor_root = ''
            if not monitor_root:
                monitor_root = (
                    '/home/akun/workspace/Car_jetson/navigation_monitoring/runs'
                )
            os.makedirs(monitor_root, exist_ok=True)
            log_file_path = os.path.join(
                monitor_root,
                f'nav2_{time.strftime("%Y-%m-%d_%H-%M-%S")}.log'
            )
            previous_log = getattr(self, 'nav2_log_file', None)
            nav2_log_file = open(log_file_path, 'w')
            self.get_logger().info(f'Nav2 的详细调试日志将保存在: {log_file_path}')
            # 使用 stdbuf 强制关闭 Linux 缓冲，保证日志实时写入文件
            # 如果不加 stdbuf，ROS 2 的日志可能会憋在内存里迟迟不写入文件
            stdbuf_cmd = ['stdbuf', '-o0', '-e0'] + cmd
            launch_env = self._nav2_launch_environment()
            spawned = self._spawn_tracked_process(
                'navigation2',
                map_name,
                stdbuf_cmd,               # 使用去缓冲的命令
                start_new_session=True,
                stdout=nav2_log_file,      # 标准输出重定向到文件
                stderr=subprocess.STDOUT,  # 错误输出合并到标准输出，一起存入文件
                env=launch_env
            )
            # ==========================================

            # process = subprocess.Popen(
            #     cmd,
            #     start_new_session=True
            # )

            if spawned is None:
                nav2_log_file.close()
                return False
            process, generation = spawned
            with self.process_lifecycle_lock:
                cancel_event = self.process_cancel_events.get('navigation2')
                if (generation is None or
                        not self._instance_matches_locked(
                            'navigation2', process, generation) or
                        cancel_event is None or cancel_event.is_set()):
                    nav2_log_file.close()
                    return False
                if previous_log is not None and not previous_log.closed:
                    previous_log.close()
                self.nav2_log_file = nav2_log_file

            if (self.wait_for_process_healthy(
                    'navigation2', generation,
                    liosam_process=liosam_process,
                    liosam_generation=liosam_generation) and
                    not self._start_was_cancelled(
                        'navigation2', generation)):
                self.get_logger().info('Navigation2启动成功')
                return True
            else:
                if self._start_was_cancelled(
                        'navigation2', generation):
                    self.get_logger().info('Navigation2启动已被关闭请求取消')
                    return False
                self.get_logger().error('Navigation2启动失败')
                self._rollback_failed_launch(
                    'navigation2', 'Navigation2启动健康检查失败',
                    process, generation)
                return False

        except Exception as e:
            self.get_logger().error(f'启动Navigation2失败: {e}')
            if (process is not None and not self._start_was_cancelled(
                    'navigation2', generation)):
                self._rollback_failed_launch(
                    'navigation2', f'启动Navigation2异常: {e}',
                    process, generation)
            else:
                if nav2_log_file is not None and not nav2_log_file.closed:
                    nav2_log_file.close()
                if process is None:
                    self._record_start_failure(
                        'navigation2', f'启动Navigation2异常: {e}')
            return False

    def _nav2_launch_environment(self):
        """Return the current environment with missing desktop session values filled."""
        launch_env = os.environ.copy()
        missing_keys = {
            key for key in GUI_ENVIRONMENT_KEYS if not launch_env.get(key)
        }
        if not missing_keys:
            return launch_env

        try:
            result = subprocess.run(
                ['systemctl', '--user', 'show-environment'],
                check=True,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.get_logger().warning(
                f'无法读取用户图形会话环境，Nav2将使用当前环境: {exc}'
            )
            return launch_env

        for line in result.stdout.splitlines():
            key, separator, value = line.partition('=')
            if separator and key in missing_keys and value:
                launch_env[key] = value

        return launch_env

    def shutdown_process(
        self, process_name: str, preserve_fault: bool = False,
        expected_process=None, expected_generation=None
    ) -> bool:
        """Coalesce duplicate stops and share the owner's verified result."""
        self._ensure_process_tracking()
        owner = False
        operation = None
        while not owner:
            with self.process_lifecycle_lock:
                target_process = (
                    expected_process if expected_process is not None else
                    self.processes.get(process_name))
                target_generation = (
                    expected_generation if expected_generation is not None else
                    self.process_generations.get(process_name))
                operation = self._shutdown_operations.get(process_name)
                if operation is None:
                    operation = {
                        'process': target_process,
                        'generation': target_generation,
                        'event': threading.Event(),
                        'result': False,
                    }
                    self._shutdown_operations[process_name] = operation
                    self._shutdown_in_progress.add(process_name)
                    if ((target_process is not None or
                         target_generation is not None) and
                            self._instance_matches_locked(
                                process_name,
                                target_process,
                                target_generation)):
                        cancel_event = self.process_cancel_events.get(
                            process_name)
                        if cancel_event is not None:
                            cancel_event.set()
                        self.process_states[process_name] = 'STOPPING'
                    owner = True
                    break
                same_instance = (
                    operation['process'] is target_process and
                    operation['generation'] == target_generation)

            self.get_logger().info(
                f'{process_name} 已在关闭，等待现有关闭事务结果')
            wait_timeout = max(
                1.0, float(getattr(self, 'shutdown_timeout', 10.0)) + 1.0)
            if not operation['event'].wait(wait_timeout):
                self.get_logger().error(
                    f'等待 {process_name} 现有关闭事务超时')
                return False
            if same_instance:
                return bool(operation['result'])

        result = False
        try:
            result = self._shutdown_process_impl(
                process_name,
                preserve_fault=preserve_fault,
                expected_process=operation['process'],
                expected_generation=operation['generation'],
            )
            return result
        finally:
            with self.process_lifecycle_lock:
                operation['result'] = bool(result)
                if self._shutdown_operations.get(process_name) is operation:
                    self._shutdown_operations.pop(process_name, None)
                    self._shutdown_in_progress.discard(process_name)
                operation['event'].set()

    def _shutdown_process_impl(
        self, process_name: str, preserve_fault: bool = False,
        expected_process=None, expected_generation=None
    ) -> bool:
        """Stop and verify the complete process group for ``process_name``."""
        self._ensure_process_tracking()
        # Capture the exact instance and cancel event before doing any blocking
        # signal/wait work.  A concurrent start callback observes the event and
        # leaves its health wait promptly; generation checks below prevent its
        # late rollback from touching a replacement instance.
        with self.process_lifecycle_lock:
            current_process = self.processes.get(process_name)
            current_generation = self.process_generations.get(process_name)
            if expected_process is not None or expected_generation is not None:
                if not self._instance_matches_locked(
                        process_name, expected_process, expected_generation):
                    self.get_logger().warning(
                        f'忽略 {process_name} 旧实例关闭请求: '
                        f'generation={expected_generation}')
                    return True
            generation = current_generation
            cancel_event = self.process_cancel_events.get(process_name)
            if cancel_event is not None:
                cancel_event.set()
            if current_process is not None:
                process = current_process
                pgid = self.process_pgids.get(process_name)
                self.process_states[process_name] = 'STOPPING'
            else:
                process = None
                pgid = None
                if generation is not None:
                    self.process_states[process_name] = 'STOPPING'

        if process is None:
            residuals = []
            if self.external_process_guard_enabled:
                residuals = self._find_untracked_processes(process_name)
            if residuals:
                pids = [pid for pid, _command in residuals]
                reason = (
                    f'无法通过本管理器关闭未纳管的 {process_name} '
                    f'残留进程 pid={pids}')
                with self.process_lifecycle_lock:
                    if (generation is None and
                            (process_name in self.processes or
                             process_name in self.process_generations)):
                        self.get_logger().error(reason)
                        return False
                    self._mark_shutdown_failure(
                        process_name, reason,
                        expected_generation=generation)
                return False
            self.get_logger().warning(f'未找到进程 {process_name}')
            with self.process_lifecycle_lock:
                if generation is None:
                    still_target_instance = (
                        process_name not in self.processes and
                        process_name not in self.process_generations)
                else:
                    still_target_instance = (
                        process_name not in self.processes and
                        self.process_generations.get(process_name) ==
                        generation)
                if still_target_instance:
                    if (process_name == 're_localization' and
                            self.active_localization_session):
                        self._publish_localization_session(False)
                        self.active_localization_session = ''
                        self.active_map = ''
                        self.localization_session_start_ns = 0
                    self.process_states[process_name] = 'STOPPED'
            return True

        if pgid is None:
            pgid = self._capture_process_group(process)
            with self.process_lifecycle_lock:
                if self._instance_matches_locked(
                        process_name, process, generation):
                    self.process_pgids[process_name] = pgid

        if pgid is None:
            reason = f'关闭失败: {process_name} 缺少可验证的进程组ID'
            self._mark_shutdown_failure(
                process_name, reason,
                expected_process=process,
                expected_generation=generation,
            )
            return False

        timeout = max(0.0, float(getattr(self, 'shutdown_timeout', 10.0)))
        shutdown_deadline = time.monotonic() + timeout
        graceful_deadline = min(
            shutdown_deadline,
            time.monotonic() + min(2.0, timeout * 0.7),
        )

        try:
            self.get_logger().info(
                f'正在关闭进程组: {process_name}, pgid={pgid}')

            members = self._live_process_group_members(pgid)
            if members:
                try:
                    os.killpg(pgid, signal.SIGTERM)
                except ProcessLookupError:
                    pass

            if self._wait_for_process_group_exit(
                    pgid, process, graceful_deadline):
                self.get_logger().info(
                    f'{process_name} 进程组 pgid={pgid} 已全部关闭')
                self._forget_process(
                    process_name, preserve_fault,
                    expected_process=process,
                    expected_generation=generation,
                )
                return True

            remaining = sorted(self._live_process_group_members(pgid))
            self.get_logger().warning(
                f'{process_name} 进程组仍有成员 {remaining}，正在强制终止')
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass

            if self._wait_for_process_group_exit(
                    pgid, process, shutdown_deadline):
                self._forget_process(
                    process_name, preserve_fault,
                    expected_process=process,
                    expected_generation=generation,
                )
                return True

            remaining = sorted(self._live_process_group_members(pgid))
            reason = (
                f'关闭失败: {process_name} 进程组 pgid={pgid} '
                f'仍有非僵尸进程 {remaining}'
            )
            self._mark_shutdown_failure(
                process_name, reason,
                expected_process=process,
                expected_generation=generation,
            )
            return False
        except Exception as e:
            reason = f'关闭 {process_name} 失败: {e}'
            self._mark_shutdown_failure(
                process_name, reason,
                expected_process=process,
                expected_generation=generation,
            )
            return False

    def _wait_for_process_group_exit(
        self, pgid: int, process, deadline: float = None
    ) -> bool:
        """Wait until no non-zombie member remains in the saved group."""
        if deadline is None:
            timeout = max(
                0.0, float(getattr(self, 'shutdown_timeout', 10.0)))
            deadline = time.monotonic() + timeout
        while True:
            try:
                process.poll()  # Reap the launch parent when it has exited.
            except (AttributeError, OSError):
                pass

            if not self._live_process_group_members(pgid):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))

    def _mark_shutdown_failure(
        self, process_name: str, reason: str, expected_process=None,
        expected_generation=None
    ):
        """Keep the live tracking record and expose a persistent fault."""
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if not self._instance_matches_locked(
                    process_name, expected_process, expected_generation):
                return
            self.process_states[process_name] = 'FAILED'
            self.health_failures[process_name] = max(
                1, self.health_failures.get(process_name, 0))
            previous_reason = self.fault_reasons.get(process_name)
            if previous_reason and previous_reason != reason:
                self.fault_reasons[process_name] = (
                    f'{previous_reason}; {reason}'
                )
            else:
                self.fault_reasons[process_name] = reason
        self.get_logger().error(reason)

    def is_process_running(
        self, process_name: str, expected_process=None,
        expected_generation=None
    ) -> bool:
        """Probe one exact instance without updating a replacement process."""
        self._ensure_process_tracking()
        strict_instance = (
            expected_process is not None or expected_generation is not None)
        with self.process_lifecycle_lock:
            if expected_process is None:
                expected_process = self.processes.get(process_name)
            if expected_generation is None:
                expected_generation = self.process_generations.get(process_name)
            if expected_process is None:
                return False
            if strict_instance and expected_generation is None:
                return False
            if not self._instance_matches_locked(
                    process_name, expected_process, expected_generation):
                return False

        healthy, reason, immediate_failure = self.probe_process_health(
            process_name)
        with self.process_lifecycle_lock:
            if not self._instance_matches_locked(
                    process_name, expected_process, expected_generation):
                return False
        self.update_process_health(
            process_name, healthy, reason, immediate_failure,
            expected_generation=expected_generation)
        with self.process_lifecycle_lock:
            return (
                self._instance_matches_locked(
                    process_name, expected_process, expected_generation) and
                self.process_states.get(process_name) in
                ('HEALTHY', 'DEGRADED')
            )

    def _mark_starting(self, process_name: str):
        self._ensure_process_tracking()
        self.process_states[process_name] = 'STARTING'
        self.health_failures[process_name] = 0
        self.fault_reasons.pop(process_name, None)
        self.confirmed_healthy_processes.discard(process_name)
        if process_name == 'liosam':
            self.liosam_ready = False
            self.liosam_ever_ready = False
            self.liosam_not_ready_since = None
            self.liosam_stable_correction_count = 0
            self.liosam_stability_ready = False
            self.liosam_last_correction_epoch = 0

    def _forget_process(
        self, process_name: str, preserve_fault: bool = False,
        expected_process=None, expected_generation=None
    ):
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if not self._instance_matches_locked(
                    process_name, expected_process, expected_generation):
                self.get_logger().warning(
                    f'忽略 {process_name} 旧实例清理: '
                    f'generation={expected_generation}')
                return False
            self.processes.pop(process_name, None)
            self.process_pgids.pop(process_name, None)
            self.process_maps.pop(process_name, None)
            self.process_started_at.pop(process_name, None)
            self.process_generations.pop(process_name, None)
            self.process_instance_ids.pop(process_name, None)
            self.process_cancel_events.pop(process_name, None)
            self.process_dependencies.pop(process_name, None)
            self.confirmed_healthy_processes.discard(process_name)
            if process_name == 're_localization':
                self._publish_localization_session(False)
                self.active_localization_session = ''
                self.active_map = ''
                self.localization_session_start_ns = 0
            if process_name == 'liosam':
                self.liosam_ready = False
                self.liosam_ever_ready = False
                self.liosam_not_ready_since = None
                self.liosam_stable_correction_count = 0
                self.liosam_stability_ready = False
                self.liosam_last_correction_epoch = 0
            if process_name == 'navigation2':
                nav2_log = getattr(self, 'nav2_log_file', None)
                if nav2_log is not None and not nav2_log.closed:
                    nav2_log.close()
            if preserve_fault:
                self.process_states[process_name] = 'FAILED'
            else:
                self.process_states[process_name] = 'STOPPED'
                self.health_failures.pop(process_name, None)
                self.fault_reasons.pop(process_name, None)
        return True

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
        self._ensure_process_tracking()
        process = self.processes.get(process_name)
        if process is None:
            return False, 'launch process is not tracked', True
        if process.poll() is not None:
            return False, f'launch process exited with code {process.returncode}', True

        if process_name == 'nav2_init_pose':
            with self.process_lifecycle_lock:
                dependency = self.process_dependencies.get(process_name)
                dependency_valid = (
                    dependency is not None and
                    len(dependency) == 6 and
                    dependency[0] == 're_localization' and
                    self._localization_dependency_matches_locked(
                        *dependency[1:])
                )
            if not dependency_valid:
                return (
                    False,
                    'bound relocalization instance stopped or was replaced',
                    True,
                )

        commands = self._running_process_commands(process)
        missing_processes = [
            pattern for pattern in self.required_process_patterns.get(process_name, ())
            if not any(pattern in command for command in commands)
        ]
        if missing_processes:
            immediate_failure = (
                process_name in self.confirmed_healthy_processes
            )
            return (
                False,
                f'missing child processes: {", ".join(missing_processes)}',
                immediate_failure,
            )

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

            with self.process_lifecycle_lock:
                dependency = self.process_dependencies.get('navigation2')
                liosam_tracked = 'liosam' in self.processes
                liosam_state = self.process_states.get('liosam')
                liosam_ready = getattr(self, 'liosam_ready', False)
                liosam_stability_ready = getattr(
                    self, 'liosam_stability_ready', False)
                if dependency is not None:
                    dependency_name, bound_process, bound_generation = dependency
                    bound_cancel = self.process_cancel_events.get(
                        dependency_name)
                    if (dependency_name != 'liosam' or
                            self.processes.get('liosam') is not bound_process or
                            self.process_generations.get('liosam') !=
                            bound_generation or
                            bound_cancel is None or bound_cancel.is_set()):
                        return (
                            False,
                            'bound LIO-SAM instance stopped or was replaced',
                            True,
                        )
            if (not liosam_tracked or
                    liosam_state in ('STOPPING', 'STOPPED', 'FAILED')):
                return (
                    False,
                    'required LIO-SAM process is not healthy and tracked',
                    True,
                )
            if not liosam_ready:
                return (
                    False,
                    'waiting for LIO-SAM IMU preintegration recovery',
                    False,
                )
            if not liosam_stability_ready:
                return (
                    False,
                    'waiting for LIO-SAM stable scan corrections',
                    False,
                )

        if process_name == 'liosam' and not getattr(self, 'liosam_ready', False):
            not_ready_since = getattr(self, 'liosam_not_ready_since', None)
            if (getattr(self, 'liosam_ever_ready', False) and
                    not_ready_since is not None):
                elapsed = max(0.0, time.monotonic() - not_ready_since)
                recovery_grace = max(
                    0.0, getattr(self, 'liosam_recovery_grace_sec', 5.0))
                if elapsed < recovery_grace:
                    return (
                        False,
                        'IMU preintegration is recovering '
                        f'({elapsed:.1f}/{recovery_grace:.1f} s)',
                        False,
                    )
                return (
                    False,
                    'IMU preintegration recovery timed out after '
                    f'{elapsed:.1f} s',
                    True,
                )
            return False, 'IMU preintegration is not ready', False

        if (process_name == 'liosam' and
                not getattr(self, 'liosam_stability_ready', False)):
            count = getattr(self, 'liosam_stable_correction_count', 0)
            required = getattr(
                self, 'liosam_stable_correction_count_required', 10)
            return (
                False,
                'waiting for LIO-SAM stable scan corrections '
                f'({count}/{required})',
                False,
            )

        return True, '', False

    def update_process_health(
        self, process_name: str, healthy: bool, reason: str,
        immediate_failure: bool = False, expected_generation=None
    ):
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if (expected_generation is not None and
                    self.process_generations.get(process_name) !=
                    expected_generation):
                return
            previous_state = self.process_states.get(process_name, 'STOPPED')
            if previous_state in ('STOPPING', 'STOPPED', 'FAILED'):
                return

            if healthy:
                if process_name == 're_localization' and previous_state == 'STARTING':
                    return  # Only the map-readiness gate can complete this startup.
                self.process_states[process_name] = 'HEALTHY'
                self.health_failures[process_name] = 0
                self.fault_reasons.pop(process_name, None)
                self.confirmed_healthy_processes.add(process_name)
                return

            recovery_in_progress = (
                process_name == 'liosam' and
                reason.startswith('IMU preintegration is recovering ')
            ) or (
                process_name == 'navigation2' and
                reason == 'waiting for LIO-SAM IMU preintegration recovery'
            ) or (
                process_name == 'liosam' and
                reason.startswith(
                    'waiting for LIO-SAM stable scan corrections ')
            ) or (
                process_name == 'navigation2' and
                reason == 'waiting for LIO-SAM stable scan corrections'
            )
            if recovery_in_progress:
                self.process_states[process_name] = 'DEGRADED'
                self.health_failures[process_name] = 0
                self.fault_reasons.pop(process_name, None)
                return

            failures = self.health_failures.get(process_name, 0) + 1
            self.health_failures[process_name] = failures
            confirmed = (
                immediate_failure or
                failures >= self.health_failure_threshold)
            if not confirmed:
                self.process_states[process_name] = 'DEGRADED'
                return

            self.process_states[process_name] = 'FAILED'
            self.fault_reasons[process_name] = reason
            if previous_state != 'FAILED':
                self.get_logger().error(
                    f'{process_name}健康检查失败: {reason}')
                message = String()
                message.data = f'{process_name}: {reason}'
                self.fault_pub.publish(message)

    def _commit_start_healthy(
        self, process_name: str, expected_generation=None, *,
        liosam_process=None, liosam_generation=None
    ) -> bool:
        """Commit startup success while the process dependencies are stable."""
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            if expected_generation is not None:
                if (self.process_generations.get(process_name) !=
                        expected_generation):
                    return False
                cancel_event = self.process_cancel_events.get(process_name)
                if cancel_event is None or cancel_event.is_set():
                    return False
            if liosam_process is not None or liosam_generation is not None:
                liosam_cancel = self.process_cancel_events.get('liosam')
                if (liosam_process is None or liosam_generation is None or
                        self.processes.get('liosam') is not liosam_process or
                        self.process_generations.get('liosam') !=
                        liosam_generation or
                        liosam_cancel is None or liosam_cancel.is_set() or
                        self.process_states.get('liosam') != 'HEALTHY'):
                    return False
            if self.process_states.get(process_name) in (
                    'STOPPING', 'STOPPED', 'FAILED'):
                return False
            self.process_states[process_name] = 'HEALTHY'
            self.health_failures[process_name] = 0
            self.fault_reasons.pop(process_name, None)
            self.confirmed_healthy_processes.add(process_name)
            if (process_name == 'navigation2' and
                    liosam_process is not None and
                    liosam_generation is not None):
                self.process_dependencies[process_name] = (
                    'liosam', liosam_process, liosam_generation)
            return True

    def health_check_callback(self):
        self._ensure_process_tracking()
        with self.process_lifecycle_lock:
            instances = [
                (
                    process_name,
                    process,
                    self.process_generations.get(process_name),
                    self.process_states.get(process_name),
                )
                for process_name, process in self.processes.items()
            ]
        for process_name, process, generation, state in instances:
            if state in ('STARTING', 'STOPPING', 'STOPPED'):
                continue
            healthy, reason, immediate = self.probe_process_health(process_name)
            with self.process_lifecycle_lock:
                if not self._instance_matches_locked(
                        process_name, process, generation):
                    continue
            self.update_process_health(
                process_name, healthy, reason, immediate,
                expected_generation=generation)
            if self.process_states.get(process_name) == 'FAILED':
                self.shutdown_process(
                    process_name,
                    preserve_fault=True,
                    expected_process=process,
                    expected_generation=generation,
                )

    def _startup_timeout_for(self, process_name: str) -> float:
        if process_name == 'navigation2':
            return self.nav2_max_startup_time
        return self.max_startup_time

    def _tracked_liosam_failure_reason(
        self, expected_process=None, expected_generation=None,
        require_healthy=False
    ) -> str:
        """Return a failure when Nav2's bound LIO-SAM disappears or fails."""
        self._ensure_process_tracking()
        strict_dependency = (
            expected_process is not None or expected_generation is not None)
        with self.process_lifecycle_lock:
            current_process = self.processes.get('liosam')
            current_generation = self.process_generations.get('liosam')
            state = self.process_states.get('liosam')
            cancel_event = self.process_cancel_events.get('liosam')
            if strict_dependency:
                if expected_process is None or expected_generation is None:
                    return 'required LIO-SAM instance has no generation'
                if (current_process is not expected_process or
                        current_generation != expected_generation):
                    return 'required LIO-SAM instance stopped or was replaced'
                if cancel_event is None or cancel_event.is_set():
                    return 'required LIO-SAM instance is stopping'
                if state in ('STOPPING', 'STOPPED', 'FAILED'):
                    return self.fault_reasons.get(
                        'liosam', f'LIO-SAM health state is {state}')
            else:
                if state == 'FAILED':
                    return self.fault_reasons.get(
                        'liosam', 'LIO-SAM health state is FAILED')
                if current_process is None:
                    return ''

        healthy, reason, immediate = self.probe_process_health('liosam')
        with self.process_lifecycle_lock:
            if strict_dependency and (
                    self.processes.get('liosam') is not expected_process or
                    self.process_generations.get('liosam') !=
                    expected_generation):
                return 'required LIO-SAM instance stopped or was replaced'
        if require_healthy and not healthy:
            return reason or 'required LIO-SAM current health probe failed'
        # This method is polled every 0.25 s while Nav2 starts. Only mutate the
        # tracked health for an immediate failure; ordinary health failures are
        # counted by the configured health timer instead of four times a second.
        if healthy:
            self.update_process_health(
                'liosam', True, '',
                expected_generation=(
                    expected_generation if strict_dependency else None))
        elif immediate:
            self.update_process_health(
                'liosam', healthy, reason, True,
                expected_generation=(
                    expected_generation if strict_dependency else None))
        with self.process_lifecycle_lock:
            state = self.process_states.get('liosam')
            if state == 'FAILED':
                return self.fault_reasons.get('liosam', reason)
            if require_healthy and state != 'HEALTHY':
                return f'required LIO-SAM state is {state or "UNKNOWN"}'
        return ''

    def _abort_navigation2_for_liosam_failure(
        self, process_name: str, expected_generation=None,
        liosam_process=None, liosam_generation=None,
        require_healthy=False
    ) -> bool:
        if process_name != 'navigation2':
            return False
        liosam_failure = self._tracked_liosam_failure_reason(
            liosam_process, liosam_generation, require_healthy)
        if not liosam_failure:
            return False

        reason = (
            'LIO-SAM failed while waiting for Navigation2 startup: '
            f'{liosam_failure}'
        )
        self.update_process_health(
            process_name, False, reason, True,
            expected_generation=expected_generation)
        return True

    def wait_for_process_healthy(
        self, process_name: str, expected_generation=None, *,
        liosam_process=None, liosam_generation=None
    ) -> bool:
        self._ensure_process_tracking()
        if expected_generation is None:
            with self.process_lifecycle_lock:
                expected_generation = self.process_generations.get(process_name)
        deadline = time.monotonic() + self._startup_timeout_for(process_name)
        while time.monotonic() < deadline:
            if self._start_was_cancelled(
                    process_name, expected_generation):
                return False
            if self._abort_navigation2_for_liosam_failure(
                    process_name, expected_generation,
                    liosam_process, liosam_generation):
                return False

            healthy, reason, immediate = self.probe_process_health(process_name)
            if self._start_was_cancelled(
                    process_name, expected_generation):
                return False
            if healthy:
                if ((liosam_process is not None or
                     liosam_generation is not None) and
                        self._abort_navigation2_for_liosam_failure(
                            process_name, expected_generation,
                            liosam_process, liosam_generation,
                            require_healthy=True)):
                    return False
                return self._commit_start_healthy(
                    process_name, expected_generation,
                    liosam_process=liosam_process,
                    liosam_generation=liosam_generation)
            if immediate:
                self.update_process_health(
                    process_name, False, reason, True,
                    expected_generation=expected_generation)
                return False
            if not self._wait_for_start_poll(
                    process_name, expected_generation, 0.25):
                return False

        if self._start_was_cancelled(process_name, expected_generation):
            return False
        if self._abort_navigation2_for_liosam_failure(
                process_name, expected_generation,
                liosam_process, liosam_generation):
            return False

        healthy, reason, immediate = self.probe_process_health(process_name)
        if self._start_was_cancelled(process_name, expected_generation):
            return False
        if healthy:
            if ((liosam_process is not None or
                 liosam_generation is not None) and
                    self._abort_navigation2_for_liosam_failure(
                        process_name, expected_generation,
                        liosam_process, liosam_generation,
                        require_healthy=True)):
                return False
            return self._commit_start_healthy(
                process_name, expected_generation,
                liosam_process=liosam_process,
                liosam_generation=liosam_generation)
        timeout_reason = f'startup health check timed out: {reason}'
        self.update_process_health(
            process_name, False, timeout_reason, True,
            expected_generation=expected_generation)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ProcessManagerNode()

    # 使用多线程executor以支持并发服务调用
    # One thread may spend the full startup timeout in a start callback while
    # stop, status, and health callbacks still need independent workers.
    # The monitor can issue four different shutdown requests together.  Keep
    # capacity for status, readiness and lifecycle callbacks while those waits
    # use worker threads.
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Prevent a pre-Popen admission from spawning after cleanup starts,
        # then drain service callbacks before touching their tracked groups.
        node.begin_shutdown()
        # begin_shutdown makes every startup wait cancellable.  Wait until all
        # callbacks have actually returned before destroying their ROS node.
        executor.shutdown(timeout_sec=None)
        if rclpy.ok():
            node.get_logger().info('关闭所有进程...')
        with node.process_lifecycle_lock:
            tracked_names = set(node.processes) | set(node.process_generations)
        shutdown_order = (
            'navigation2', 'liosam', 'nav2_init_pose', 're_localization')
        for process_name in shutdown_order:
            if process_name in tracked_names:
                node.shutdown_process(process_name)
        for process_name in sorted(tracked_names - set(shutdown_order)):
            node.shutdown_process(process_name)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
