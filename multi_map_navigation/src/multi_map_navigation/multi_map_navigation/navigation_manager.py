#!/usr/bin/env python3
"""
导航管理器
开发者B - 导航模块

管理航点导航序列并与地图切换协调
"""
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from std_msgs.msg import String
from multi_map_navigation_msgs.msg import (
    LocalizationResult,
    LocalizationSession,
    MapSwitchCancel,
    MapSwitchResult,
    MapSwitchTrigger,
    Waypoint,
    WaypointList,
)
from multi_map_navigation_msgs.srv import GetProcessStatus, StartProcess, ShutdownProcess, PubNewPath
from typing import List, Optional
import math
import time
import threading
import uuid
import tf2_ros
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from multi_map_navigation.charging_navigation import ChargingNavigation
from multi_map_navigation.stack_transaction_lock import StackTransactionLock
from multi_map_navigation.static_footprint_gate import StaticMapFootprintChecker


def _quaternion_from_yaw(yaw):
    """Return an x/y/z/w quaternion without a NumPy-dependent helper."""
    half_yaw = float(yaw) * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class NavigationManager(ChargingNavigation, Node):
    CLEANUP_DEFERRED = object()
    STACK_PROCESS_NAMES = (
        're_localization', 'nav2_init_pose', 'liosam', 'navigation2')
    # ProcessManager may spend 150 s proving Nav2 healthy and up to 20 s
    # rolling its process group back.  Keep the client deadline beyond both.
    PROCESS_START_TIMEOUT_SEC = 180.0
    # ProcessManager allows a process group 10 s to stop gracefully.
    PROCESS_SHUTDOWN_TIMEOUT_SEC = 15.0
    CLEAN_STATUS_TIMEOUT_SEC = 15.0
    CLEAN_STATUS_POLL_SEC = 0.25
    CLEAN_STATUS_CONFIRMATIONS = 2
    STACK_TRANSACTION_WAIT_SEC = 1.0
    DEFERRED_CLEANUP_WAIT_SEC = 1200.0
    DEFERRED_STACK_START_TIMEOUT_SEC = 300.0

    def __init__(self):
        super().__init__('navigation_manager')

        # 声明参数
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('navigation_timeout', 300.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('retry_delay', 5.0)
        self.declare_parameter(
            'map_directory',
            '/home/akun/workspace/Car_jetson/multi_map_navigation/src/map')

        # 获取参数
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        self.max_retries = self.get_parameter('max_retries').value
        self.retry_delay = self.get_parameter('retry_delay').value
        self.map_directory = self.get_parameter('map_directory').value

        # 导航状态
        self.state_lock = threading.RLock() # 可重入锁，防止同线程嵌套调用死锁

        self.waypoint_list: Optional[List[Waypoint]] = None # 当前任务的航点列表
        self.current_waypoint_index = 0 # 当前要去的航点下标
        self.current_map = None
        self.is_navigating = False
        self.is_map_switching = False
        self._active_map_switch_transaction_id = ''
        self._active_map_switch_context = None
        self.task_id = None
        self.current_goal_handle = None # 当前发给 Nav2 的目标句柄
        self._navigation_generation = 0
        self._goal_sequence = 0
        self._active_goal_token = None
        self.total_path = None
        self.stack_fault_active = False
        self._active_localization_session = ''
        self._active_localization_map = ''
        self._localization_session_start_ns = 0
        self._expected_localization_session = ''
        self._expected_localization_map = ''
        self._expected_session_start_ns = 0
        self._latest_localization_result = None
        self._pending_localization_result = None
        self._startup_footprint_verified_session = ''
        self._static_footprint_checkers = {}
        self._startup_tf_buffer = tf2_ros.Buffer()
        self._startup_tf_listener = tf2_ros.TransformListener(
            self._startup_tf_buffer, self)
        self._last_status_has_identity = False
        self._last_status_from_service = False
        self._last_reported_stack_state = ''
        self._last_reported_stack_map = ''
        self._last_reported_stack_session = ''
        # This gate is latched before every cleanup.  It is only released
        # after ProcessManager reports a completely stopped stack on
        # consecutive queries.
        self._navigation_cleanup_active = False
        self._last_stack_rollback_verified = True
        self._stack_start_deferred = False
        self._stack_start_deferred_deadline = None
        self._stack_start_retry_timer = None
        self._stack_start_retry_generation = 0
        self._stack_transaction_lock = StackTransactionLock()
        self._deferred_cleanup_lock = threading.Lock()
        self._deferred_cleanup_thread = None
        self.pending_replan_endpoints = None
        self.stack_recovery_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._reset_stack_recovery()

        # 创建可重入回调组，避免服务调用与订阅回调互相阻塞
        self.service_callback_group = ReentrantCallbackGroup()

        # ROS2订阅器用于航点列表
        # 添加后缀test订阅测试话题
        self.waypoint_list_sub = self.create_subscription(
            WaypointList,
            '/waypoint_list',
            # '/test/waypoint_list',
            self.waypoint_list_callback,
            10
        )

        # ROS2订阅器用于地图切换完成
        self.map_switch_complete_sub = self.create_subscription(
            MapSwitchResult,
            '/map_switch_complete',
            # '/test/map_switch_complete',
            self.map_switch_complete_callback,
            10
        )

        self.map_switch_cancel_pub = self.create_publisher(
            MapSwitchCancel,
            '/map_switch_cancel',
            10,
        )

        self.process_fault_sub = self.create_subscription(
            String,
            '/process_manager/fault',
            self.process_fault_callback,
            10,
            callback_group=self.service_callback_group
        )
        self.lio_localization_fault_sub = self.create_subscription(
            String,
            '/lio_sam/localization_fault',
            self.process_fault_callback,
            10,
            callback_group=self.service_callback_group,
        )

        localization_qos = QoSProfile(depth=1)
        localization_qos.reliability = ReliabilityPolicy.RELIABLE
        localization_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.localization_session_sub = self.create_subscription(
            LocalizationSession,
            '/localization_session',
            self.localization_session_callback,
            localization_qos,
            callback_group=self.service_callback_group,
        )
        self.localization_result_sub = self.create_subscription(
            LocalizationResult,
            '/localization_result',
            self.localization_result_callback,
            localization_qos,
            callback_group=self.service_callback_group,
        )

        # ROS2发布器用于机器人状态
        self.robot_state_pub = self.create_publisher(
            String,
            '/robot_state',
            10
        )

        # ROS2发布器用于地图切换触发
        self.map_switch_trigger_pub = self.create_publisher(
            MapSwitchTrigger,
            '/trigger_map_switch',
            # '/test/trigger_map_switch',
            10
        )

        self.emergency_cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Navigation2动作客户端
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.init_charging_navigation()

        # 发布初始状态
        self.publish_robot_state("idle")

        # 进程管理服务客户端 - 使用可重入回调组
        self.start_process_client = self.create_client(
            StartProcess, 
            '/process_manager/start_process',
            # '/test/process_manager/start_process',
            callback_group=self.service_callback_group
        )
        self.shutdown_process_client = self.create_client(
            ShutdownProcess, 
            '/process_manager/shutdown_process',
            # '/test/process_manager/shutdown_process',
            callback_group=self.service_callback_group
        )
        self.get_process_status_client = self.create_client(
            GetProcessStatus,
            '/process_manager/get_status',
            # '/test/process_manager/get_status',
            callback_group=self.service_callback_group
        )

        # 发布路径的客户端
        # 添加后缀test接入测试服务接口
        self.pub_new_path_client = self.create_client(
            PubNewPath, 
            '/route_planner/pub_new_path',
            # '/test/route_planner/pub_new_path',
            callback_group=self.service_callback_group
        )

        # 等待服务可用
        self.get_logger().info('等待启动进程服务...')
        while self.start_process_client.wait_for_service(timeout_sec=10.0) == False:
            pass
        self.get_logger().info('启动进程服务已连接')

        self.get_logger().info('等待关闭进程服务...')
        while self.shutdown_process_client.wait_for_service(timeout_sec=10.0) == False:
            pass
        self.get_logger().info('关闭进程服务已连接')
        
        self.get_logger().info('等待路径服务...')
        while self.pub_new_path_client.wait_for_service(timeout_sec=10.0) == False:
            pass
        self.get_logger().info('发布路径服务已连接')

        # for test
        self.test_count = 0

    def waypoint_list_callback(self, msg: WaypointList):
        """
        function: 接收航点列表的回调
        param: @ msg: 包含导航航点的WaypointList消息
        ------
        test: 测试通过，可以正常接收
        """

        # 加锁，防止多线程同时改变同一个状态变量，读完自动解锁
        with self.state_lock:
            if getattr(self, '_navigation_cleanup_active', False):
                self.get_logger().warn('导航堆栈清理中，忽略新航点列表')
                return
            if getattr(self, 'docking_hold', False):
                self.get_logger().warn('充电对接保持中，忽略新任务；需先完成或恢复当前任务')
                return
            # 判断是否还在运行中
            if self.is_navigating:
                endpoints = self.pending_replan_endpoints
                is_requested_replan = (
                    endpoints is not None
                    and bool(msg.waypoints)
                    and (msg.waypoints[0].name, msg.waypoints[-1].name) == endpoints
                )
                if not is_requested_replan:
                    self.get_logger().warn('已在导航中，忽略新航点列表')
                    return
                self.get_logger().info('接收当前任务请求的备用路线')
            self._reset_stack_recovery()
            self._navigation_generation = getattr(self, '_navigation_generation', 0) + 1
            self._active_goal_token = None
            self._active_map_switch_transaction_id = ''
            self._active_map_switch_context = None
            self.current_goal_handle = None
            self.pending_replan_endpoints = None
            # 存储航点列表
            self.waypoint_list = msg.waypoints
            self.task_id = msg.task_id
            self.current_waypoint_index = 0
            self.current_map = msg.start_map_name
            self.total_path = msg.total_path
            self.is_navigating = True
            self.stack_fault_active = False
            self.publish_robot_state('running')
            # 取消上条路径的navgation2导航任务
            # if self.current_goal_handle is not None:
            #     self.current_goal_handle.cancel_goal_async()

            # 打印日志
        self.get_logger().info(
            f'收到航点列表: task_id={msg.task_id}, '
            f'航点列表: waypoint_list{self.waypoint_list}'
            f'total_waypoints={msg.total_waypoints}'
        )

        # 开始导航序列
        self.start_navigation_sequence()

    def start_navigation_sequence(self):
        """
        function:从第一个航点开始导航序列
        param:
        return:
        ----------
        test: 测试通过
        """
        with self.state_lock:
            if self.stack_fault_active or not self.is_navigating:
                return
            waypoint_list = self.waypoint_list
            navigation_generation = getattr(self, '_navigation_generation', 0)
            self.publish_robot_state('running')

        if not waypoint_list or len(waypoint_list) == 0:
            self.get_logger().error('没有航点可导航')
            self.complete_navigation(expected_generation=navigation_generation)
            return

        self.get_logger().info(
            f'开始导航序列，共 {len(waypoint_list)} 个航点'
        )

        # 导航到第一个航点
        self.navigate_to_next_waypoint(navigation_context=(navigation_generation, 0))

    def navigate_to_next_waypoint(self, navigation_context=None):
        """
        function: 导航到序列中的下一个航点
        param: 
        return:
        -----------
        test: 测试通过
        """
        with self.state_lock:
            if (navigation_context is not None
                    and not self._navigation_context_is_current(*navigation_context)):
                return
            if self.stack_fault_active or not self.is_navigating:
                return
            if getattr(self, 'docking_hold', False):
                self.get_logger().warning('对接保持中，等待显式释放控制权')
                return
            waypoint_list = self.waypoint_list
            current_waypoint_index = self.current_waypoint_index
            navigation_generation = getattr(self, '_navigation_generation', 0)

        if not waypoint_list or current_waypoint_index >= len(waypoint_list):
            self.get_logger().info('所有航点已完成')
            self.complete_navigation(expected_generation=navigation_generation)
            return
            
        waypoint = waypoint_list[current_waypoint_index]

        self.get_logger().info(
            f'正在导航到航点 {current_waypoint_index + 1}/{len(waypoint_list)}: '
            f'type={waypoint.type}, map={waypoint.map_name}'
        )

        # A status timeout/partial stack is unsafe: starting another copy can
        # leave stale TF and DDS participants.  Only an explicitly all-running
        # stack may be reused, and only an explicitly all-stopped stack may be
        # started.
        process_status_dict = self.get_process_status()
        if not self._navigation_context_is_current(
                navigation_generation, current_waypoint_index, waypoint):
            return
        status_kind = self._classify_process_status(process_status_dict)
        deferred_start = getattr(self, '_stack_start_deferred', False)
        reported_state = getattr(self, '_last_reported_stack_state', '')
        if deferred_start and reported_state != 'RUNNING':
            deadline = getattr(
                self, '_stack_start_deferred_deadline', None)
            if deadline is not None and time.monotonic() >= deadline:
                with self.state_lock:
                    self._stack_start_deferred = False
                    self._stack_start_deferred_deadline = None
                self.get_logger().error(
                    '等待 ProcessManager 启动事务结束超时')
                fault = String()
                fault.data = 'ProcessManager startup busy timeout'
                self.process_fault_callback(fault)
                return
            if reported_state in ('FAILED',):
                with self.state_lock:
                    self._stack_start_deferred = False
                    self._stack_start_deferred_deadline = None
                self.abort_navigation(
                    reason=2,
                    navigation_context=(
                        navigation_generation, current_waypoint_index))
                return
            if reported_state not in ('STOPPED', 'STARTING', 'PARTIAL'):
                self._schedule_deferred_stack_start(
                    (navigation_generation, current_waypoint_index))
                return
            launched = self.launch_new_stack(
                waypoint.map_name,
                resume=True,
                deadline_monotonic=deadline,
            )
            if launched is None:
                if not self._navigation_context_is_current(
                        navigation_generation, current_waypoint_index, waypoint):
                    return
                if deadline is not None and time.monotonic() >= deadline:
                    with self.state_lock:
                        self._stack_start_deferred = False
                        self._stack_start_deferred_deadline = None
                    fault = String()
                    fault.data = 'ProcessManager startup busy timeout'
                    self.process_fault_callback(fault)
                    return
                self._schedule_deferred_stack_start(
                    (navigation_generation, current_waypoint_index))
                return
            with self.state_lock:
                self._stack_start_deferred = False
                self._stack_start_deferred_deadline = None
            if launched:
                if self._navigation_context_is_current(
                        navigation_generation, current_waypoint_index, waypoint):
                    self.send_nav2_goal(
                        waypoint, navigation_generation,
                        current_waypoint_index)
            else:
                self.abort_navigation(
                    reason=2,
                    navigation_context=(
                        navigation_generation, current_waypoint_index))
            return
        if status_kind == 'running':
            with self.state_lock:
                self._stack_start_deferred = False
                self._stack_start_deferred_deadline = None
            status_from_service = getattr(
                self, '_last_status_from_service', False)
            if (status_from_service and
                    getattr(self, '_last_reported_stack_state', '') !=
                    'RUNNING'):
                self.get_logger().error(
                    '导航栈未处于完整 RUNNING 状态，拒绝复用')
                self.abort_navigation(
                    reason=2,
                    navigation_context=(
                        navigation_generation, current_waypoint_index))
                return
            identity_valid = getattr(
                self, '_last_status_has_identity', False)
            if status_from_service and not identity_valid:
                self.get_logger().error(
                    '运行中的导航栈缺少地图/会话身份，拒绝复用')
                self.abort_navigation(
                    reason=2,
                    navigation_context=(
                        navigation_generation, current_waypoint_index))
                return
            if (status_from_service
                    and self._last_reported_stack_map != waypoint.map_name):
                self.get_logger().error(
                    '运行中的导航栈地图与目标不一致，拒绝复用: '
                    f'stack_map={self._last_reported_stack_map!r}, '
                    f'waypoint_map={waypoint.map_name!r}')
                self.abort_navigation(
                    reason=2,
                    navigation_context=(
                        navigation_generation, current_waypoint_index))
                return
            # 等待局部代价地图重建，避免到达航点后立即发送新目标导致障碍物消失
            time.sleep(6.0)
            if self._navigation_context_is_current(
                    navigation_generation, current_waypoint_index, waypoint):
                self.send_nav2_goal(
                    waypoint, navigation_generation, current_waypoint_index)
        elif status_kind == 'stopped':
            self.get_logger().info('首次导航，启动完整导航堆栈...')
            launched = self.launch_new_stack(waypoint.map_name)
            if launched is None:
                self.get_logger().warning(
                    'ProcessManager 正忙，延迟重试启动且不清理现有事务')
                with self.state_lock:
                    self._stack_start_deferred = True
                    if getattr(
                            self, '_stack_start_deferred_deadline', None) is None:
                        self._stack_start_deferred_deadline = (
                            time.monotonic() +
                            self.DEFERRED_STACK_START_TIMEOUT_SEC)
                self._schedule_deferred_stack_start(
                    (navigation_generation, current_waypoint_index))
            elif launched:
                if self._navigation_context_is_current(
                        navigation_generation, current_waypoint_index, waypoint):
                    self.send_nav2_goal(
                        waypoint, navigation_generation, current_waypoint_index)
            else:
                if not self._navigation_context_is_current(
                        navigation_generation, current_waypoint_index, waypoint):
                    self.get_logger().warning('导航任务已故障或取消，不再重试启动导航堆栈')
                    return
                self.get_logger().error('导航堆栈启动失败，安排恢复')
                self.abort_navigation(
                    reason=2, navigation_context=(navigation_generation, current_waypoint_index))
        else:
            self.get_logger().error(
                '导航子栈状态不是全运行或全停止（部分运行/未知），拒绝直接启动')
            self.abort_navigation(
                reason=2, navigation_context=(navigation_generation, current_waypoint_index))

    def _schedule_deferred_stack_start(self, navigation_context):
        """Retry an unknown/busy start without entering destructive recovery."""
        with self.state_lock:
            timer = getattr(self, '_stack_start_retry_timer', None)
            if timer is not None and timer.is_alive():
                return
            retry_generation = getattr(
                self, '_stack_start_retry_generation', 0)

            def retry():
                with self.state_lock:
                    if (retry_generation !=
                            getattr(self, '_stack_start_retry_generation', 0)):
                        return
                    if self._stack_start_retry_timer is not timer:
                        return
                    self._stack_start_retry_timer = None
                self.navigate_to_next_waypoint(
                    navigation_context=navigation_context)

            timer = threading.Timer(
                max(0.25, min(float(self.retry_delay), 2.0)), retry)
            timer.daemon = True
            self._stack_start_retry_timer = timer
            timer.start()

    def _get_stack_transaction_lock(self):
        lock = getattr(self, '_stack_transaction_lock', None)
        if lock is None:
            lock = StackTransactionLock()
            self._stack_transaction_lock = lock
        return lock

    def launch_new_stack(
        self, map_name: str, *, resume: bool = False,
        deadline_monotonic=None,
    ):
        """Own the complete four-process startup or defer without cleanup."""
        if self._stack_start_cancelled(deadline_monotonic):
            return None
        transaction_lock = self._get_stack_transaction_lock()
        if not transaction_lock.acquire():
            self.get_logger().warning(
                '另一个控制器正在变更导航栈，延迟本次启动')
            return None
        try:
            return self._launch_new_stack_owned(
                map_name,
                resume=resume,
                deadline_monotonic=deadline_monotonic,
            )
        finally:
            transaction_lock.release()

    def _launch_new_stack_owned(
        self, map_name: str, *, resume: bool = False,
        deadline_monotonic=None,
    ):
        """
        function: 为指定地图启动新导航堆栈
        param: @map_name: 要启动的地图名称
        return: 启动成功返回True，否则返回False
        -----------
        test: 测试通过
        """
        # 启动顺序: re_localization -> nav2_init_pose -> liosam -> navigation2

        def start_process(process_name, process_map):
            if self._stack_start_cancelled(deadline_monotonic):
                return None
            req = StartProcess.Request()
            req.process_name = process_name
            req.map_name = process_map
            future = self.start_process_client.call_async(req)
            if deadline_monotonic is None:
                response = self._wait_for_future(
                    future, timeout_sec=self.PROCESS_START_TIMEOUT_SEC)
            else:
                response = self._wait_for_future_until(
                    future,
                    timeout_sec=self.PROCESS_START_TIMEOUT_SEC,
                    deadline_monotonic=deadline_monotonic,
                )
            if response is None:
                self.get_logger().warning(
                    f'启动{process_name}结果未知，禁止回滚其他事务')
                return None
            if getattr(response, 'busy', False):
                self.get_logger().warning(
                    f'启动{process_name}暂忙: {response.message}')
                return None
            if not response.success:
                self.get_logger().error(f'启动{process_name}失败')
                return False
            if process_name == 're_localization':
                instance_id = getattr(response, 'instance_id', '')
                active_map = getattr(response, 'active_map', '')
                session_start_ns = int(
                    getattr(response, 'session_start_ns', 0))
                if (not instance_id or active_map != process_map
                        or session_start_ns <= 0):
                    self.get_logger().error(
                        're_localization 启动响应缺少本轮会话证据')
                    return False
                with self.state_lock:
                    changed = (
                        self._active_localization_session != instance_id or
                        self._active_localization_map != active_map or
                        self._localization_session_start_ns != session_start_ns)
                    if changed:
                        self._latest_localization_result = None
                    self._active_localization_session = instance_id
                    self._active_localization_map = active_map
                    self._localization_session_start_ns = session_start_ns
                    self._expected_localization_session = instance_id
                    self._expected_localization_map = active_map
                    self._expected_session_start_ns = session_start_ns
                    pending = self._pending_localization_result
                    self._pending_localization_result = None
                    if pending is not None:
                        self._accept_localization_result_locked(pending)
            self.get_logger().info(f'启动{process_name}成功')
            return True

        # A new stack must never inherit a result/TF from the previous map.
        if not resume:
            self._reset_localization_gate(map_name)

        try:
            # 启动顺序: re_localization -> nav2_init_pose -> TF -> liosam -> navigation2
            result = start_process('re_localization', map_name)
            if result is None:
                return None
            if not result:
                raise RuntimeError('re_localization startup failed')
            result = start_process('nav2_init_pose', map_name)
            if result is None:
                return None
            if not result:
                raise RuntimeError('nav2_init_pose startup failed')

            self.get_logger().info('等待重定位完成并发布 map -> odom TF...')
            if deadline_monotonic is None:
                tf_ready = self.wait_for_map_to_odom_link_tf(
                    timeout_sec=600.0)
            else:
                tf_ready = self.wait_for_map_to_odom_link_tf(
                    timeout_sec=600.0,
                    deadline_monotonic=deadline_monotonic,
                )
            if tf_ready is None:
                return None
            if not tf_ready:
                raise RuntimeError('map -> odom TF startup timeout')
            self.get_logger().info('检测到已发布map -> odom TF...')

            result = start_process('liosam', '')
            if result is None:
                return None
            if not result:
                raise RuntimeError('liosam startup failed')
            result = start_process('navigation2', map_name)
            if result is None:
                return None
            if not result:
                raise RuntimeError('navigation2 startup failed')
        except Exception as exc:
            self.get_logger().error(f'导航堆栈启动事务失败: {exc}')
            # The process manager owns process groups; use its verified
            # shutdown barrier before allowing a retry.
            with self.state_lock:
                gate_was_active = self._navigation_cleanup_active
                self._navigation_cleanup_active = True
            try:
                rollback_ok = self.shutdown_all_processes_service()
            except Exception as rollback_exc:
                self.get_logger().error(
                    f'启动失败后的回滚服务异常: {rollback_exc}')
                rollback_ok = False
            self._last_stack_rollback_verified = rollback_ok is not False
            if rollback_ok is not False:
                with self.state_lock:
                    if not gate_was_active:
                        self._navigation_cleanup_active = False
            else:
                self.get_logger().error('启动失败后的导航堆栈回滚未确认干净')
                self._latch_cleanup_failure('导航堆栈启动回滚失败')
            return False

        self._last_stack_rollback_verified = True
        self.get_logger().info('Navigation2关键节点和生命周期状态均已就绪')
        return True

    def _reset_localization_gate(self, map_name: str):
        """Invalidate all localization evidence before a new stack starts."""
        with self.state_lock:
            self._active_localization_session = ''
            self._active_localization_map = map_name or ''
            self._localization_session_start_ns = 0
            self._expected_localization_session = ''
            self._expected_localization_map = map_name or ''
            self._expected_session_start_ns = 0
            self._latest_localization_result = None
            self._pending_localization_result = None
            self._startup_footprint_verified_session = ''

    @staticmethod
    def _stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def localization_session_callback(self, message: LocalizationSession):
        """Track the exact process-manager generation used by this task."""
        with self.state_lock:
            if not message.active:
                if (not message.session_id
                        or message.session_id == self._active_localization_session):
                    self._active_localization_session = ''
                    self._localization_session_start_ns = 0
                    self._latest_localization_result = None
                    self._pending_localization_result = None
                    self._startup_footprint_verified_session = ''
                return

            changed = (
                message.session_id != self._active_localization_session
                or message.map_id != self._active_localization_map
                or int(message.session_start_ns) != self._localization_session_start_ns
            )
            if changed:
                self._latest_localization_result = None
                self._startup_footprint_verified_session = ''
            self._active_localization_session = message.session_id
            self._active_localization_map = message.map_id
            self._localization_session_start_ns = int(message.session_start_ns)

            pending = self._pending_localization_result
            self._pending_localization_result = None
            if pending is not None:
                self._accept_localization_result_locked(pending)

    def localization_result_callback(self, message: LocalizationResult):
        with self.state_lock:
            if not self._active_localization_session:
                self._pending_localization_result = message
                return
            self._accept_localization_result_locked(message)

    def _accept_localization_result_locked(self, message: LocalizationResult):
        if (message.session_id != self._active_localization_session
                or message.map_id != self._active_localization_map):
            return False
        if not message.success:
            self._latest_localization_result = message
            return False
        cloud_stamp_ns = self._stamp_to_ns(message.cloud_stamp)
        if cloud_stamp_ns < self._localization_session_start_ns:
            self.get_logger().warning(
                '忽略会话开始前的定位结果点云: '
                f'{cloud_stamp_ns} < {self._localization_session_start_ns}')
            return False
        if not math.isfinite(float(message.fitness_score)):
            return False
        if float(message.fitness_score) > 0.05:
            return False
        self._latest_localization_result = message
        return True

    def _localization_result_ready(self, map_name: str):
        with self.state_lock:
            result = self._latest_localization_result
            session_id = self._active_localization_session
            session_map = self._active_localization_map
            session_start_ns = self._localization_session_start_ns
            expected_session = getattr(
                self, '_expected_localization_session', '')
            expected_map = getattr(self, '_expected_localization_map', '')
            expected_start_ns = getattr(
                self, '_expected_session_start_ns', 0)
        if (not expected_session or expected_map != map_name
                or expected_start_ns <= 0):
            return None
        if (session_id != expected_session or session_map != expected_map
                or session_start_ns != expected_start_ns or result is None):
            return None
        if (result.session_id != expected_session or result.map_id != map_name
                or not result.success):
            return None
        if self._stamp_to_ns(result.cloud_stamp) < expected_start_ns:
            return None
        return result

    @staticmethod
    def _transform_yaw(transform):
        q = transform.transform.rotation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _static_footprint_checker(self, map_name):
        cache = getattr(self, '_static_footprint_checkers', None)
        if cache is None:
            cache = {}
            self._static_footprint_checkers = cache
        checker = cache.get(map_name)
        if checker is None:
            map_directory = getattr(
                self, 'map_directory',
                '/home/akun/workspace/Car_jetson/multi_map_navigation/src/map')
            checker = StaticMapFootprintChecker(
                f'{map_directory}/{map_name}/{map_name}_clean.yaml')
            cache[map_name] = checker
        return checker

    def _wait_for_startup_static_footprint(
        self, map_name, *, required_samples=10, sample_period=0.2,
        timeout_sec=12.0,
    ):
        """Require about two seconds of legal map->base_link footprint samples."""
        session_id = getattr(self, '_active_localization_session', '')
        if not session_id:
            self.get_logger().error('首目标足迹门禁缺少活动定位会话')
            return False
        if getattr(self, '_startup_footprint_verified_session', '') == session_id:
            return True

        try:
            checker = self._static_footprint_checker(map_name)
        except Exception as exc:
            self.get_logger().error(f'加载静态地图足迹门禁失败: {exc}')
            return False
        tf_buffer = getattr(self, '_startup_tf_buffer', None)
        if tf_buffer is None:
            tf_buffer = tf2_ros.Buffer()
            self._startup_tf_buffer = tf_buffer
            self._startup_tf_listener = tf2_ros.TransformListener(
                tf_buffer, self)

        deadline = time.monotonic() + timeout_sec
        consecutive = 0
        last_stamp_ns = 0
        last_reason = 'no map -> base_link transform'
        while time.monotonic() < deadline:
            if self._stack_start_cancelled():
                return False
            try:
                transform = tf_buffer.lookup_transform(
                    target_frame='map', source_frame='base_link',
                    time=rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
                stamp_ns = self._stamp_to_ns(transform.header.stamp)
                now_ns = self.get_clock().now().nanoseconds
                fresh = (-500_000_000 <= now_ns - stamp_ns
                         <= 2_000_000_000 and stamp_ns > last_stamp_ns)
                if fresh:
                    legal, last_reason = checker.check_rectangular_footprint(
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        self._transform_yaw(transform),
                        half_length=0.43,
                        half_width=0.28,
                        padding=0.07,
                    )
                    last_stamp_ns = stamp_ns
                    consecutive = consecutive + 1 if legal else 0
                    self.get_logger().debug(
                        '首目标静态足迹合法样本 '
                        f'{consecutive}/{required_samples}: {last_reason}')
                    if consecutive >= required_samples:
                        with self.state_lock:
                            if (session_id != getattr(
                                    self, '_active_localization_session', '')):
                                return False
                            self._startup_footprint_verified_session = session_id
                        self.get_logger().info(
                            'map -> base_link 完整足迹连续10次合法，允许首目标')
                        return True
            except (LookupException, ConnectivityException,
                    ExtrapolationException):
                consecutive = 0
            except Exception as exc:
                consecutive = 0
                last_reason = str(exc)
            time.sleep(sample_period)

        self.get_logger().error(
            f'首目标静态足迹连续合法门禁失败: {last_reason}')
        return False

    @staticmethod
    def _pose_matches_transform(pose, transform, tolerance=0.20):
        dx = pose.position.x - transform.transform.translation.x
        dy = pose.position.y - transform.transform.translation.y
        dz = pose.position.z - transform.transform.translation.z
        if math.sqrt(dx * dx + dy * dy + dz * dz) > tolerance:
            return False
        a = pose.orientation
        b = transform.transform.rotation
        norm_a = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w)
        norm_b = math.sqrt(b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w)
        if norm_a < 1e-6 or norm_b < 1e-6:
            return False
        dot = abs(
            a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
        return dot / (norm_a * norm_b) >= math.cos(math.radians(10.0))

    def _transform_is_from_current_session(self, transform):
        stamp_ns = self._stamp_to_ns(transform.header.stamp)
        with self.state_lock:
            session_start_ns = getattr(
                self, '_expected_session_start_ns', 0)
        if session_start_ns <= 0 or stamp_ns < session_start_ns:
            return False
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        return -500_000_000 <= age_ns <= 2_000_000_000
    
    def _wait_for_future(self, future, timeout_sec: float):
        """等待 future 完成，由 MultiThreadedExecutor 在其他线程处理回调"""
        start = time.time()
        while not future.done() and time.time() - start < timeout_sec:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error(f'服务调用等待超时 ({timeout_sec:.1f}s)')
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f'异步调用失败: {exc}')
            return None

    def _stack_start_cancelled(self, deadline_monotonic=None):
        if (deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic):
            return True
        lock = getattr(self, 'state_lock', None)
        if lock is None or not hasattr(self, 'is_navigating'):
            return False
        with lock:
            return (
                bool(getattr(self, 'stack_fault_active', False))
                or not bool(self.is_navigating)
            )

    def _wait_for_future_until(
        self, future, timeout_sec: float, *, deadline_monotonic=None
    ):
        """Wait for a startup response while honoring task cancellation."""
        wait_deadline = time.monotonic() + timeout_sec
        if deadline_monotonic is not None:
            wait_deadline = min(wait_deadline, deadline_monotonic)
        while not future.done() and time.monotonic() < wait_deadline:
            if self._stack_start_cancelled(deadline_monotonic):
                cancel = getattr(future, 'cancel', None)
                if callable(cancel):
                    cancel()
                return None
            time.sleep(0.05)
        if self._stack_start_cancelled(deadline_monotonic):
            cancel = getattr(future, 'cancel', None)
            if callable(cancel):
                cancel()
            return None
        if not future.done():
            cancel = getattr(future, 'cancel', None)
            if callable(cancel):
                cancel()
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f'异步调用失败: {exc}')
            return None

    def wait_for_map_to_odom_link_tf(
        self, timeout_sec: float = 300.0, check_interval: float = 0.5,
        deadline_monotonic=None,
    ):
        """
        function: 等待 map -> odom TF 变换可用
        param: @ timeout_sec: 等待时间
               @ check_interval: 检查间隔
        return: 成功返回True, 失败返回False
        ---------------
        test: 测试通过
        """
        self.get_logger().info("等待本轮定位结果及 map -> odom TF 变换就绪...")

        if self._stack_start_cancelled(deadline_monotonic):
            self.get_logger().warning('导航任务已故障、取消或启动等待超时')
            return None

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self)

        wait_deadline = time.monotonic() + timeout_sec
        if deadline_monotonic is not None:
            wait_deadline = min(wait_deadline, deadline_monotonic)

        consecutive_matches = 0
        while time.monotonic() < wait_deadline:
            if self._stack_start_cancelled(deadline_monotonic):
                self.get_logger().warning(
                    '导航任务已故障、取消或启动等待超时')
                return None

            result = self._localization_result_ready(
                getattr(self, '_active_localization_map', None))
            if result is None:
                consecutive_matches = 0
                self.get_logger().debug('等待本轮成功定位结果...')
                time.sleep(check_interval)
                continue

            try:
                # 尝试查询最新的变换（time=0 表示 latest）
                trans: TransformStamped = tf_buffer.lookup_transform(
                    target_frame="map",
                    source_frame="odom",
                    time=rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)  # 小超时避免阻塞
                )
                if (self._transform_is_from_current_session(trans)
                        and self._pose_matches_transform(result.pose, trans)):
                    consecutive_matches += 1
                    self.get_logger().debug(
                        f"本轮 map -> odom 一致样本 {consecutive_matches}/3")
                    if consecutive_matches >= 3:
                        self.get_logger().info(
                            '本轮定位结果与 map -> odom TF 连续一致，TF 已就绪')
                        return True
                else:
                    consecutive_matches = 0
                    self.get_logger().warning(
                        'TF 与本轮定位结果不一致，拒绝使用该 TF')

            except (LookupException, ConnectivityException, ExtrapolationException):
                # 正常等待中，不报错
                self.get_logger().debug("map -> odom TF 尚未可用，继续等待...")
            
            except Exception as ex:
                self.get_logger().warn(f"TF 查询异常: {ex}")

            # 避免 CPU 空转
            time.sleep(check_interval)
        if (deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic):
            self.get_logger().warning('导航栈延迟启动等待已到截止时间')
            return None
        self.get_logger().error(f"等待 map -> odom TF 超时 ({timeout_sec}s)")
        return False

    def is_map_switch_point(self, waypoint: Waypoint) -> bool:
        """
        检查航点是否为地图切换点

        参数:
            waypoint: 要检查的航点

        返回:
            如果航点是地图切换点返回True，否则返回False
        """
        return waypoint.type == 4

    def is_common_point(self, waypoint: Waypoint) -> bool:
        """
        检查航点是否为普通导航点

        参数:
            waypoint: 要检查的航点

        返回:
            如果航点是普通导航换点返回True，否则返回False
        """
        return waypoint.type == 1

    def is_trafficlight_point(self, waypoint: Waypoint) -> bool:
        """
        检查航点是否为红绿灯点

        参数:
            waypoint: 要检查的航点

        返回:
            如果航点是地图切换点返回True，否则返回False
        """
        return waypoint.type == 2

    def is_charge_point(self, waypoint: Waypoint) -> bool:
        """
        检查航点是否为充电点

        参数:
            waypoint: 要检查的航点

        返回:
            如果航点是充电点返回True，否则返回False
        """
        return waypoint.type == 3

    def trigger_map_switch(self, waypoint: Waypoint):
        """
        function: 发布话题, 触发地图切换操作
        param: @waypoint: 地图切换航点
        return: 
        -----------
        test: 测试通过
        """
        transaction_id = uuid.uuid4().hex
        with self.state_lock:
            self.is_map_switching = True
            self._active_map_switch_transaction_id = transaction_id
            current_map = self.current_map
            current_waypoint_index = self.current_waypoint_index
            waypoint_list = self.waypoint_list
            navigation_generation = getattr(self, '_navigation_generation', 0)
            self._active_map_switch_context = (
                transaction_id,
                navigation_generation,
                current_waypoint_index,
                waypoint,
            )

        # 创建地图切换触发消息
        trigger_msg = MapSwitchTrigger()
        trigger_msg.header.stamp = self.get_clock().now().to_msg()
        trigger_msg.transaction_id = transaction_id
        trigger_msg.current_map = current_map
        trigger_msg.next_map = waypoint.next_map_name

        # 创建切换位姿（使用当前点在下一张地图坐标系下的坐标）
        from geometry_msgs.msg import PoseStamped, Quaternion

        switch_pose = PoseStamped()
        switch_pose.header.stamp = self.get_clock().now().to_msg()
        switch_pose.header.frame_id = "map"
        switch_pose.pose.position.x = waypoint.next_x
        switch_pose.pose.position.y = waypoint.next_y
        switch_pose.pose.position.z = 0.0

        # 将yaw角转换为四元数
        q = _quaternion_from_yaw(waypoint.next_yaw)
        switch_pose.pose.orientation.x = q[0]
        switch_pose.pose.orientation.y = q[1]
        switch_pose.pose.orientation.z = q[2]
        switch_pose.pose.orientation.w = q[3]

        trigger_msg.switch_pose = switch_pose
        trigger_msg.current_waypoint_id = waypoint.id

        # 如果可用，获取下一个航点ID
        if current_waypoint_index + 1 < len(waypoint_list):
            trigger_msg.next_waypoint_id = waypoint_list[current_waypoint_index + 1].id
        else:
            trigger_msg.next_waypoint_id = -1

        # 发布触发
        self.map_switch_trigger_pub.publish(trigger_msg)

        self.get_logger().info(
            f'已触发地图切换: {trigger_msg.current_map} -> {trigger_msg.next_map}'
        )

    def map_switch_complete_callback(self, msg: MapSwitchResult):
        """
        function: 地图切换完成的回调
        param: @ msg: 消息
        return:
        ----------------
        test: 测试通过
        """
        with self.state_lock:
            transaction_id = getattr(msg, 'transaction_id', '')
            switch_context = getattr(
                self, '_active_map_switch_context', None)
            if (not self.is_map_switching
                    or not transaction_id
                    or transaction_id != getattr(
                        self, '_active_map_switch_transaction_id', '')
                    or switch_context is None
                    or switch_context[0] != transaction_id
                    or not self._navigation_context_is_current(
                        switch_context[1], switch_context[2],
                        switch_context[3])):
                self.get_logger().warning(
                    '忽略不属于当前事务的地图切换结果')
                return
            self._active_map_switch_transaction_id = ''
            self._active_map_switch_context = None
            navigation_generation = switch_context[1]
            waypoint_index = switch_context[2]
            waypoint = switch_context[3]
            self.publish_robot_state("running")

            if not msg.success:
                failure_context = (navigation_generation, waypoint_index)
                next_context = None
            else:
                self.is_map_switching = False
                self.current_map = waypoint.next_map_name
                self.get_logger().info('地图切换成功完成')
                failure_context = None
                # Advance while the exact task/waypoint identity remains locked.
                # A late result can therefore never mutate a replacement task.
                self._advance_current_waypoint()
                next_context = (
                    navigation_generation, self.current_waypoint_index)

        if msg.success:
            self.navigate_to_next_waypoint(navigation_context=next_context)
        else:
            self.get_logger().error('地图切换失败，安排恢复')
            self.abort_navigation(
                reason=2, navigation_context=failure_context)

    def _navigation_context_is_current(self, generation, waypoint_index, waypoint=None):
        """Reject work resumed after its task or waypoint has been replaced."""
        with self.state_lock:
            if (generation != getattr(self, '_navigation_generation', 0)
                    or self.stack_fault_active or not self.is_navigating
                    or getattr(self, '_navigation_cleanup_active', False)
                    or getattr(self, 'docking_hold', False)
                    or not self.waypoint_list
                    or waypoint_index != self.current_waypoint_index
                    or not 0 <= waypoint_index < len(self.waypoint_list)):
                return False
            return waypoint is None or self.waypoint_list[waypoint_index] is waypoint

    def _goal_token_is_current_locked(self, token):
        return (
            token is not None
            and token == getattr(self, '_active_goal_token', None)
            and self._navigation_context_is_current(token[0], token[1])
        )

    def _route_map_consistency_gate(
            self, waypoint, navigation_generation, waypoint_index):
        """Reject a goal until the map-switch transaction owns its map."""
        with self.state_lock:
            if not self._navigation_context_is_current(
                    navigation_generation, waypoint_index, waypoint):
                return False

            active_map = getattr(self, 'current_map', None)
            waypoint_map = getattr(waypoint, 'map_name', None)
            if active_map == waypoint_map:
                return True

            reason = (
                'ROUTE_MAP_MISMATCH: Nav2 goal rejected; '
                f'active_map={active_map!r}, waypoint_map={waypoint_map!r}, '
                f'waypoint_index={waypoint_index}. The map-switch flow must '
                'complete before dispatching this goal.'
            )
            self.stack_fault_active = True
            self._navigation_cleanup_active = True
            self._navigation_generation = getattr(
                self, '_navigation_generation', 0) + 1
            self._active_goal_token = None
            self.pending_replan_endpoints = None
            goal_handle = getattr(self, 'current_goal_handle', None)
            self.current_goal_handle = None
            self._reset_stack_recovery()

        self.get_logger().error(reason)
        self.publish_robot_state('fault')
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(
                    f'取消地图不一致时的Nav2目标失败: {exc}')

        # Reuse the verified stack-cleanup path.  Do not publish cmd_vel or a
        # parking command here: the goal was rejected before vehicle control.
        threading.Thread(
            target=self._cleanup_failed_stack,
            args=(reason,),
            daemon=True,
        ).start()
        return False

    def send_nav2_goal(self, waypoint: Waypoint, navigation_generation=None,
                       waypoint_index=None):
        """
        function: 向Navigation2发送导航目标
        param: @ waypoint: 目标航点
        ---------
        test: 测试通过
        """

        with self.state_lock:
            if navigation_generation is None:
                navigation_generation = getattr(self, '_navigation_generation', 0)
            if waypoint_index is None:
                waypoint_index = self.current_waypoint_index
        if not self._route_map_consistency_gate(
                waypoint, navigation_generation, waypoint_index):
            return

        active_session = getattr(self, '_active_localization_session', '')
        if (active_session
                and getattr(self, '_startup_footprint_verified_session', '')
                != active_session
                and not self._wait_for_startup_static_footprint(
                    waypoint.map_name)):
            fault = String()
            fault.data = 'STARTUP_STATIC_FOOTPRINT_GATE_FAILED'
            self.process_fault_callback(fault)
            return

        # 等待动作服务器
        self.get_logger().info('等待nav2服务器...')
        if not self.nav2_client.wait_for_server(timeout_sec=50.0):
            self.get_logger().error('Navigation2动作服务器不可用')
            if self._navigation_context_is_current(
                    navigation_generation, waypoint_index, waypoint):
                self.abort_navigation(
                    reason=2, navigation_context=(navigation_generation, waypoint_index))
            return
            
        # 创建目标消息
        from geometry_msgs.msg import PoseStamped

        goal_msg = NavigateToPose.Goal()

        # 构建位姿
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = 0.0

        # 将yaw角转换为四元数
        q = _quaternion_from_yaw(waypoint.yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal_msg.pose = pose
        goal_msg.behavior_tree = self.navigation_behavior_tree(waypoint)

        self.get_logger().info(
            f'向Navigation2发送目标: '
            f'x={waypoint.x:.2f}, '
            f'y={waypoint.y:.2f}, '
            f'yaw={waypoint.yaw:.2f}'
        )
        
        with self.state_lock:
            # Recheck under the same lock used for send_goal_async.  A map
            # switch or stale callback may have changed ownership while the
            # action server was becoming available.
            if not self._route_map_consistency_gate(
                    waypoint, navigation_generation, waypoint_index):
                return
            self._goal_sequence = getattr(self, '_goal_sequence', 0) + 1
            goal_token = (navigation_generation, waypoint_index, self._goal_sequence)
            self._active_goal_token = goal_token
            self.publish_robot_state("running")
            try:
                # Keep the final state check and action request atomic with fault handling.
                send_goal_future = self.nav2_client.send_goal_async(
                    goal_msg,
                    feedback_callback=self.nav2_feedback_callback
                )
            except Exception as exc:
                self._active_goal_token = None
                self.get_logger().error(f'发送导航目标失败: {exc}')
                fault = String()
                fault.data = f'navigation2 action goal request failed: {exc}'
                self.process_fault_callback(fault)
                return
        send_goal_future.add_done_callback(
            lambda future: self.nav2_goal_response_callback(future, goal_token))

    def _fault_for_nav2_goal(self, goal_token, reason):
        with self.state_lock:
            if not self._goal_token_is_current_locked(goal_token):
                return
            fault = String()
            fault.data = reason
            self.process_fault_callback(fault)

    def nav2_goal_response_callback(self, future, goal_token):
        """
        function: Navigation2目标响应回调
        -------------
        test: 测试通过
        """
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'发送导航目标失败: {exc}')
            self._fault_for_nav2_goal(
                goal_token, f'navigation2 action goal request failed: {exc}')
            return

        if goal_handle is None or not goal_handle.accepted:
            with self.state_lock:
                if not self._goal_token_is_current_locked(goal_token):
                    return
                self._active_goal_token = None
            self.get_logger().error('导航目标被拒绝')
            self.abort_navigation(reason=2, navigation_context=goal_token[:2])
            return

        with self.state_lock:
            if not self._goal_token_is_current_locked(goal_token):
                stale = True
            else:
                stale = False
                self.get_logger().info('导航目标已接受')
                self.current_goal_handle = goal_handle
        if stale:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'取消过期Nav2目标失败: {exc}')
            return

        # 等待结果
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda result: self.nav2_result_callback(result, goal_token))
        except Exception as exc:
            self._fault_for_nav2_goal(
                goal_token, f'navigation2 action result request failed: {exc}')

    def nav2_feedback_callback(self, feedback_msg):
        """Navigation2反馈回调"""
        feedback = feedback_msg.feedback
        # 记录导航进度
        self.get_logger().debug(
            f'导航反馈: distance_remaining={feedback.distance_remaining:.2f}m'
        )

    def nav2_result_callback(self, future, goal_token):
        """
        function: Navigation2结果回调
        param: 
        return:
        -----------
        test: 测试通过
        """
        with self.state_lock:
            if not self._goal_token_is_current_locked(goal_token):
                self.get_logger().warning('Nav2目标已过期，忽略迟到的结果')
                return

        # Get the result wrapper
        try:
            result_wrapper = future.result()
        except Exception as exc:
            self.get_logger().error(f'获取Nav2结果失败: {exc}')
            self._fault_for_nav2_goal(
                goal_token, f'navigation2 action result failed: {exc}')
            return
        if result_wrapper is None:
            self.get_logger().error('Nav2返回了空结果')
            self._fault_for_nav2_goal(
                goal_token, 'navigation2 action returned an empty result')
            return
        with self.state_lock:
            if not self._goal_token_is_current_locked(goal_token):
                return
            self.publish_robot_state("running")

        # 使用ros2 interface show action_msgs/msg/GoalStatus查看
        # status=4 means SUCCEEDED in action_msgs.msg.GoalStatus
        if result_wrapper.status == 4:
            self.get_logger().info('导航目标成功完成 (SUCCEEDED)')
            # print("[TEST] 测试通过")
            self.on_goal_reached(goal_token)
        elif result_wrapper.status == 5:
            # print("[TEST] 测试通过")
            self.get_logger().info('导航目标被取消 (CANCELED)')
            with self.state_lock:
                if not self._goal_token_is_current_locked(goal_token):
                    return
                self._active_goal_token = None
                self.current_goal_handle = None
            self.abort_navigation(reason=0, navigation_context=goal_token[:2])
        else:
            # print("[TEST] 测试通过")
            self.get_logger().error(f'导航目标失败，状态码: {result_wrapper.status}')
            with self.state_lock:
                if not self._goal_token_is_current_locked(goal_token):
                    return
                self._active_goal_token = None
                self.current_goal_handle = None
            self.abort_navigation(reason=1, navigation_context=goal_token[:2])

    def on_goal_reached(self, goal_token=None, navigation_context=None):
        """
        function: 处理成功到达目标。
        param:
        return:
        -----------------
        test: 测试通过
        """
        with self.state_lock:
            if goal_token is not None and not self._goal_token_is_current_locked(goal_token):
                return
            if (navigation_context is not None
                    and not self._navigation_context_is_current(*navigation_context)):
                return
            waypoint_list = self.waypoint_list
            current_waypoint_index = self.current_waypoint_index
            current_map = self.current_map
            navigation_generation = getattr(self, '_navigation_generation', 0)
            if not waypoint_list or not 0 <= current_waypoint_index < len(waypoint_list):
                return
            if goal_token is not None:
                self._active_goal_token = None
                self.current_goal_handle = None
            waypoint = waypoint_list[current_waypoint_index]
            # Charge handling precedes final-waypoint completion: P0 is only
            # the start of docking and must keep localization alive.
            if self.is_charge_point(waypoint):
                if not getattr(self, 'docking_hold', False):
                    self.start_docking(waypoint)
                return
            self.get_logger().info(
                f'到达航点 {current_waypoint_index + 1}/{len(waypoint_list)}')
            finished = current_waypoint_index == len(waypoint_list) - 1
            if not finished:
                if self.is_map_switch_point(waypoint):
                    if current_map != waypoint_list[current_waypoint_index + 1].map_name:
                        self.publish_robot_state("running")
                        self.trigger_map_switch(waypoint)
                        return
                    delay = False
                elif self.is_common_point(waypoint):
                    delay = True
                elif self.is_trafficlight_point(waypoint):
                    self._advance_current_waypoint()
                    # TODO: resume from a traffic-light event.
                    return
                else:
                    return
                self._advance_current_waypoint()
                next_context = (navigation_generation, self.current_waypoint_index)

        if finished:
            self.complete_navigation(expected_generation=navigation_generation)
            return
        # Never hold state_lock while sleeping or waiting on stack services.
        if delay:
            time.sleep(6.0)
        self.navigate_to_next_waypoint(navigation_context=next_context)
    
    def complete_navigation(self, expected_generation=None):
        """
        function: 完成导航序列,关闭所有进程
        param:
        return:
        ----------
        test: 测试通过
        """
        self.get_logger().info('导航结束，关闭所有进程')
        with self.state_lock:
            if (expected_generation is not None
                    and expected_generation != getattr(self, '_navigation_generation', 0)):
                return
            map_switch_transaction_id = getattr(
                self, '_active_map_switch_transaction_id', '')
            self._active_map_switch_transaction_id = ''
            self._active_map_switch_context = None
            self._navigation_generation = getattr(self, '_navigation_generation', 0) + 1
            self._active_goal_token = None
            self._navigation_cleanup_active = True
            self.is_navigating = False
            self.is_map_switching = False
            self.waypoint_list = None
            self.current_waypoint_index = 0
            self.current_goal_handle = None
            self.pending_replan_endpoints = None
            self._reset_stack_recovery()

        self._publish_map_switch_cancel(
            'navigation_complete', map_switch_transaction_id)

        # Invalidate the task before service calls, which may wait for a response.
        try:
            cleanup_ok = self.shutdown_all_processes_service()
        except Exception as exc:
            self.get_logger().error(f'导航完成后的清理异常: {exc}')
            cleanup_ok = False
        if cleanup_ok is self.CLEANUP_DEFERRED:
            self._schedule_deferred_cleanup(
                mode='complete', reason='导航完成后的堆栈清理')
            return None
        # Existing unit-test doubles returned None before this API had a bool
        # contract.  Production now always returns True/False; only explicit
        # False is retained as a compatibility failure boundary here.
        cleanup_verified = cleanup_ok is not False
        if not cleanup_verified:
            self._latch_cleanup_failure('导航完成后的堆栈清理未确认干净')
            return False

        with self.state_lock:
            self._navigation_cleanup_active = False
            self.stack_fault_active = False
        self.publish_robot_state("idle")
        return True

    def process_fault_callback(self, msg: String):
        """Fail the active task deterministically when a required stack node dies."""
        with self.state_lock:
            if not self.is_navigating or self.stack_fault_active:
                return
            self.stack_fault_active = True
            self._navigation_cleanup_active = True
            self._navigation_generation = getattr(self, '_navigation_generation', 0) + 1
            self._active_goal_token = None
            self._reset_stack_recovery()
            self.pending_replan_endpoints = None
            goal_handle = self.current_goal_handle
            map_switch_transaction_id = getattr(
                self, '_active_map_switch_transaction_id', '')
            self._active_map_switch_transaction_id = ''
            self._active_map_switch_context = None

        self.get_logger().error(f'检测到导航栈故障: {msg.data}')
        self._publish_map_switch_cancel(
            'navigation_fault', map_switch_transaction_id)
        self.stop_docking_for_fault()
        self.publish_robot_state('fault')

        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'取消Nav2目标失败: {exc}')

        stop = Twist()
        for _ in range(3):
            self.emergency_cmd_vel_pub.publish(stop)

        # Service calls wait for responses, so clean up on a worker instead of
        # blocking the ROS subscription callback that received the fault.
        threading.Thread(
            target=self._cleanup_failed_stack,
            args=(msg.data,),
            daemon=True
        ).start()

    def _cleanup_failed_stack(self, reason: str):
        try:
            cleanup_ok = self.shutdown_all_processes_service()
        except Exception as exc:
            self.get_logger().error(f'故障后的清理异常: {exc}')
            cleanup_ok = False
        if cleanup_ok is self.CLEANUP_DEFERRED:
            with self.state_lock:
                self.is_navigating = False
                self.is_map_switching = False
                self.current_goal_handle = None
                self.waypoint_list = None
                self.current_waypoint_index = 0
                self._navigation_cleanup_active = True
            self.publish_robot_state('fault')
            self._schedule_deferred_cleanup(mode='fault', reason=reason)
            return
        self._finish_fault_cleanup(reason, cleanup_ok)

    def _finish_fault_cleanup(self, reason: str, cleanup_ok):
        """Apply the terminal state after a fault cleanup attempt."""
        cleanup_verified = cleanup_ok is not False
        with self.state_lock:
            self.is_navigating = False
            self.is_map_switching = False
            self._active_map_switch_transaction_id = ''
            self._active_map_switch_context = None
            self.current_goal_handle = None
            self.waypoint_list = None
            self.current_waypoint_index = 0
            self._navigation_cleanup_active = not cleanup_verified
        self.publish_robot_state('fault')
        if cleanup_verified:
            self.get_logger().error(f'导航任务因栈故障终止，等待新任务: {reason}')
        else:
            self.get_logger().error(
                f'导航任务因栈故障终止且清理未确认，保持故障门禁: {reason}')

    def _schedule_deferred_cleanup(self, *, mode: str, reason: str):
        """Wait for the active stack transaction, then clean up exactly once."""
        lock = getattr(self, '_deferred_cleanup_lock', None)
        if lock is None:
            lock = threading.Lock()
            self._deferred_cleanup_lock = lock

        with lock:
            existing = getattr(self, '_deferred_cleanup_thread', None)
            if existing is not None and existing.is_alive():
                return

            def cleanup_worker():
                try:
                    cleanup_ok = False
                    transaction_lock = self._get_stack_transaction_lock()
                    try:
                        if transaction_lock.acquire(
                                timeout_sec=self.DEFERRED_CLEANUP_WAIT_SEC):
                            try:
                                cleanup_ok = self._shutdown_all_processes_owned()
                            finally:
                                transaction_lock.release()
                        else:
                            self.get_logger().error(
                                '等待导航栈事务结束超时，无法执行延迟清理')
                    except Exception as exc:
                        self.get_logger().error(f'延迟清理异常: {exc}')

                    if mode == 'complete':
                        if cleanup_ok is False:
                            self._latch_cleanup_failure(
                                '导航完成后的延迟堆栈清理未确认干净')
                        else:
                            with self.state_lock:
                                self._navigation_cleanup_active = False
                                self.stack_fault_active = False
                            self.publish_robot_state('idle')
                    else:
                        self._finish_fault_cleanup(reason, cleanup_ok)
                finally:
                    with lock:
                        if (self._deferred_cleanup_thread is
                                threading.current_thread()):
                            self._deferred_cleanup_thread = None

            thread = threading.Thread(target=cleanup_worker, daemon=True)
            self._deferred_cleanup_thread = thread
            thread.start()

    def _latch_cleanup_failure(self, reason: str):
        """Keep the manager faulted and reject tasks after unverified cleanup."""
        with self.state_lock:
            self.stack_fault_active = True
            self._navigation_cleanup_active = True
            self.is_navigating = False
            self.is_map_switching = False
            self._active_goal_token = None
            self.current_goal_handle = None
            self.waypoint_list = None
            self.current_waypoint_index = 0
            self.pending_replan_endpoints = None
            self._stack_recovery_token = None
        self.publish_robot_state('fault')
        self.get_logger().error(f'导航清理屏障进入故障锁定: {reason}')

    def _publish_map_switch_cancel(
        self, reason: str, transaction_id: str = ''
    ):
        if not transaction_id:
            return
        publisher = getattr(self, 'map_switch_cancel_pub', None)
        if publisher is None:
            return
        message = MapSwitchCancel()
        message.transaction_id = transaction_id
        message.reason = reason
        publisher.publish(message)
        
    def _reset_stack_recovery(self):
        """Invalidate queued recovery when a route starts, ends, or faults."""
        with self.state_lock:
            start_retry_timer = getattr(
                self, '_stack_start_retry_timer', None)
            self._stack_start_retry_timer = None
            self._stack_start_retry_generation = getattr(
                self, '_stack_start_retry_generation', 0) + 1
            self._stack_start_deferred = False
            self._stack_start_deferred_deadline = None
            if start_retry_timer is not None:
                start_retry_timer.cancel()
            self._stack_recovery_generation = getattr(
                self, '_stack_recovery_generation', 0) + 1
            self._stack_recovery_context = None
            self._stack_recovery_attempts = 0
            self._stack_recovery_token = None
            timer = getattr(self, '_stack_recovery_timer', None)
            self._stack_recovery_timer = None
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)

    def _advance_current_waypoint(self):
        with self.state_lock:
            self.current_waypoint_index += 1
            self._active_goal_token = None
            self.current_goal_handle = None
            self._reset_stack_recovery()

    def _stack_recovery_is_current(self, token):
        """Caller holds state_lock; old timers cannot act on a newer waypoint."""
        return (
            token == getattr(self, '_stack_recovery_token', None)
            and token[:2] == (
                self._stack_recovery_generation, self.current_waypoint_index)
            and self.is_navigating
            and not self.stack_fault_active
            and not getattr(self, 'docking_hold', False)
            and bool(self.waypoint_list)
            and 0 <= self.current_waypoint_index < len(self.waypoint_list)
        )

    def _schedule_stack_recovery(self) -> bool:
        """Share a bounded restart budget across all failures of one waypoint."""
        with self.state_lock:
            if (not self.is_navigating or self.stack_fault_active
                    or getattr(self, 'docking_hold', False)
                    or not self.waypoint_list
                    or not 0 <= self.current_waypoint_index < len(self.waypoint_list)):
                return False
            if self._stack_recovery_token is not None:
                return True
            context = (self._stack_recovery_generation, self.current_waypoint_index)
            if context != self._stack_recovery_context:
                self._stack_recovery_context = context
                self._stack_recovery_attempts = 0
            if self._stack_recovery_attempts >= self.max_retries:
                failure = (
                    f'navigation stack recovery exhausted: waypoint='
                    f'{self.current_waypoint_index}, max_retries={self.max_retries}')
            else:
                self._stack_recovery_attempts += 1
                token = (*context, self._stack_recovery_attempts)
                self._stack_recovery_token = token
                retry_map_switch = self.is_map_switching
                try:
                    self._stack_recovery_timer = self.create_timer(
                        max(0.001, float(self.retry_delay)),
                        lambda: self._run_stack_recovery(token, retry_map_switch),
                        clock=self.stack_recovery_clock)
                    return True
                except Exception as exc:
                    self._stack_recovery_token = None
                    failure = f'cannot schedule navigation stack recovery: {exc}'

        # Exhaustion is a failed task, not a successful/idle route completion.
        fault = String()
        fault.data = failure
        self.process_fault_callback(fault)
        return False

    def _run_stack_recovery(self, token, retry_map_switch):
        """Run once per timer; never restart by recursively calling navigation."""
        with self.state_lock:
            if token != self._stack_recovery_token:
                return
            timer = self._stack_recovery_timer
            if timer is None:
                return
            self._stack_recovery_timer = None
            timer.cancel()
            self.destroy_timer(timer)
            if not self._stack_recovery_is_current(token):
                self._stack_recovery_token = None
                return
            waypoint = self.waypoint_list[self.current_waypoint_index]
            map_name = waypoint.map_name
            navigation_context = (
                getattr(self, '_navigation_generation', 0), self.current_waypoint_index)

        self.get_logger().warning(
            f'系统故障，重启导航栈 {token[2]}/{self.max_retries}')
        transaction_lock = self._get_stack_transaction_lock()
        if not transaction_lock.acquire():
            self.get_logger().warning(
                '另一个控制器正在变更导航栈，延迟恢复且不执行清理')
            with self.state_lock:
                if token == self._stack_recovery_token:
                    self._stack_recovery_token = None
                    self._stack_recovery_attempts = max(
                        0, self._stack_recovery_attempts - 1)
            self._schedule_stack_recovery()
            return
        try:
            try:
                with self.state_lock:
                    self._navigation_cleanup_active = True
                cleanup_ok = self.shutdown_all_processes_service()
                if cleanup_ok is False:
                    self._latch_cleanup_failure('恢复前的导航堆栈清理未确认干净')
                    return
                with self.state_lock:
                    self._navigation_cleanup_active = False
                    if not self._stack_recovery_is_current(token):
                        return
                restarted = self.launch_new_stack(map_name)
            except Exception as exc:
                self.get_logger().error(f'重启导航栈异常: {exc}')
                restarted = False
        finally:
            transaction_lock.release()

        with self.state_lock:
            if not self._stack_recovery_is_current(token):
                return
            self._stack_recovery_token = None
            if restarted and retry_map_switch:
                self.is_map_switching = False

        if not restarted:
            self._schedule_stack_recovery()
            return
        # A successful restart does not replenish the waypoint's retry budget.
        if retry_map_switch:
            self.on_goal_reached(navigation_context=navigation_context)
        else:
            self.send_nav2_goal(waypoint, *navigation_context)

    def abort_navigation(self, reason, navigation_context=None)->bool:
        """
        function: 中止导航序列, 根据不同原因进行不同的处理
        param: @reason: 0: 主动取消（不需要处理）
                        1: 导航路径失败（请求备用路线）
                        2: 系统故障（安排重启，返回值表示是否已安排）
        ---------
        test: 测试通过
        """
        self.get_logger().warn('导航序列已中止')
        with self.state_lock:
            if (navigation_context is not None
                    and not self._navigation_context_is_current(*navigation_context)):
                return False
            current_waypoint_index = self.current_waypoint_index
            waypoint_list = self.waypoint_list
            if reason == 2:
                return self._schedule_stack_recovery()
 
        # 调用服务重新下发导航任务
        if reason == 0:
            print("[DEBG] reason = 0 测试通过///")
            # 主动取消，不处理
            self.get_logger().info('导航目标已被主动取消，忽略')
            return True

        elif reason == 1:
            print("[DEBG] reason = 1 测试通过///")
            # 导航失败 → 请求备用路线
            self.get_logger().warn('导航路径失败，正在请求备用路线...')
            with self.state_lock:
                if (navigation_context is not None
                        and not self._navigation_context_is_current(*navigation_context)):
                    return False
                self.publish_robot_state('running')

            # route_planner 需要最后一个已经到达的航点，才能封禁失败边并
            # 从该点重新规划。第一个航点就失败时不存在可靠的重规划起点。
            if not waypoint_list or current_waypoint_index <= 0:
                self.get_logger().error('尚无已完成航点，无法请求备用路线')
                if navigation_context is None:
                    self.complete_navigation()
                else:
                    self.complete_navigation(expected_generation=navigation_context[0])
                return False

            req = PubNewPath.Request()
            req.points = list(waypoint_list[:current_waypoint_index])
            # RoutePlanner publishes the replacement while this task remains
            # active. Accept that route without allowing unrelated tasks to
            # overwrite the one currently running.
            with self.state_lock:
                if (navigation_context is not None
                        and not self._navigation_context_is_current(*navigation_context)):
                    return False
                self.pending_replan_endpoints = (
                    req.points[-1].name, waypoint_list[-1].name)
                future = self.pub_new_path_client.call_async(req)
            response = self._wait_for_future(future, timeout_sec=10.0)
            if response is None or not response.success:
                with self.state_lock:
                    if (navigation_context is not None
                            and not self._navigation_context_is_current(*navigation_context)):
                        return False
                    self.pending_replan_endpoints = None
                self.get_logger().error('发布新路径失败')
                if navigation_context is None:
                    self.complete_navigation()
                else:
                    self.complete_navigation(expected_generation=navigation_context[0])
                return False
            self.get_logger().info('备用路线已重新规划并下发')
            return True

    @staticmethod
    def _classify_process_status(process_status):
        """Classify a complete query without treating UNKNOWN as stopped."""
        if not isinstance(process_status, dict) or not process_status:
            return 'unknown'
        values = [bool(value) for value in process_status.values()]
        if all(values):
            return 'running'
        if not any(values):
            return 'stopped'
        return 'partial'

    def get_process_status(self) -> Optional[dict]:
        try:
            req = GetProcessStatus.Request()
            future = self.get_process_status_client.call_async(req)
            res = self._wait_for_future(future, timeout_sec=5.0)
            if res is None:
                self._last_status_has_identity = False
                self._last_status_from_service = True
                self._last_reported_stack_state = ''
                self.get_logger().error('查询导航子栈状态失败，状态为UNKNOWN')
                return None
            self._last_reported_stack_map = getattr(res, 'active_map', '')
            self._last_reported_stack_session = getattr(
                res, 'localization_session_id', '')
            self._last_reported_stack_state = getattr(
                res, 'stack_state', '')
            self._last_status_has_identity = bool(
                self._last_reported_stack_state == 'RUNNING'
                and self._last_reported_stack_map
                and self._last_reported_stack_session)
            self._last_status_from_service = True
            return {"re_localization": res.re_localization_running,
                    "liosam": res.liosam_running,
                    "nav2_init_pose": res.nav2_init_pose_running,
                    "navigation2": res.navigation2_running}
        except Exception as exc:
            self._last_status_has_identity = False
            self._last_status_from_service = True
            self._last_reported_stack_state = ''
            self.get_logger().error(
                f'查询导航子栈状态异常，状态为UNKNOWN: {exc}')
            return None
        
    def shutdown_process_by_name(self, process_name: str) -> bool:
        """
        function: 通过服务名关闭进程
        param: @process_name: 进程名，如'navigation2', 'liosam', 'nav2_init_pose', 're_localization'
        return: 成功返回True, 失败返回False
        --------
        test:测试通过
        """
        try:
            req = ShutdownProcess.Request()
            req.process_name = process_name
            future = self.shutdown_process_client.call_async(req)
            response = self._wait_for_future(
                future, timeout_sec=self.PROCESS_SHUTDOWN_TIMEOUT_SEC)
            if response is not None and response.success:
                return True
            self.get_logger().error(f'关闭进程{process_name}未获成功响应')
            return False
        except Exception as e:
            self.get_logger().error(f'关闭进程{process_name}异常: {e}')
            return False

    def _wait_for_all_processes_stopped(self) -> bool:
        """Require consecutive all-stopped observations from ProcessManager."""
        deadline = time.monotonic() + self.CLEAN_STATUS_TIMEOUT_SEC
        confirmations = 0
        while True:
            status = self.get_process_status()
            if self._classify_process_status(status) == 'stopped':
                confirmations += 1
                if confirmations >= self.CLEAN_STATUS_CONFIRMATIONS:
                    return True
            else:
                confirmations = 0

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(self.CLEAN_STATUS_POLL_SEC, remaining))

        self.get_logger().error(
            '导航子栈关闭后未连续确认全部停止')
        return False

    def shutdown_all_processes_service(self):
        """Own a full cleanup, deferring while another transaction is active."""
        transaction_lock = self._get_stack_transaction_lock()
        if not transaction_lock.acquire(
                timeout_sec=self.STACK_TRANSACTION_WAIT_SEC):
            self.get_logger().warning(
                '其他导航栈事务仍在运行，保持清理门禁并延迟重试')
            return self.CLEANUP_DEFERRED
        try:
            return self._shutdown_all_processes_owned()
        finally:
            transaction_lock.release()

    def _shutdown_all_processes_owned(self) -> bool:
        """
        function: 通过服务关闭所有进程
        param: 
        return: 每个关闭请求成功且连续确认全停止时返回True
        --------
        test:测试通过
        """
        # Reverse startup order.  Do not return early: every process must get a
        # shutdown request even if an earlier response fails or times out.
        processes = list(reversed(self.STACK_PROCESS_NAMES))
        responses = {}
        for process_name in processes:
            responses[process_name] = self.shutdown_process_by_name(process_name)
        self._last_shutdown_results = responses

        responses_ok = all(responses.values())
        stopped_ok = self._wait_for_all_processes_stopped()
        if not responses_ok:
            failed = [name for name, success in responses.items() if not success]
            self.get_logger().error(
                f'导航子栈关闭服务失败: {", ".join(failed)}')
        return responses_ok and stopped_ok

    def publish_robot_state(self, state: str):
        """
        发布机器人状态

        参数:
            state: 机器人状态（'idle'或'running'）
        """
        msg = String()
        msg.data = state
        self.robot_state_pub.publish(msg)
        self.get_logger().debug(f'机器人状态: {state}')

def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()

    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
