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
from std_msgs.msg import String, Bool
from multi_map_navigation_msgs.msg import WaypointList, Waypoint, MapSwitchTrigger
from multi_map_navigation_msgs.srv import GetProcessStatus, StartProcess, ShutdownProcess, PubNewPath
from typing import List, Optional
import time
import threading
import tf2_ros
from rclpy.clock import Clock, ClockType
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from multi_map_navigation.charging_navigation import ChargingNavigation

class NavigationManager(ChargingNavigation, Node):
    def __init__(self):
        super().__init__('navigation_manager')

        # 声明参数
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('navigation_timeout', 300.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('retry_delay', 5.0)

        # 获取参数
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        self.max_retries = self.get_parameter('max_retries').value
        self.retry_delay = self.get_parameter('retry_delay').value

        # 导航状态
        self.state_lock = threading.RLock() # 可重入锁，防止同线程嵌套调用死锁

        self.waypoint_list: Optional[List[Waypoint]] = None # 当前任务的航点列表
        self.current_waypoint_index = 0 # 当前要去的航点下标
        self.current_map = None
        self.is_navigating = False
        self.is_map_switching = False
        self.task_id = None
        self.current_goal_handle = None # 当前发给 Nav2 的目标句柄
        self._navigation_generation = 0
        self._goal_sequence = 0
        self._active_goal_token = None
        self.total_path = None
        self.stack_fault_active = False
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
            Bool,
            '/map_switch_complete',
            # '/test/map_switch_complete',
            self.map_switch_complete_callback,
            10
        )

        self.process_fault_sub = self.create_subscription(
            String,
            '/process_manager/fault',
            self.process_fault_callback,
            10,
            callback_group=self.service_callback_group
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

        # 如果进程是全部启动，则直接发送目标
        process_status_dict = self.get_process_status()
        if not self._navigation_context_is_current(
                navigation_generation, current_waypoint_index, waypoint):
            return
        if all(process_status_dict.values()):
            # 等待局部代价地图重建，避免到达航点后立即发送新目标导致障碍物消失
            time.sleep(6.0)
            if self._navigation_context_is_current(
                    navigation_generation, current_waypoint_index, waypoint):
                self.send_nav2_goal(
                    waypoint, navigation_generation, current_waypoint_index)
        # 如果进程还未启动，则启动进程
        else:
            self.get_logger().info('首次导航，启动完整导航堆栈...')
            if self.launch_new_stack(waypoint.map_name):
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

    def launch_new_stack(self, map_name: str) -> bool:
        """
        function: 为指定地图启动新导航堆栈
        param: @map_name: 要启动的地图名称
        return: 启动成功返回True，否则返回False
        -----------
        test: 测试通过
        """
        # 启动顺序: re_localization -> nav2_init_pose -> liosam -> navigation2

        # 启动re_localization
        req = StartProcess.Request()
        req.process_name = 're_localization'
        req.map_name = map_name
        future = self.start_process_client.call_async(req)
        response = self._wait_for_future(future, timeout_sec=10.0)
        if not response or not response.success:
            self.get_logger().error('启动re_localization失败')
            return False
        self.get_logger().info('启动re_localization成功')

        # 启动nav2_init_pose
        req = StartProcess.Request()
        req.process_name = 'nav2_init_pose'
        req.map_name = map_name
        future = self.start_process_client.call_async(req)
        response = self._wait_for_future(future, timeout_sec=10.0)
        if not response or not response.success:
            self.get_logger().error('启动nav2_init_pose失败')
            return False
        self.get_logger().info('启动nav2_init_pose成功')

        # 等待tf变换完成
        self.get_logger().info('等待重定位完成并发布 map -> odom TF...')
        if not self.wait_for_map_to_odom_link_tf(timeout_sec=600.0):  # 可根据实际调整
            self.get_logger().error('map -> odom TF 长时间未出现，启动失败')
            
            return False
        self.get_logger().info('检测到已发布map -> odom TF...')

        # 启动liosam
        req = StartProcess.Request()
        req.process_name = 'liosam'
        req.map_name = ''
        future = self.start_process_client.call_async(req)
        response = self._wait_for_future(future, timeout_sec=75.0)
        if not response or not response.success:
            self.get_logger().error('启动liosam失败')
            return False
        self.get_logger().info('启动liosam成功')

        # 启动navigation2
        req = StartProcess.Request()
        req.process_name = 'navigation2'
        req.map_name = map_name
        future = self.start_process_client.call_async(req)
        response = self._wait_for_future(future, timeout_sec=75.0)
        if not response or not response.success:
            self.get_logger().error('启动navigation2失败')   
            return False

        self.get_logger().info('Navigation2关键节点和生命周期状态均已就绪')
        return True
    
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

    def wait_for_map_to_odom_link_tf(self, timeout_sec: float = 300.0, check_interval: float = 0.5) -> bool:
        """
        function: 等待 map -> odom TF 变换可用
        param: @ timeout_sec: 等待时间
               @ check_interval: 检查间隔
        return: 成功返回True, 失败返回False
        ---------------
        test: 测试通过
        """
        self.get_logger().info("等待 map -> odom TF 变换就绪...")

        with self.state_lock:
            if self.stack_fault_active or not self.is_navigating:
                self.get_logger().warning('导航任务已故障或取消，停止等待 map -> odom TF')
                return False

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self)

        start_time = self.get_clock().now()
        deadline = start_time + rclpy.duration.Duration(seconds=timeout_sec)

        while self.get_clock().now() < deadline:
            with self.state_lock:
                if self.stack_fault_active or not self.is_navigating:
                    self.get_logger().warning('导航任务已故障或取消，停止等待 map -> odom TF')
                    return False

            # rclpy.spin_once(self, timeout_sec=0)
            try:
                # 尝试查询最新的变换（time=0 表示 latest）
                trans: TransformStamped = tf_buffer.lookup_transform(
                    target_frame="map",
                    source_frame="odom",
                    time=rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)  # 小超时避免阻塞
                )
                self.get_logger().info(
                    f"map -> odom TF 已就绪！ "
                    f"translation: {trans.transform.translation.x:.2f}, {trans.transform.translation.y:.2f}"
                )
                return True

            except (LookupException, ConnectivityException, ExtrapolationException):
                # 正常等待中，不报错
                self.get_logger().debug("map -> odom TF 尚未可用，继续等待...")
            
            except Exception as ex:
                self.get_logger().warn(f"TF 查询异常: {ex}")

            # 避免 CPU 空转
            time.sleep(check_interval)
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
        with self.state_lock:
            self.is_map_switching = True
            current_map = self.current_map
            current_waypoint_index = self.current_waypoint_index
            waypoint_list = self.waypoint_list

        # 创建地图切换触发消息
        trigger_msg = MapSwitchTrigger()
        trigger_msg.header.stamp = self.get_clock().now().to_msg()
        trigger_msg.current_map = current_map
        trigger_msg.next_map = waypoint.next_map_name

        # 创建切换位姿（使用当前点在下一张地图坐标系下的坐标）
        from geometry_msgs.msg import PoseStamped, Quaternion
        from tf_transformations import quaternion_from_euler

        switch_pose = PoseStamped()
        switch_pose.header.stamp = self.get_clock().now().to_msg()
        switch_pose.header.frame_id = "map"
        switch_pose.pose.position.x = waypoint.next_x
        switch_pose.pose.position.y = waypoint.next_y
        switch_pose.pose.position.z = 0.0

        # 将yaw角转换为四元数
        q = quaternion_from_euler(0, 0, waypoint.next_yaw)
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

    def map_switch_complete_callback(self, msg: Bool):
        """
        function: 地图切换完成的回调
        param: @ msg: 消息
        return:
        ----------------
        test: 测试通过
        """
        with self.state_lock:
            is_map_switching = self.is_map_switching

        if not is_map_switching:
            return

        with self.state_lock:
            current_waypoint_index = self.current_waypoint_index
            waypoint_list = self.waypoint_list

        self.publish_robot_state("running")

        if msg.data:
            self.get_logger().info('地图切换成功完成')

            # 更新当前地图
            if current_waypoint_index < len(waypoint_list):
                waypoint = waypoint_list[current_waypoint_index]
                with self.state_lock:
                    self.current_map = waypoint.next_map_name

            # 移动到下一个航点
            self._advance_current_waypoint()
            self.navigate_to_next_waypoint()
            with self.state_lock:
                self.is_map_switching = False
        else:
            self.get_logger().error('地图切换失败，安排恢复')
            self.abort_navigation(reason=2)

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
            if not self._navigation_context_is_current(
                    navigation_generation, waypoint_index, waypoint):
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
        from tf_transformations import quaternion_from_euler

        goal_msg = NavigateToPose.Goal()

        # 构建位姿
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = 0.0

        # 将yaw角转换为四元数
        q = quaternion_from_euler(0, 0, waypoint.yaw)
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
            if not self._navigation_context_is_current(
                    navigation_generation, waypoint_index, waypoint):
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
            self._navigation_generation = getattr(self, '_navigation_generation', 0) + 1
            self._active_goal_token = None
            self._navigation_cleanup_active = True
            self.is_navigating = False
            self.is_map_switching = False
            self.waypoint_list = None
            self.current_waypoint_index = 0
            self.current_goal_handle = None
            self.stack_fault_active = False
            self.pending_replan_endpoints = None
            self._reset_stack_recovery()

        # Invalidate the task before service calls, which may wait for a response.
        self.shutdown_all_processes_service()
        with self.state_lock:
            self._navigation_cleanup_active = False
            self.publish_robot_state("idle")

    def process_fault_callback(self, msg: String):
        """Fail the active task deterministically when a required stack node dies."""
        with self.state_lock:
            if not self.is_navigating or self.stack_fault_active:
                return
            self.stack_fault_active = True
            self._navigation_generation = getattr(self, '_navigation_generation', 0) + 1
            self._active_goal_token = None
            self._reset_stack_recovery()
            self.pending_replan_endpoints = None
            goal_handle = self.current_goal_handle

        self.get_logger().error(f'检测到导航栈故障: {msg.data}')
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
        self.shutdown_all_processes_service()
        with self.state_lock:
            self.is_navigating = False
            self.is_map_switching = False
            self.current_goal_handle = None
            self.waypoint_list = None
            self.current_waypoint_index = 0
            self.publish_robot_state('fault')
        self.get_logger().error(f'导航任务因栈故障终止，等待新任务: {reason}')
        
    def _reset_stack_recovery(self):
        """Invalidate queued recovery when a route starts, ends, or faults."""
        with self.state_lock:
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
        try:
            self.shutdown_all_processes_service()
            with self.state_lock:
                if not self._stack_recovery_is_current(token):
                    return
            restarted = self.launch_new_stack(map_name)
        except Exception as exc:
            self.get_logger().error(f'重启导航栈异常: {exc}')
            restarted = False

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

    def get_process_status(self) -> dict:
        req = GetProcessStatus.Request()
        future = self.get_process_status_client.call_async(req)
        res = self._wait_for_future(future, timeout_sec=5.0)
        if res is None:
            self.get_logger().error('查询导航子栈状态失败')
            return {
                "re_localization": False,
                "liosam": False,
                "nav2_init_pose": False,
                "navigation2": False,
            }
        return {"re_localization": res.re_localization_running,
                "liosam":res.liosam_running,
                "nav2_init_pose": res.nav2_init_pose_running,
                "navigation2": res.navigation2_running}
        
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
            response = self._wait_for_future(future, timeout_sec=5.0)
            if response is not None and response.success:
                return True
            return False
        except Exception as e:
            self.get_logger().error(f'关闭进程{process_name}异常: {e}')
            return False

    def shutdown_all_processes_service(self):
        """
        function: 通过服务关闭所有进程
        param: 
        return:
        --------
        test:测试通过
        """
        processes = ['navigation2', 'liosam', 'nav2_init_pose', 're_localization']
        for process_name in processes:
            self.shutdown_process_by_name(process_name)

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
