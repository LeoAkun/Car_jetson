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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from multi_map_navigation_msgs.msg import WaypointList, Waypoint, MapSwitchTrigger
from multi_map_navigation_msgs.srv import GetProcessStatus, StartProcess, ShutdownProcess, PubNewPath
from typing import List, Optional
import time
import threading
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class NavigationManager(Node):
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

        self.waypoint_list: Optional[List[Waypoint]] = None
        self.current_waypoint_index = 0 # 表示要导航去的点
        self.current_map = None
        self.is_navigating = False
        self.is_map_switching = False
        self.task_id = None
        self.current_goal_handle = None
        self.total_path = None

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

        # Navigation2动作客户端
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

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

        # 判断是否还在运行中
        with self.state_lock:
            if self.is_navigating:
                self.get_logger().warn('已在导航中，忽略新航点列表')
                return
            # 存储航点列表
            self.waypoint_list = msg.waypoints
            self.task_id = msg.task_id
            self.current_waypoint_index = 0
            self.current_map = msg.start_map_name
            self.total_path = msg.total_path
            self.is_navigating = True
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
            waypoint_list = self.waypoint_list

        if not waypoint_list or len(waypoint_list) == 0:
            self.get_logger().error('没有航点可导航')
            self.complete_navigation()
            return

        self.get_logger().info(
            f'开始导航序列，共 {len(waypoint_list)} 个航点'
        )

        self.publish_robot_state('running')

        # 导航到第一个航点
        self.navigate_to_next_waypoint()

    def navigate_to_next_waypoint(self):
        """
        function: 导航到序列中的下一个航点
        param: 
        return:
        -----------
        test: 测试通过
        """
        with self.state_lock:
            waypoint_list = self.waypoint_list
            current_waypoint_index = self.current_waypoint_index

        if not waypoint_list or current_waypoint_index >= len(waypoint_list):
            self.get_logger().info('所有航点已完成')
            self.complete_navigation()
            return
            
        waypoint = waypoint_list[self.current_waypoint_index]

        self.get_logger().info(
            f'正在导航到航点 {current_waypoint_index + 1}/{len(waypoint_list)}: '
            f'type={waypoint.type}, map={waypoint.map_name}'
        )

        # 如果进程是全部启动，则直接发送目标
        process_status_dict = self.get_process_status()
        if all(process_status_dict.values()):
            self.send_nav2_goal(waypoint)
        # 如果进程还未启动，则启动进程
        else:
            self.get_logger().info('首次导航，启动完整导航堆栈...')
            if self.launch_new_stack(waypoint.map_name):
                # print(f"[TEST] self.send_nav2_goal(waypoint),waypoint: {waypoint}")                
                self.send_nav2_goal(waypoint)
            else:
                self.get_logger().error('导航堆栈启动失败，中止导航')
                # 重启失败
                if self.abort_navigation(reason = 2) == False:
                    self.get_logger().error('系统故障，等待远程驾驶连接...')
                    self.complete_navigation()
                    return
                # 重启成功，继续导航
                else:
                    self.navigate_to_next_waypoint()
                    return

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
        self._wait_for_future(future, timeout_sec=10.0)
        if not future.result() or not future.result().success:
            self.get_logger().error('启动re_localization失败')
            return False
        self.get_logger().info('启动re_localization成功')

        # 启动nav2_init_pose
        req = StartProcess.Request()
        req.process_name = 'nav2_init_pose'
        req.map_name = map_name
        future = self.start_process_client.call_async(req)
        self._wait_for_future(future, timeout_sec=10.0)
        if not future.result() or not future.result().success:
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
        self._wait_for_future(future, timeout_sec=10.0)
        if not future.result() or not future.result().success:
            self.get_logger().error('启动liosam失败')
            return False
        self.get_logger().info('启动liosam成功')

        # 启动navigation2
        req = StartProcess.Request()
        req.process_name = 'navigation2'
        req.map_name = map_name
        future = self.start_process_client.call_async(req)
        self._wait_for_future(future, timeout_sec=10.0)
        if not future.result() or not future.result().success:
            self.get_logger().error('启动navigation2失败')   
            return False

        self.get_logger().info('等待navigaion2全部加载完成')
        time.sleep(50.0)
        self.get_logger().info('navigaion2全部加载完成')
        return True
    
    def _wait_for_future(self, future, timeout_sec: float):
        """等待 future 完成，由 MultiThreadedExecutor 在其他线程处理回调"""
        start = time.time()
        while not future.done() and time.time() - start < timeout_sec:
            time.sleep(0.05)
        return future.result()

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

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self)

        start_time = self.get_clock().now()
        deadline = start_time + rclpy.duration.Duration(seconds=timeout_sec)

        while self.get_clock().now() < deadline:
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
            with self.state_lock:
                self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
            with self.state_lock:
                self.is_map_switching = False
        else:
            self.get_logger().error('地图切换失败，中止导航')
            # 重启失败，等待远程驾驶
            if self.abort_navigation(reason = 2) == False:
                self.get_logger().error('系统故障，等待远程驾驶连接...')
                self.complete_navigation()
            # 重启成功，重新切换地图
            else:
                with self.state_lock:
                    self.is_map_switching = False
                self.on_goal_reached()

    def send_nav2_goal(self, waypoint: Waypoint):
        """
        function: 向Navigation2发送导航目标
        param: @ waypoint: 目标航点
        ---------
        test: 测试通过
        """

        # 等待动作服务器
        self.get_logger().info('等待nav2服务器...')
        if not self.nav2_client.wait_for_server(timeout_sec=50.0):
            self.get_logger().error('Navigation2动作服务器不可用')
            # 重启失败
            if self.abort_navigation(reason = 2) == False:
                self.get_logger().error('系统故障，等待远程驾驶连接...')
                self.complete_navigation()
                return
            # 重启成功,重新发送目标点
            else:
                self.navigate_to_next_waypoint()
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

        self.get_logger().info(
            f'向Navigation2发送目标: '
            f'x={waypoint.x:.2f}, '
            f'y={waypoint.y:.2f}, '
            f'yaw={waypoint.yaw:.2f}'
        )
        
        self.publish_robot_state("running")

        # 发送目标
        send_goal_future = self.nav2_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav2_feedback_callback
        )
        # print("[TEST] send_nav2_goal")
        send_goal_future.add_done_callback(self.nav2_goal_response_callback)

    def nav2_goal_response_callback(self, future):
        """
        function: Navigation2目标响应回调
        -------------
        test: 测试通过
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            # 重启失败
            if self.abort_navigation(reason = 2) == False:
                self.get_logger().error('系统故障，等待远程驾驶连接...')
                self.complete_navigation()
                return
            # 重启成功,重新发送目标点
            else:
                self.navigate_to_next_waypoint()
                return

        self.get_logger().info('导航目标已接受')
        with self.state_lock:
            self.current_goal_handle = goal_handle

        # 等待结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)

    def nav2_feedback_callback(self, feedback_msg):
        """Navigation2反馈回调"""
        feedback = feedback_msg.feedback
        # 记录导航进度
        self.get_logger().debug(
            f'导航反馈: distance_remaining={feedback.distance_remaining:.2f}m'
        )

    def nav2_result_callback(self, future):
        """
        function: Navigation2结果回调
        param: 
        return:
        -----------
        test: 测试通过
        """
        # Get the result wrapper
        result_wrapper = future.result()
        self.publish_robot_state("running")

        # 使用ros2 interface show action_msgs/msg/GoalStatus查看
        # status=4 means SUCCEEDED in action_msgs.msg.GoalStatus
        if result_wrapper.status == 4:
            self.get_logger().info('导航目标成功完成 (SUCCEEDED)')
            # print("[TEST] 测试通过")
            self.on_goal_reached()
        elif result_wrapper.status == 5:
            # print("[TEST] 测试通过")
            self.get_logger().info('导航目标被切换 (CANCELED)')
            self.abort_navigation(reason = 0)
        else:
            # print("[TEST] 测试通过")
            self.get_logger().error(f'导航目标失败，状态码: {result_wrapper.status}')
            self.abort_navigation(reason = 1)

    def on_goal_reached(self):
        """
        function: 处理成功到达目标。
        param:
        return:
        -----------------
        test: 测试通过
        """
        with self.state_lock:
            waypoint_list = self.waypoint_list
            current_waypoint_index = self.current_waypoint_index
            current_map = self.current_map

        waypoint = waypoint_list[current_waypoint_index]
        self.get_logger().info(
            f'到达航点 {current_waypoint_index + 1}/{len(waypoint_list)}'
        )
        
        # 检查这是否为最终航点
        if current_waypoint_index == len(waypoint_list) - 1:
            self.get_logger().info('到达最终航点')
            self.complete_navigation()
        else:

            # 如果是地图切换点
            if self.is_map_switch_point(waypoint):
                self.get_logger().info('检测为地图切换点')
                
                # 如果当前地图和地图切换点下一个点的地图不一样，则需要切换地图
                if current_map != waypoint_list[current_waypoint_index + 1].map_name:
                    # 在回调中修改 self.current_waypoint_index
                    # print("[TEST] 需要切换地图")
                    self.publish_robot_state("running")
                    self.trigger_map_switch(waypoint)
                # 如果当前地图和地图切换点下一个点的地图一样，则不需要切换地图
                else:
                    # print("[TEST] 不需要切换地图")
                    self.publish_robot_state("running")
                    with self.state_lock:
                        self.current_waypoint_index += 1
                    self.navigate_to_next_waypoint()

            # 如果为普通路点
            elif self.is_common_point(waypoint):            
                self.get_logger().info('检测为普通导航点')
                with self.state_lock:
                    self.current_waypoint_index += 1
                self.navigate_to_next_waypoint()

            elif self.is_trafficlight_point(waypoint):
                self.get_logger().info('检测为红绿灯点')
                with self.state_lock:
                    self.current_waypoint_index += 1
                
                # TODO 停车，检测红绿灯，如果是绿灯则走，如果是红灯黄灯则停
                pass

            elif self.is_charge_point(waypoint):
                self.get_logger().info('检测为充电点')
                with self.state_lock:
                    self.current_waypoint_index += 1

                # TODO 对准充电桩停车
                pass
            
            else:

                # TODO
                pass
    
    def complete_navigation(self):
        """
        function: 完成导航序列,关闭所有进程
        param:
        return:
        ----------
        test: 测试通过
        """
        self.get_logger().info('导航结束，关闭所有进程')

        # 关闭所有进程
        self.shutdown_all_processes_service()

        # 更新状态
        with self.state_lock:
            self.is_navigating = False
            self.is_map_switching = False
            self.waypoint_list = None
            self.current_waypoint_index = 0
            self.current_goal_handle = None
        self.publish_robot_state("idle")
        
    def abort_navigation(self, reason)->bool:
        """
        function: 中止导航序列, 根据不同原因进行不同的处理
        param: @reason: 0: 主动取消（不需要处理）
                        1: 导航路径失败（请求备用路线）
                        2: 系统故障（重启进程进程）
        ---------
        test: 测试通过
        """
        self.get_logger().warn('导航序列已中止')
        with self.state_lock:
            current_waypoint_index = self.current_waypoint_index
            waypoint_list = self.waypoint_list
 
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
                self.is_navigating = False
            self.publish_robot_state('running')
            req = PubNewPath.Request()
            points = []
            
            for idx in range(current_waypoint_index):
                points.append(waypoint_list[idx])
            # print(f"[TEST] points: {points}")
            req.points =  points
            future = self.pub_new_path_client.call_async(req)
            self._wait_for_future(future, timeout_sec=10.0)
            if not future.result() or not future.result().success:
                self.get_logger().error('发布新路径失败')
                self.complete_navigation()
                return False
            # print("[TEST] 发起请求成功")
            return True

        elif reason == 2:
            # 系统故障 → 尝试重启
            # print("[TEST] reason = 2 测试通过")
            self.get_logger().warn('系统故障，尝试重启')
            attempt_count = 0
            self.shutdown_all_processes_service()
            while self.launch_new_stack(waypoint_list[current_waypoint_index].map_name) == False:
                self.shutdown_all_processes_service()
                attempt_count  += 1     
                self.get_logger().warn(f'系统故障，尝试重启, 尝试次数{attempt_count}/{self.max_retries}')  
                if attempt_count >= self.max_retries:
                    self.get_logger().error(f'系统故障, 达到最大重启尝试次数')
                    self.complete_navigation()
                    return False
            self.publish_robot_state("running")
            return  True
        
    def get_process_status(self) -> dict:
        req = GetProcessStatus.Request()
        future = self.get_process_status_client.call_async(req)
        self._wait_for_future(future, timeout_sec=5.0)
        res =  future.result()
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
            self._wait_for_future(future, timeout_sec=5.0)
            if future.result() and future.result().success:
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
