#!/usr/bin/env python3
"""
导航管理器
开发者B - 导航模块

管理航点导航序列并与地图切换协调
"""
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from multi_map_navigation_msgs.msg import WaypointList, Waypoint, MapSwitchTrigger
from multi_map_navigation_msgs.srv import StartProcess, ShutdownProcess
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
        self.waypoint_list: Optional[List[Waypoint]] = None
        self.current_waypoint_index = 0
        self.current_map = None
        self.is_navigating = False
        self.is_map_switching = False
        self.task_id = None

        # ROS2订阅器用于航点列表
        self.waypoint_list_sub = self.create_subscription(
            WaypointList,
            '/waypoint_list',
            self.waypoint_list_callback,
            10
        )

        # ROS2订阅器用于地图切换完成
        self.map_switch_complete_sub = self.create_subscription(
            Bool,
            '/map_switch_complete',
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
            10
        )

        # Navigation2动作客户端
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 发布初始状态
        self.publish_robot_state('idle')

        # 进程管理服务客户端
        self.start_process_client = self.create_client(StartProcess, '/process_manager/start_process')
        self.shutdown_process_client = self.create_client(ShutdownProcess, '/process_manager/shutdown_process')

        # 等待服务可用
        self.get_logger().info('等待进程管理器服务...')
        self.start_process_client.wait_for_service(timeout_sec=10.0)
        self.shutdown_process_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('进程管理器服务已连接')

    def waypoint_list_callback(self, msg: WaypointList):
        """
        接收航点列表的回调

        参数:
            msg: 包含导航航点的WaypointList消息
        """
        if self.is_navigating:
            self.get_logger().warn('已在导航中，忽略新航点列表')
            return

        self.get_logger().info(
            f'收到航点列表: task_id={msg.task_id}, '
            f'total_waypoints={msg.total_waypoints}'
        )

        # 存储航点列表
        self.waypoint_list = msg.waypoints
        self.task_id = msg.task_id
        self.current_waypoint_index = 0
        self.current_map = msg.start_map_name

        # 开始导航序列
        self.start_navigation_sequence()

    def start_navigation_sequence(self):
        """从第一个航点开始导航序列"""
        if not self.waypoint_list or len(self.waypoint_list) == 0:
            self.get_logger().error('没有航点可导航')
            return

        self.get_logger().info(
            f'开始导航序列，共 {len(self.waypoint_list)} 个航点'
        )

        # 更新状态
        self.is_navigating = True
        self.publish_robot_state('running')

        # 导航到第一个航点
        self.navigate_to_next_waypoint()

    def navigate_to_next_waypoint(self):
        """导航到序列中的下一个航点"""
        if not self.waypoint_list or self.current_waypoint_index >= len(self.waypoint_list):
            self.get_logger().info('所有航点已完成')
            self.complete_navigation()
            return

        waypoint = self.waypoint_list[self.current_waypoint_index]

        self.get_logger().info(
            f'正在导航到航点 {self.current_waypoint_index + 1}/{len(self.waypoint_list)}: '
            f'type={waypoint.type}, map={waypoint.map_name}'
        )

        if self.current_waypoint_index == 0:
            # 在导航到第1个点之前，先启动导航堆栈
            self.get_logger().info('首次导航，启动完整导航堆栈...')
            self.launch_new_stack(waypoint.map_name)
            # 注意：不在这里发送导航目标，而是在全部启动完成后的回调中发送
            return

        # 非首次导航，直接发送导航目标
        self.send_nav2_goal(waypoint)

    def launch_new_stack(self, map_name: str) -> bool:
        """
        为指定地图启动新导航堆栈

        参数:
            map_name: 要启动的地图名称

        返回:
            启动成功返回True，否则返回False
        """
        # 启动顺序: re_localization -> nav2_init_pose -> liosam -> navigation2
        self.get_logger().info('开始启动导航堆栈...')

        # 使用异步回调，需要依次启动各个进程
        self.start_re_localization_async(map_name)
        return True  # 返回True，实际结果在回调中处理

    def shutdown_process_service(self, process_name: str) -> bool:
        """通过服务关闭进程"""
        try:
            req = ShutdownProcess.Request()
            req.process_name = process_name

            # 异步直接调用，不需要等待返回值
            future = self.shutdown_process_client.call_async(req)
            # rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            # if future.result() and future.result().success:
            #     return True
            # return False
            return True
        except Exception as e:
            self.get_logger().error(f'关闭进程{process_name}异常: {e}')
            return False

    def start_re_localization_async(self, map_name: str):
        """异步启动re_localization"""
        req = StartProcess.Request()
        req.process_name = 're_localization'
        req.map_name = map_name
        future = self.start_process_client.call_async(req)
        future.add_done_callback(lambda f: self.re_localization_callback(f, map_name))

    def re_localization_callback(self, future, map_name: str):
        """re_localization启动回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ re_localization启动成功: {response.message}')
                # 继续启动nav2_init_pose
                self.start_nav2_init_pose_async(map_name)
            else:
                self.get_logger().error(f'❌ re_localization启动失败: {response.message}')
                self.abort_navigation()
        except Exception as e:
            self.get_logger().error(f'❌ re_localization服务调用异常: {e}')
            self.abort_navigation()

    def start_nav2_init_pose_async(self, map_name: str):
        """异步启动nav2_init_pose"""
        req = StartProcess.Request()
        req.process_name = 'nav2_init_pose'
        req.map_name = map_name
        future = self.start_process_client.call_async(req)
        future.add_done_callback(lambda f: self.nav2_init_pose_callback(f, map_name))

    def nav2_init_pose_callback(self, future, map_name: str):
        """nav2_init_pose启动回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ nav2_init_pose启动成功')
                # 等待TF变换
                self.get_logger().info('⏳ 等待重定位完成并发布 map -> odom TF...')
                self.wait_for_map_to_odom_link_tf_async(timeout_sec=300.0)
            else:
                self.get_logger().error(f'❌ nav2_init_pose启动失败: {response.message}')
                self.shutdown_process_service('re_localization')
                self.abort_navigation()
        except Exception as e:
            self.get_logger().error(f'❌ nav2_init_pose服务调用异常: {e}')
            self.shutdown_process_service('re_localization')
            self.abort_navigation()

    def wait_for_map_to_odom_link_tf_async(self, timeout_sec: float = 300.0):
        """异步等待 map -> odom TF 变换"""
        import threading

        def check_tf():
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer, self)

            start_time = self.get_clock().now()
            deadline = start_time + rclpy.duration.Duration(seconds=timeout_sec)

            while self.get_clock().now() < deadline:
                try:
                    trans = tf_buffer.lookup_transform(
                        target_frame="map",
                        source_frame="odom",
                        time=rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    self.get_logger().info(
                        f"✅ map -> odom TF 已就绪！ "
                        f"translation: ({trans.transform.translation.x:.2f}, {trans.transform.translation.y:.2f})"
                    )
                    # TF就绪，继续启动liosam
                    self.start_liosam_async()
                    return
                except (LookupException, ConnectivityException, ExtrapolationException):
                    time.sleep(0.5)
                except Exception as ex:
                    self.get_logger().warn(f"TF 查询异常: {ex}")
                    time.sleep(0.5)

            self.get_logger().error(f"❌ 等待 map -> odom TF 超时 ({timeout_sec}s)")
            self.shutdown_process_service('nav2_init_pose')
            self.shutdown_process_service('re_localization')
            self.abort_navigation()

        thread = threading.Thread(target=check_tf, daemon=True)
        thread.start()

    def start_liosam_async(self):
        """异步启动liosam"""
        req = StartProcess.Request()
        req.process_name = 'liosam'
        req.map_name = ''
        future = self.start_process_client.call_async(req)
        future.add_done_callback(self.liosam_callback)

    def liosam_callback(self, future):
        """liosam启动回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ LIO-SAM启动成功')
                # 获取当前地图并继续启动navigation2
                if self.current_waypoint_index < len(self.waypoint_list):
                    waypoint = self.waypoint_list[self.current_waypoint_index]
                    self.start_navigation2_async(waypoint.map_name)
            else:
                self.get_logger().error(f'❌ LIO-SAM启动失败: {response.message}')
                self.shutdown_process_service('nav2_init_pose')
                self.shutdown_process_service('re_localization')
                self.abort_navigation()
        except Exception as e:
            self.get_logger().error(f'❌ LIO-SAM服务调用异常: {e}')
            self.shutdown_process_service('nav2_init_pose')
            self.shutdown_process_service('re_localization')
            self.abort_navigation()

    def start_navigation2_async(self, map_name: str):
        """异步启动navigation2"""
        req = StartProcess.Request()
        req.process_name = 'navigation2'
        req.map_name = map_name
        future = self.start_process_client.call_async(req)
        future.add_done_callback(lambda f: self.navigation2_callback(f, map_name))

    def navigation2_callback(self, future, map_name: str):
        """navigation2启动回调"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Navigation2启动成功')
                self.get_logger().info('🎉 导航堆栈启动完成，开始导航到目标点')
                # 导航堆栈启动完成，发送导航目标
                if self.waypoint_list and self.current_waypoint_index < len(self.waypoint_list):
                    waypoint = self.waypoint_list[self.current_waypoint_index]
                    self.send_nav2_goal(waypoint)
            else:
                self.get_logger().error(f'❌ Navigation2启动失败: {response.message}')
                self.shutdown_process_service('liosam')
                self.shutdown_process_service('nav2_init_pose')
                self.shutdown_process_service('re_localization')
                self.abort_navigation()
        except Exception as e:
            self.get_logger().error(f'❌ Navigation2服务调用异常: {e}')
            self.shutdown_process_service('liosam')
            self.shutdown_process_service('nav2_init_pose')
            self.shutdown_process_service('re_localization')
            self.abort_navigation()

    def wait_for_map_to_odom_link_tf(
        self,
        timeout_sec: float = 300.0, # 5分钟
        check_interval: float = 0.5
    ) -> bool:
        """
        等待 map -> odom TF 变换可用 (已弃用，使用wait_for_map_to_odom_link_tf_async)
        """
        self.get_logger().warning("wait_for_map_to_odom_link_tf已弃用，使用异步版本")
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
        触发地图切换操作

        参数:
            waypoint: 地图切换航点
        """
        self.is_map_switching = True

        # 创建地图切换触发消息
        trigger_msg = MapSwitchTrigger()
        trigger_msg.header.stamp = self.get_clock().now().to_msg()
        trigger_msg.current_map = self.current_map
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
        if self.current_waypoint_index + 1 < len(self.waypoint_list):
            trigger_msg.next_waypoint_id = self.waypoint_list[self.current_waypoint_index + 1].id
        else:
            trigger_msg.next_waypoint_id = -1

        # 发布触发
        self.map_switch_trigger_pub.publish(trigger_msg)

        self.get_logger().info(
            f'已触发地图切换: {trigger_msg.current_map} -> {trigger_msg.next_map}'
        )

    def map_switch_complete_callback(self, msg: Bool):
        """
        地图切换完成的回调

        参数:
            msg: 指示成功/失败的Bool消息
        """
        if not self.is_map_switching:
            return

        self.is_map_switching = False

        if msg.data:
            self.get_logger().info('地图切换成功完成')

            # 更新当前地图
            if self.current_waypoint_index < len(self.waypoint_list):
                waypoint = self.waypoint_list[self.current_waypoint_index]
                self.current_map = waypoint.next_map_name

            # 移动到下一个航点
            self.current_waypoint_index += 1
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().error('地图切换失败，中止导航')
            self.abort_navigation()

    def send_nav2_goal(self, waypoint: Waypoint):
        """
        向Navigation2发送导航目标

        参数:
            waypoint: 目标航点
        """

        self.get_logger().info('⏳ 等待 Navigation2 完全激活...')
        time.sleep(5.0)  # 给足够时间让 Nav2 完成配置
        # 等待动作服务器
        if not self.nav2_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Navigation2动作服务器不可用')
            self.abort_navigation()
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

        # 发送目标
        send_goal_future = self.nav2_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav2_feedback_callback
        )

        send_goal_future.add_done_callback(self.nav2_goal_response_callback)

    def nav2_goal_response_callback(self, future):
        """Navigation2目标响应回调"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            self.abort_navigation()
            return

        self.get_logger().info('导航目标已接受')

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
        """Navigation2结果回调"""
        # Get the result wrapper
        result_wrapper = future.result()

        # Check the status
        # status=4 means SUCCEEDED in action_msgs.msg.GoalStatus
        if result_wrapper.status == 4:
            self.get_logger().info('导航目标成功完成 (SUCCEEDED)')
            self.on_goal_reached()
        else:
            self.get_logger().error(f'导航目标失败，状态码: {result_wrapper.status}')
            self.abort_navigation()

    def on_goal_reached(self):
        """处理成功到达目标"""
        # 添加防护性检查
        # if self.waypoint_list is None:
        #     self.get_logger().warn('航点列表为空，忽略目标到达事件')
        #     return
        # if self.current_waypoint_index >= len(self.waypoint_list):
        #     self.get_logger().warn(f'航点索引超出范围: {self.current_waypoint_index} >= {len(self.waypoint_list)}')
        #     return

        waypoint = self.waypoint_list[self.current_waypoint_index]

        self.get_logger().info(
            f'到达航点 {self.current_waypoint_index + 1}/{len(self.waypoint_list)}'
        )
        
        # 检查这是否为最终航点
        if self.current_waypoint_index == len(self.waypoint_list) - 1:
            self.get_logger().info('到达最终航点')
            self.complete_navigation()
        else:

            # 如果是地图切换点
            if self.is_map_switch_point(waypoint):
                self.get_logger().info('检测为地图切换点')
                self.trigger_map_switch(waypoint)

            # 如果为普通路点
            elif self.is_common_point(waypoint):            
                self.get_logger().info('检测为普通导航点')
                self.current_waypoint_index += 1
                self.navigate_to_next_waypoint()

            elif self.is_trafficlight_point(waypoint):
                self.get_logger().info('检测为红绿灯点')

                # TODO 停车，检测红绿灯，如果是绿灯则走，如果是红灯黄灯则停
                pass

            elif self.is_charge_point(waypoint):
                self.get_logger().info('检测为充电点')

                # TODO 对准充电桩停车
                pass
            
            else:

                # TODO
                pass

    def complete_navigation(self):
        """完成导航序列"""
        self.get_logger().info('导航序列成功完成')

        # 关闭所有进程
        self.shutdown_all_processes_service()

        # 更新状态
        self.is_navigating = False
        self.publish_robot_state('idle')

        # 重置导航状态
        self.waypoint_list = None
        self.current_waypoint_index = 0

    def shutdown_all_processes_service(self):
        """通过服务关闭所有进程"""
        processes = ['navigation2', 'liosam', 'nav2_init_pose', 're_localization']
        for process_name in processes:
            self.shutdown_process_service(process_name)

    def abort_navigation(self):
        """中止导航序列"""
        self.get_logger().error('导航序列已中止')

        # 更新状态
        self.is_navigating = False
        self.publish_robot_state('idle')

        # 重置导航状态
        self.waypoint_list = None
        self.current_waypoint_index = 0

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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
