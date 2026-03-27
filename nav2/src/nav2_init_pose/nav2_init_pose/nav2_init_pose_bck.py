import rclpy
import math
import os
import sys
import time
import numpy as np
import itertools
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from re_localization.srv import ReLocalization
import transforms3d.euler as euler

# --- 2. 核心节点 ---
class PoseInitNode(Node):
    def __init__(self):
        super().__init__('pose_init_node')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_gps = None
        self.current_transform = None  # 存储当前的变换

        # 创建定时器，每100ms发布一次变换
        self.timer = self.create_timer(0.1, self.publish_transform)

        # --- 核心参数配置 ---
        self.fitness_score_threshold = 5.0  # 匹配分数阈值 (越小越好)
        
        # 角度搜索设置
        self.angle_step = 30                # 角度步长 (度)
        self.trials_per_pose = 3            # 每个位置/角度尝试次数 (为节省时间，建议设为1)

        # 客户端与订阅
        self.re_localization_client = self.create_client(ReLocalization, '/re_localization')

        print(f"[DEBG] 延迟...")
        time.sleep(5)

    def euler_to_quaternion(self, roll_deg, pitch_deg, yaw_deg):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        w, x, y, z = euler.euler2quat(roll, pitch, yaw, axes='sxyz')
        return [x, y, z, w]

    def execute_logic(self):
        """主执行逻辑"""
        
        # 1. 等待服务
        while not self.re_localization_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /re_localization 服务...')
            rclpy.spin_once(self, timeout_sec=0.1)

        # 3. 准备搜索变量
        best_score = float('inf')
        best_pose_msg = None
        best_info = ""

        # 4. 仅角度
        request = ReLocalization.Request()
        init_x, init_y = -148.56,493.92

        for yaw in range(0, 360, self.angle_step):
            
            q = self.euler_to_quaternion(0, 0, yaw)
            # 填充请求
            request.initial_pose.header.frame_id = "map"
            request.initial_pose.header.stamp = self.get_clock().now().to_msg()
            request.initial_pose.pose.pose.position.x = init_x
            request.initial_pose.pose.pose.position.y = init_y
            request.initial_pose.pose.pose.position.z = 0.0
            request.initial_pose.pose.pose.orientation.x = q[0]
            request.initial_pose.pose.pose.orientation.y = q[1]
            request.initial_pose.pose.pose.orientation.z = q[2]
            request.initial_pose.pose.pose.orientation.w = q[3]

            # 调用服务
            future = self.re_localization_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            try:
                resp = future.result()
                if resp.success:
                    score = resp.fitness_score
                    self.get_logger().info(f"位置({init_x:.1f},{init_y:.1f}) 角度{yaw:3d}° → score = {score:.4f}")
                    # 如果找到更好的结果
                    if score < best_score:
                        best_score = score
                        best_pose_msg = resp.pose.pose.pose
                        best_info = f"位置({px:.1f}, {py:.1f}), 角度 {yaw}°"
                        self.get_logger().info(f"✨ 发现更佳点: {best_info} | Score: {score:.4f}")
                        
                        # 【优化】如果分数极好 (例如 < 0.5)，可以直接提前退出
                        if score < 0.5:
                            self.get_logger().info("🔥 分数极佳，提前结束搜索！")
                            self.broadcast_tf(best_pose_msg)
                            return True
            except Exception as e:
                pass

    def broadcast_tf(self, pose):
        """保存变换信息，由定时器定期发布"""
        self.current_transform = pose
        self.get_logger().info("📡 变换已更新，将持续发布")

    def publish_transform(self):
        """定时器回调：持续发布 map->odom 变换"""
        if self.current_transform is not None:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = self.current_transform.position.x
            t.transform.translation.y = self.current_transform.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation = self.current_transform.orientation
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseInitNode()
    
    try:
        # 使用 execute_logic 接管流程
        for i in range(3):
            if node.execute_logic():
                # 成功后保持节点存活以维护 TF
                print("[System] 初始化成功，系统挂起以保持 TF 广播...")
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()