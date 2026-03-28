# 粗网格 + 启发式搜索
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
from sensor_msgs.msg import NavSatFix

class GpsToMapConverter:
    def __init__(self, csv_file_path):
        self.EARTH_RADIUS = 6378137.0
        
        if not os.path.exists(csv_file_path):
            raise FileNotFoundError(f"找不到文件: {csv_file_path}")

        print(f"[Info] 正在读取标定文件: {csv_file_path}")
        try:
            self.calibration_data = np.loadtxt(csv_file_path, delimiter=',')
        except Exception as e:
            raise ValueError(f"读取 CSV 失败: {e}")

        self.anchor_lat = self.calibration_data[0, 0]
        self.anchor_lon = self.calibration_data[0, 1]
        self.affine_matrix = None
        self._fit_transform_6param()

    def _latlon_to_enu(self, lat, lon):
        rad_lat = np.radians(lat)
        rad_lon = np.radians(lon)
        rad_anchor_lat = np.radians(self.anchor_lat)
        rad_anchor_lon = np.radians(self.anchor_lon)

        delta_lat = rad_lat - rad_anchor_lat
        delta_lon = rad_lon - rad_anchor_lon

        north = delta_lat * self.EARTH_RADIUS
        east  = delta_lon * self.EARTH_RADIUS * np.cos(rad_anchor_lat)

        if np.isscalar(lat):
            return np.array([east, north])
        else:
            return np.column_stack([east, north])

    def _fit_transform_6param(self):
        lats = self.calibration_data[:, 0]
        lons = self.calibration_data[:, 1]
        enu = self._latlon_to_enu(lats, lons)
        src = np.column_stack([enu, np.ones(len(enu))])
        dst = self.calibration_data[:, 2:4]
        M, residuals, rank, s = np.linalg.lstsq(src, dst, rcond=None)
        self.affine_matrix = np.vstack([M.T, [0.0, 0.0, 1.0]])

    def gps_to_map(self, lat, lon):
        enu = self._latlon_to_enu(lat, lon)
        pt = np.array([enu[0], enu[1], 1.0])
        map_pt = self.affine_matrix @ pt
        return map_pt[0], map_pt[1]

# --- 2. 核心节点 ---
class PoseInitNode(Node):
    def __init__(self):
        super().__init__('pose_init_node')

                
        self.declare_parameter('map_csv', '')
        self.map_csv = self.get_parameter('map_csv').value

        self.converter = GpsToMapConverter(self.map_csv)
        

        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_gps = None
        self.current_transform = None  # 存储当前的变换

        # 创建定时器，每100ms发布一次变换
        self.timer = None

        # --- 核心参数配置 ---
        self.fitness_score_threshold = 0.05 # 匹配分数阈值 (越小越好)

        # 粗粒度网格搜索设置（用于跳出局部最优）
        self.coarse_search_radius = 15.0    # 粗搜索半径 (米)
        self.coarse_grid_spacing = 5.0      # 粗搜索网格间距 (米)
        self.coarse_angle_step = 90         # 粗搜索角度步长 (度)

        # 启发式搜索设置
        self.initial_step = 2.0             # 初始搜索步长 (米)
        self.min_step = 0.3                 # 最小搜索步长 (米)
        self.step_reduction = 0.5           # 步长缩减因子
        self.max_iterations = 50            # 最大迭代次数

        # 角度搜索设置
        self.angle_step = 30                # 角度步长 (度)
        self.angle_fine_step = 15           # 精细角度步长 (度)

        # 客户端与订阅
        self.re_localization_client = self.create_client(ReLocalization, '/re_localization')
        self.gps_sub = self.create_subscription(
            NavSatFix, 
            '/sensing/gnss/pose_with_covariance', 
            self.gps_callback, 
            10
        )

        # 配置文件路径
        # CSV_PATH = '/home/akun/workspace/Car_jetson/nav2/src/nav2_init_pose/record_gps_map2.csv'


    def gps_callback(self, msg: NavSatFix):
        self.latest_gps = msg

    def euler_to_quaternion(self, roll_deg, pitch_deg, yaw_deg):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        w, x, y, z = euler.euler2quat(roll, pitch, yaw, axes='sxyz')
        return [x, y, z, w]

    def find_best_angle(self, x, y, angle_step):
        """在给定位置找到最佳角度"""
        best_score = float('inf')
        best_angle = 0
        best_pose = None

        request = ReLocalization.Request()
        for yaw in range(0, 360, angle_step):
            q = self.euler_to_quaternion(0, 0, yaw)
            request.initial_pose.header.frame_id = "map"
            request.initial_pose.header.stamp = self.get_clock().now().to_msg()
            request.initial_pose.pose.pose.position.x = float(x)
            request.initial_pose.pose.pose.position.y = float(y)
            request.initial_pose.pose.pose.position.z = 0.0
            request.initial_pose.pose.pose.orientation.x = q[0]
            request.initial_pose.pose.pose.orientation.y = q[1]
            request.initial_pose.pose.pose.orientation.z = q[2]
            request.initial_pose.pose.pose.orientation.w = q[3]

            future = self.re_localization_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            try:
                resp = future.result()
                if resp.success and resp.fitness_score < best_score:
                    best_score = resp.fitness_score
                    best_angle = yaw
                    best_pose = resp.pose.pose.pose
            except:
                pass

        return best_score, best_angle, best_pose

    def coarse_grid_search(self, center_x, center_y):
        """粗粒度网格搜索：在GPS中心点周围大范围采样，找全局最优起始点，避免局部最优"""
        radius = self.coarse_search_radius
        spacing = self.coarse_grid_spacing
        steps = np.arange(-radius, radius + spacing, spacing)
        total = len(steps) ** 2
        count = 0

        best_score = float('inf')
        best_x, best_y = center_x, center_y
        best_angle = 0
        best_pose = None

        self.get_logger().info(f"🔍 粗网格搜索: 半径={radius}m 间距={spacing}m 共{total}个点...")

        for dx in steps:
            for dy in steps:
                test_x = center_x + dx
                test_y = center_y + dy
                count += 1
                score, angle, pose = self.find_best_angle(test_x, test_y, self.coarse_angle_step)
                self.get_logger().info(f"  网格[{count}/{total}] ({test_x:.1f},{test_y:.1f}) score={score:.4f}")
                if score < best_score:
                    best_score = score
                    best_x, best_y = test_x, test_y
                    best_angle = angle
                    best_pose = pose
                    if best_score < self.fitness_score_threshold:
                        self.get_logger().info("🔥 粗搜索已达阈值，提前结束网格搜索")
                        return best_score, best_x, best_y, best_angle, best_pose

        self.get_logger().info(f"✅ 粗网格搜索完成: 最优点({best_x:.2f},{best_y:.2f}) score={best_score:.4f}")
        return best_score, best_x, best_y, best_angle, best_pose

    def execute_logic(self):
        """主执行逻辑 - 粗网格搜索 + 启发式爬山"""

        # 1. 等待服务
        while not self.re_localization_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /re_localization 服务...')
            rclpy.spin_once(self, timeout_sec=0.1)

        # 2. 等待 GPS
        self.get_logger().info('等待 GPS 数据...')
        while self.latest_gps is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        lat = self.latest_gps.latitude
        lon = self.latest_gps.longitude
        gps_x, gps_y = self.converter.gps_to_map(lat, lon)
        self.get_logger().info(f"✅ GPS起始点: ({gps_x:.2f}, {gps_y:.2f})")

        # 3. 粗粒度网格搜索（避免局部最优）
        best_score, current_x, current_y, best_angle, best_pose = self.coarse_grid_search(gps_x, gps_y)
        self.get_logger().info(f"🚀 网格搜索后最优起始点: ({current_x:.2f},{current_y:.2f}) fitness={best_score:.4f}")

        if best_score < self.fitness_score_threshold:
            self.get_logger().info("🔥 粗搜索已达阈值，跳过爬山搜索")
            self.current_transform = best_pose
            self.timer = self.create_timer(0.1, self.publish_transform)
            return True

        # 4. 精细爬山搜索（best-improvement：评估所有8方向后再移动最优）
        step_size = self.initial_step
        directions = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)]

        iteration = 0
        while iteration < self.max_iterations and step_size >= self.min_step:
            iteration += 1

            # 评估所有8个邻居，选最优（best-improvement，而非first-improvement）
            best_nb_score = best_score
            best_nb_x, best_nb_y = current_x, current_y
            best_nb_angle, best_nb_pose = best_angle, best_pose

            for dx, dy in directions:
                test_x = current_x + dx * step_size
                test_y = current_y + dy * step_size
                score, angle, pose = self.find_best_angle(test_x, test_y, self.angle_step)
                if score < best_nb_score:
                    best_nb_score = score
                    best_nb_x, best_nb_y = test_x, test_y
                    best_nb_angle, best_nb_pose = angle, pose

            if best_nb_score < best_score:
                self.get_logger().info(f"✨ 迭代{iteration}: ({best_nb_x:.2f},{best_nb_y:.2f}) fitness={best_nb_score:.4f} 角度={best_nb_angle}° (改进 {best_score-best_nb_score:.4f})")
                best_score = best_nb_score
                best_angle = best_nb_angle
                best_pose = best_nb_pose
                current_x, current_y = best_nb_x, best_nb_y

                if best_score < self.fitness_score_threshold:
                    self.get_logger().info("🔥 达到阈值，提前结束！")
                    self.current_transform = best_pose
                    self.timer = self.create_timer(0.1, self.publish_transform)
                    return True
            else:
                step_size *= self.step_reduction
                self.get_logger().info(f"🔄 未改进，缩小步长至 {step_size:.2f}m")

        # 5. 精细角度搜索
        if best_pose:
            self.get_logger().info(f"🎯 精细角度搜索...")
            final_score, final_angle, final_pose = self.find_best_angle(current_x, current_y, self.angle_fine_step)
            if final_score < best_score:
                best_score = final_score
                best_pose = final_pose
                best_angle = final_angle

        # 6. 结算
        if best_pose and best_score < 10.0:
            self.get_logger().info(f"🏆 最终结果: ({current_x:.2f},{current_y:.2f}) 角度={best_angle}° fitness={best_score:.4f}")
            self.current_transform = best_pose
            self.timer = self.create_timer(0.1, self.publish_transform)
            return True
        else:
            self.get_logger().error(f"❌ 搜索失败 fitness={best_score:.4f}")
            return False

    # def broadcast_tf(self, pose):
    #     """保存变换信息，由定时器定期发布"""
    #     self.current_transform = pose
    #     self.get_logger().info("📡 变换已更新，将持续发布")

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