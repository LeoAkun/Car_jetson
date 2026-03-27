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
        self.fitness_score_threshold = 1.0  # 匹配分数阈值 (越小越好)
        
        # 搜索范围设置
        self.search_radius = 3.0           # 搜索半径 (米)
        self.search_step_dist = 1.0         # 搜索步长 (米) -> 生成网格点
        
        # 角度搜索设置
        self.angle_step = 30                # 角度步长 (度)
        self.trials_per_pose = 1            # 每个位置/角度尝试次数 (为节省时间，建议设为1)

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

    def generate_search_grid(self, center_x, center_y):
        """
        生成以 center_x, center_y 为中心的网格搜索点
        """
        # 生成偏移量 [-10, -6, -2, 2, 6, 10]
        offsets = np.arange(-self.search_radius, self.search_radius + 0.1, self.search_step_dist)
        
        # 始终包含 (0,0) 中心点
        if 0.0 not in offsets:
            offsets = np.append(offsets, 0.0)
        
        points = []
        for dx in offsets:
            for dy in offsets:
                # 可以选择只搜索圆形区域内的点，减少计算量
                if math.sqrt(dx**2 + dy**2) <= self.search_radius * 1.2: 
                    points.append((center_x + dx, center_y + dy))
        
        # 将中心点放到列表第一个，优先搜索
        points.sort(key=lambda p: (p[0]-center_x)**2 + (p[1]-center_y)**2)
        
        self.get_logger().info(f"生成搜索网格: 半径 {self.search_radius}m, 步长 {self.search_step_dist}m, 共 {len(points)} 个位置点")
        return points

    def execute_logic(self):
        """主执行逻辑"""
        
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
        center_x, center_y = self.converter.gps_to_map(lat, lon)
        self.get_logger().info(f"✅ GPS中心点: ({center_x:.2f}, {center_y:.2f})")

        # 3. 准备搜索变量
        best_score = float('inf')
        best_pose_msg = None
        best_info = ""

        # 生成网格点
        search_points = self.generate_search_grid(center_x, center_y)
        total_steps = len(search_points) * (360 // self.angle_step)
        current_step = 0

        self.get_logger().info(f"🚀 开始全域搜索 (预计尝试 {total_steps} 次调用)...")

        # 4. 双层循环：位置 -> 角度
        request = ReLocalization.Request()
        
        for pt_idx, (px, py) in enumerate(search_points):
            self.get_logger().info(f"--- 搜索点 [{pt_idx+1}/{len(search_points)}]: ({px:.2f}, {py:.2f}) ---")
            
            for yaw in range(0, 360, self.angle_step):
                current_step += 1
                q = self.euler_to_quaternion(0, 0, yaw)

                # 填充请求
                request.initial_pose.header.frame_id = "map"
                request.initial_pose.header.stamp = self.get_clock().now().to_msg()
                request.initial_pose.pose.pose.position.x = float(px)
                request.initial_pose.pose.pose.position.y = float(py)
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
                        self.get_logger().info(f"位置({px:.1f},{py:.1f}) 角度{yaw:3d}° → score = {score:.4f}")
                        # 如果找到更好的结果
                        if score < best_score:
                            best_score = score
                            best_pose_msg = resp.pose.pose.pose
                            best_info = f"位置({px:.1f}, {py:.1f}), 角度 {yaw}°"
                            self.get_logger().info(f"✨ 发现更佳点: {best_info} | Score: {score:.4f}")
                            
                            # 【优化】如果分数极好 (例如 < 0.5)，可以直接提前退出
                            if score < self.fitness_score_threshold:
                                self.get_logger().info("🔥 分数极佳，提前结束搜索！")
                                self.current_transform = best_pose_msg
                                self.timer = self.create_timer(0.1, self.publish_transform)
                                # self.broadcast_tf(best_pose_msg)
                                return True
                except Exception as e:
                    pass

        # 5. 结算
        if best_pose_msg and best_score < 10.0:
            self.get_logger().info(f"🏆 最终最佳匹配: {best_info} | Score: {best_score:.4f}")
            self.current_transform = best_pose_msg
            self.timer = self.create_timer(0.1, self.publish_transform)
            # self.broadcast_tf(best_pose_msg)
            return True
        else:
            self.get_logger().error(f"❌ 搜索失败。最佳分数 {best_score:.4f} 仍高于阈值 {10.0}")
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