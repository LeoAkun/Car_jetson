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

        # 粒子群优化（PSO）设置
        self.pso_particles = 30         # 粒子数量
        self.pso_iterations = 6        # PSO迭代轮数
        self.pso_search_radius = 10.0   # 搜索半径 (米，以GPS点为中心)
        self.pso_w = 0.7                # 惯性权重
        self.pso_c1 = 1.5               # 认知系数（向自身历史最优靠拢）
        self.pso_c2 = 1.5               # 社会系数（向全局最优靠拢）
        self.pso_angle_step = 30        # PSO阶段粗角度步长 (度)

        # 精细爬山设置（PSO收敛后使用）
        self.initial_step = 1.0         # 初始搜索步长 (米)
        self.min_step = 0.2             # 最小搜索步长 (米)
        self.step_reduction = 0.5       # 步长缩减因子
        self.max_iterations = 30        # 爬山最大迭代次数
        self.angle_step = 15            # 爬山角度步长 (度)
        self.angle_fine_step = 15       # 精细角度步长 (度)

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

    def pso_search(self, center_x, center_y):
        """粒子群优化搜索全局最优位置，避免局部最优"""
        n = self.pso_particles
        r = self.pso_search_radius
        w = self.pso_w
        c1 = self.pso_c1
        c2 = self.pso_c2

        # 初始化粒子位置（在搜索范围内均匀随机分布）
        np.random.seed(42)
        pos = np.random.uniform(-r, r, (n, 2)) + np.array([center_x, center_y])
        vel = np.random.uniform(-1.0, 1.0, (n, 2))

        pbest_pos = pos.copy()
        pbest_score = np.full(n, float('inf'))
        gbest_pos = pos[0].copy()
        gbest_score = float('inf')
        gbest_angle = 0
        gbest_pose = None

        # 评估初始粒子
        self.get_logger().info(f"🌀 PSO初始化: {n}个粒子，搜索半径={r}m")
        for i in range(n):
            score, angle, pose = self.find_best_angle(pos[i, 0], pos[i, 1], self.pso_angle_step)
            pbest_score[i] = score
            if score < gbest_score:
                gbest_score = score
                gbest_pos = pos[i].copy()
                gbest_angle = angle
                gbest_pose = pose
            self.get_logger().info(f"  粒子[{i+1}/{n}] ({pos[i,0]:.1f},{pos[i,1]:.1f}) score={score:.4f}")
            if gbest_score < self.fitness_score_threshold:
                self.get_logger().info("🔥 PSO初始化已达阈值，提前结束")
                return gbest_score, gbest_pos[0], gbest_pos[1], gbest_angle, gbest_pose

        # PSO迭代
        for iteration in range(self.pso_iterations):
            r1 = np.random.random((n, 2))
            r2 = np.random.random((n, 2))
            vel = (w * vel
                   + c1 * r1 * (pbest_pos - pos)
                   + c2 * r2 * (gbest_pos - pos))
            # 限制速度防止粒子飞出边界
            vel = np.clip(vel, -r * 0.3, r * 0.3)
            pos = pos + vel
            # 边界约束（反弹）
            lo = np.array([center_x - r, center_y - r])
            hi = np.array([center_x + r, center_y + r])
            mask_lo = pos < lo
            mask_hi = pos > hi
            vel[mask_lo] *= -0.5
            vel[mask_hi] *= -0.5
            pos = np.clip(pos, lo, hi)

            improved = False
            for i in range(n):
                score, angle, pose = self.find_best_angle(pos[i, 0], pos[i, 1], self.pso_angle_step)
                if score < pbest_score[i]:
                    pbest_score[i] = score
                    pbest_pos[i] = pos[i].copy()
                if score < gbest_score:
                    gbest_score = score
                    gbest_pos = pos[i].copy()
                    gbest_angle = angle
                    gbest_pose = pose
                    improved = True

            self.get_logger().info(
                f"  PSO迭代[{iteration+1}/{self.pso_iterations}] gbest=({gbest_pos[0]:.2f},{gbest_pos[1]:.2f}) "
                f"score={gbest_score:.4f}{'  ✨改进' if improved else ''}")

            if gbest_score < self.fitness_score_threshold:
                self.get_logger().info("🔥 PSO已达阈值，提前结束")
                break

        self.get_logger().info(f"✅ PSO完成: 最优点({gbest_pos[0]:.2f},{gbest_pos[1]:.2f}) score={gbest_score:.4f} angle={gbest_angle}°")
        return gbest_score, gbest_pos[0], gbest_pos[1], gbest_angle, gbest_pose

    def hill_climb(self, start_x, start_y, start_score, start_angle, start_pose):
        """从PSO最优点出发做精细爬山（best-improvement），返回 (score, x, y, angle, pose)"""
        current_x, current_y = start_x, start_y
        best_score = start_score
        best_angle = start_angle
        best_pose = start_pose
        step_size = self.initial_step
        directions = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)]

        for iteration in range(self.max_iterations):
            if step_size < self.min_step:
                break
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
                best_score = best_nb_score
                best_angle = best_nb_angle
                best_pose = best_nb_pose
                current_x, current_y = best_nb_x, best_nb_y
                self.get_logger().info(f"    爬山迭代{iteration+1}: ({current_x:.2f},{current_y:.2f}) score={best_score:.4f}")
                if best_score < self.fitness_score_threshold:
                    break
            else:
                step_size *= self.step_reduction

        # 精细角度搜索
        fine_score, fine_angle, fine_pose = self.find_best_angle(current_x, current_y, self.angle_fine_step)
        if fine_score < best_score:
            best_score, best_angle, best_pose = fine_score, fine_angle, fine_pose

        return best_score, current_x, current_y, best_angle, best_pose

    def execute_logic(self):
        """主执行逻辑 - PSO全局搜索 + 精细爬山"""

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

        # 3. PSO全局搜索
        pso_score, pso_x, pso_y, pso_angle, pso_pose = self.pso_search(gps_x, gps_y)

        if pso_pose is None:
            self.get_logger().error("❌ PSO搜索无有效结果")
            return False

        if pso_score < self.fitness_score_threshold:
            self.get_logger().info(f"🔥 PSO已达阈值，直接采用: ({pso_x:.2f},{pso_y:.2f}) score={pso_score:.4f}")
            self.current_transform = pso_pose
            self.timer = self.create_timer(0.1, self.publish_transform)
            return True

        # 4. 精细爬山（从PSO最优点出发）
        self.get_logger().info(f"🏃 从PSO最优点({pso_x:.2f},{pso_y:.2f})开始精细爬山...")
        final_score, final_x, final_y, final_angle, final_pose = \
            self.hill_climb(pso_x, pso_y, pso_score, pso_angle, pso_pose)

        # 5. 结算
        if final_pose and final_score < 10.0:
            self.get_logger().info(f"🏆 最终结果: ({final_x:.2f},{final_y:.2f}) 角度={final_angle}° fitness={final_score:.4f}")
            self.current_transform = final_pose
            self.timer = self.create_timer(0.1, self.publish_transform)
            return True
        else:
            self.get_logger().error(f"❌ 搜索失败 fitness={final_score:.4f}")
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