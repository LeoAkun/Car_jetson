import rclpy
import math
import os
import sys
import time
import numpy as np
import itertools
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from re_localization.srv import ReLocalization
from multi_map_navigation_msgs.msg import LocalizationResult
from multi_map_navigation.static_footprint_gate import StaticMapFootprintChecker
from sensor_msgs.msg import NavSatFix


class LocalizationSearchAborted(RuntimeError):
    """Stop all candidate loops when infrastructure cannot evaluate this run."""


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
        """
        经纬度转换为局部EN坐标
        """
        # 当前点经纬度的弧度值
        rad_lat = np.radians(lat)
        rad_lon = np.radians(lon)
        # CSV 第一行取出来的参考点的弧度值
        rad_anchor_lat = np.radians(self.anchor_lat)
        rad_anchor_lon = np.radians(self.anchor_lon)
        # 当前点与参考点的弧度差
        delta_lat = rad_lat - rad_anchor_lat
        delta_lon = rad_lon - rad_anchor_lon
        # 当前点与参考点的距离
        north = delta_lat * self.EARTH_RADIUS
        east  = delta_lon * self.EARTH_RADIUS * np.cos(rad_anchor_lat)

        if np.isscalar(lat):
            return np.array([east, north])
        else:
            return np.column_stack([east, north])

    def _fit_transform_6param(self):
        """
        用标定数据把GPS局部EN坐标系和地图坐标系之间的线性关系拟合出来
        """
        # 取出所有点的经纬度
        lats = self.calibration_data[:, 0]
        lons = self.calibration_data[:, 1]
        enu = self._latlon_to_enu(lats, lons)
        # 构造源矩阵，每个点[east, north, 1]
        src = np.column_stack([enu, np.ones(len(enu))])
        # 取出地图坐标（拟合目标）
        dst = self.calibration_data[:, 2:4]
        # 最小二乘法拟合，得到仿射变换矩阵
        M, residuals, rank, s = np.linalg.lstsq(src, dst, rcond=None)
        self.affine_matrix = np.vstack([M.T, [0.0, 0.0, 1.0]])

    def gps_to_map(self, lat, lon):
        """
        对外真正使用的转换函数
        """
        enu = self._latlon_to_enu(lat, lon)
        pt = np.array([enu[0], enu[1], 1.0])
        # 仿射变换矩阵和当前点EN坐标相乘，得到地图坐标
        map_pt = self.affine_matrix @ pt
        return map_pt[0], map_pt[1]

# --- 2. 核心节点 ---
class PoseInitNode(Node):
    SERVICE_NOT_READY_MESSAGES = frozenset((
        'map or initial pose is not ready',
        'localization lifecycle is not active',
        'registration target is not ready',
    ))

    def __init__(self):
        super().__init__('pose_init_node')

                
        self.declare_parameter('map_csv', '')
        self.map_csv = self.get_parameter('map_csv').value
        self.declare_parameter('localization_session_id', '')
        self.declare_parameter('map_name', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('session_start_ns', 0)
        self.declare_parameter('use_initial_pose', False)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.localization_session_id = self.get_parameter(
            'localization_session_id').value
        self.map_name = self.get_parameter('map_name').value
        self.map_yaml = self.get_parameter('map_yaml').value
        self.session_start_ns = int(self.get_parameter('session_start_ns').value)
        if self.session_start_ns <= 0:
            self.session_start_ns = self.get_clock().now().nanoseconds
        self.use_initial_pose = bool(
            self.get_parameter('use_initial_pose').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_yaw = float(self.get_parameter('initial_yaw').value)
        # 创建一个坐标转换器，用于将GPS坐标转换为地图坐标
        self.converter = GpsToMapConverter(self.map_csv)
        
        self.latest_gps = None
        self.localization_cloud_stamp = None
        self.first_localization_cloud_stamp = None
        self.first_fitness_score = float('inf')
        self.observation_id = 1
        self.observation_has_cloud = False
        self._observation_min_cloud_stamp = None
        self.confirmation_count = 0
        self.localization_result_published = False
        self.localization_deadline = None
        self.max_localization_duration = 120.0
        self.max_cloud_age = 120.0
        self.direct_accept_threshold = 0.025
        self.fitness_score_threshold = 0.05
        self.confirmation_yaw_tolerance = math.radians(3.0)
        self.footprint_checker = StaticMapFootprintChecker(self.map_yaml)

        result_qos = QoSProfile(depth=1)
        result_qos.reliability = ReliabilityPolicy.RELIABLE
        result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.localization_result_pub = self.create_publisher(
            LocalizationResult, '/localization_result', result_qos)

        # --- 核心参数配置 ---
        # 粒子群优化（PSO）设置
        self.pso_particles = 30         # 粒子数量
        self.pso_iterations = 6        # PSO迭代轮数
        self.pso_search_radius = 10.0   # 搜索半径 (米，以GPS点为中心)
        self.pso_w = 0.7                # 惯性权重
        self.pso_c1 = 1.5               # 认知系数（向自身历史最优靠拢）
        self.pso_c2 = 1.5               # 社会系数（向全局最优靠拢）
        self.pso_angle_step = 30        # PSO阶段粗角度步长 (度) 一圈12个方向

        # 精细爬山设置（PSO收敛后使用）
        self.initial_step = 1.0         # 初始搜索步长 (米)
        self.min_step = 0.2             # 最小搜索步长 (米)
        self.step_reduction = 0.5       # 步长缩减因子
        self.max_iterations = 30        # 爬山最大迭代次数
        self.angle_step = 15            # 爬山角度步长 (度)
        self.angle_fine_step = 15       # 精细角度步长 (度)

        # 创建一个客户端，调用重定位服务
        self.re_localization_client = self.create_client(ReLocalization, '/re_localization')
        # 创建一个订阅器，订阅GPS数据
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
        """
        欧拉角转四元数
        """
        roll = math.radians(roll_deg) * 0.5
        pitch = math.radians(pitch_deg) * 0.5
        yaw = math.radians(yaw_deg) * 0.5
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [x, y, z, w]

    @staticmethod
    def _stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _response_matches_session(self, response):
        if response.session_id != self.localization_session_id:
            self.get_logger().error(
                'Rejecting localization response from another session: '
                f'{response.session_id!r}')
            return False
        if response.map_id != self.map_name:
            self.get_logger().error(
                'Rejecting localization response for another map: '
                f'{response.map_id!r}')
            return False
        cloud_stamp_ns = self._stamp_to_ns(response.cloud_stamp)
        if cloud_stamp_ns < self.session_start_ns:
            self.get_logger().error(
                'Rejecting localization response based on stale point cloud: '
                f'{cloud_stamp_ns} < {self.session_start_ns}')
            return False
        if int(response.observation_id) != self.observation_id:
            self.get_logger().error(
                'Rejecting localization response for another observation: '
                f'{response.observation_id} != {self.observation_id}')
            return False
        return True

    def publish_localization_result(self, success, score, pose=None, reason=''):
        if self.localization_result_published:
            return
        result = LocalizationResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = 'map'
        result.session_id = self.localization_session_id
        result.map_id = self.map_name
        result.success = bool(success)
        result.fitness_score = float(score)
        if pose is not None:
            result.pose = pose
        if self.localization_cloud_stamp is not None:
            result.cloud_stamp = self.localization_cloud_stamp
        result.observation_id = int(self.observation_id)
        result.confirmation_count = int(self.confirmation_count)
        result.first_fitness_score = float(self.first_fitness_score)
        if self.first_localization_cloud_stamp is not None:
            result.first_cloud_stamp = self.first_localization_cloud_stamp
        result.reason = reason
        self.localization_result_pub.publish(result)
        self.localization_result_published = True

    def accept_localization(self, pose, score, reason='localization accepted'):
        self.publish_localization_result(
            True, score, pose, reason)

    @staticmethod
    def _pose_yaw(pose):
        q = pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def _angular_distance(first, second):
        return abs(math.atan2(
            math.sin(first - second), math.cos(first - second)))

    def _pose_delta(self, first, second):
        distance = math.hypot(
            first.position.x - second.position.x,
            first.position.y - second.position.y)
        yaw_delta = self._angular_distance(
            self._pose_yaw(first), self._pose_yaw(second))
        return distance, yaw_delta

    def _prior_is_consistent(self, pose, prior_x, prior_y, prior_yaw,
                             distance_limit, yaw_limit=None):
        distance = math.hypot(
            pose.position.x - prior_x, pose.position.y - prior_y)
        if distance > distance_limit:
            return False, f'prior distance {distance:.3f}m > {distance_limit:.3f}m'
        if yaw_limit is not None:
            yaw_delta = self._angular_distance(
                self._pose_yaw(pose), prior_yaw)
            if yaw_delta > yaw_limit:
                return False, (
                    f'prior yaw delta {math.degrees(yaw_delta):.2f}deg > '
                    f'{math.degrees(yaw_limit):.2f}deg')
        return True, f'prior distance {distance:.3f}m'

    def _static_footprint_is_legal(self, pose):
        return self.footprint_checker.check_rectangular_footprint(
            pose.position.x,
            pose.position.y,
            self._pose_yaw(pose),
            half_length=0.43,
            half_width=0.28,
            # Nav2 footprint_padding=0.02 plus the planned 0.05 startup margin.
            padding=0.07,
        )

    def _cloud_age(self, stamp):
        if stamp is None:
            return float('inf')
        return max(
            0.0,
            (self.get_clock().now().nanoseconds - self._stamp_to_ns(stamp))
            / 1_000_000_000.0,
        )

    def _start_new_observation(self, min_cloud_stamp):
        self.observation_id += 1
        self.observation_has_cloud = False
        self._observation_min_cloud_stamp = min_cloud_stamp

    def _local_confirm(self, reference_pose):
        center_x = reference_pose.position.x
        center_y = reference_pose.position.y
        center_yaw = math.degrees(self._pose_yaw(reference_pose))
        candidates = [(0.0, 0.0)]
        candidates.extend((dx, dy) for dx, dy in (
            (0.2, 0.0), (-0.2, 0.0), (0.0, 0.2), (0.0, -0.2)))
        best = (float('inf'), None, None)
        for dx, dy in candidates:
            for yaw_offset in (0.0, -3.0, 3.0):
                score, pose, cloud_stamp = self._evaluate_candidate(
                    center_x + dx, center_y + dy,
                    center_yaw + yaw_offset)
                if score < best[0]:
                    best = (score, pose, cloud_stamp)
        return best

    def validate_and_accept(self, pose, score, *, prior_x, prior_y,
                            prior_yaw=None, prior_distance_limit=15.0,
                            prior_yaw_limit=None):
        """Apply prior, static-map and adaptive independent-observation gates."""
        self.confirmation_count = 0
        if self.localization_deadline is None:
            self.localization_deadline = float('inf')
        if time.monotonic() >= self.localization_deadline:
            self.publish_localization_result(
                False, score, reason='localization exceeded 120 second deadline')
            return False
        if pose is None or not math.isfinite(score):
            self.publish_localization_result(
                False, score, reason='localization returned no finite candidate')
            return False
        self.first_fitness_score = score
        self.first_localization_cloud_stamp = self.localization_cloud_stamp

        consistent, prior_reason = self._prior_is_consistent(
            pose, prior_x, prior_y, prior_yaw,
            prior_distance_limit, prior_yaw_limit)
        if not consistent:
            self.publish_localization_result(
                False, score, reason=f'localization prior mismatch: {prior_reason}')
            return False

        legal, footprint_reason = self._static_footprint_is_legal(pose)
        cloud_is_old = self._cloud_age(self.localization_cloud_stamp) > self.max_cloud_age
        needs_confirmation = score >= self.direct_accept_threshold
        poor_score_retried = False
        if score > self.fitness_score_threshold:
            previous_stamp = self.localization_cloud_stamp
            self._start_new_observation(previous_stamp)
            retry_score, retry_pose, retry_stamp = self._local_confirm(pose)
            if (retry_pose is None
                    or retry_score > self.fitness_score_threshold):
                self.localization_cloud_stamp = retry_stamp or previous_stamp
                self.publish_localization_result(
                    False, retry_score,
                    reason='fresh-cloud retry did not meet hard threshold')
                return False
            pose = retry_pose
            score = retry_score
            self.localization_cloud_stamp = retry_stamp
            self.first_fitness_score = retry_score
            self.first_localization_cloud_stamp = retry_stamp
            consistent, prior_reason = self._prior_is_consistent(
                pose, prior_x, prior_y, prior_yaw,
                prior_distance_limit, prior_yaw_limit)
            legal, footprint_reason = self._static_footprint_is_legal(pose)
            if not consistent:
                self.publish_localization_result(
                    False, score,
                    reason=f'fresh-cloud retry prior mismatch: {prior_reason}')
                return False
            cloud_is_old = self._cloud_age(retry_stamp) > self.max_cloud_age
            needs_confirmation = score >= self.direct_accept_threshold
            poor_score_retried = True

        if legal and not needs_confirmation and not cloud_is_old:
            self.confirmation_count = 1
            self.accept_localization(
                pose, score,
                f'strong localization accepted; {prior_reason}; {footprint_reason}')
            return True

        reference_pose = pose
        previous_stamp = self.localization_cloud_stamp
        reason = (
            'weak score' if needs_confirmation else
            'stale cloud' if cloud_is_old else 'illegal static footprint')
        self.get_logger().warning(
            f'{reason}; starting fresh-cloud local confirmation')

        # One normal confirmation plus one fresh retry for an illegal/weak result.
        confirmation_attempts = 1 if poor_score_retried else 2
        for attempt in range(confirmation_attempts):
            if time.monotonic() >= self.localization_deadline:
                break
            self._start_new_observation(previous_stamp)
            confirm_score, confirm_pose, confirm_stamp = self._local_confirm(
                reference_pose)
            if confirm_pose is None or confirm_score > self.fitness_score_threshold:
                previous_stamp = confirm_stamp or previous_stamp
                continue
            distance, yaw_delta = self._pose_delta(reference_pose, confirm_pose)
            # Translation delta is diagnostic only; retain the yaw gate.
            if yaw_delta > self.confirmation_yaw_tolerance:
                previous_stamp = confirm_stamp
                self.get_logger().warning(
                    '独立点云位姿不一致: '
                    f'{distance:.3f}m, {math.degrees(yaw_delta):.2f}deg')
                continue
            consistent, prior_reason = self._prior_is_consistent(
                confirm_pose, prior_x, prior_y, prior_yaw,
                prior_distance_limit, prior_yaw_limit)
            legal, footprint_reason = self._static_footprint_is_legal(confirm_pose)
            if not consistent or not legal:
                previous_stamp = confirm_stamp
                continue
            self.localization_cloud_stamp = confirm_stamp
            self.confirmation_count = 2
            self.accept_localization(
                confirm_pose, confirm_score,
                'fresh-cloud localization confirmed; '
                f'delta={distance:.3f}m/{math.degrees(yaw_delta):.2f}deg; '
                f'{prior_reason}; {footprint_reason}')
            return True

        self.publish_localization_result(
            False, score,
            reason=f'fresh-cloud confirmation failed after {reason}')
        return False

    def find_best_angle(self, x, y, angle_step):
        """在给定位置找到最佳角度"""
        best_score = float('inf')
        best_angle = 0
        best_pose = None

        for yaw in range(0, 360, angle_step):
            score, pose, cloud_stamp = self._evaluate_candidate(x, y, yaw)
            if score < best_score:
                best_score = score
                best_angle = yaw
                best_pose = pose
                self.localization_cloud_stamp = cloud_stamp

        return best_score, best_angle, best_pose

    def _localization_time_remaining(self, phase):
        """Use the same monotonic deadline for discovery and service replies."""
        if self.localization_deadline is None:
            self.localization_deadline = (
                time.monotonic() + self.max_localization_duration)
        remaining = self.localization_deadline - time.monotonic()
        if remaining <= 0.0:
            raise LocalizationSearchAborted(
                'localization exceeded 120 second deadline while ' + phase)
        return remaining

    def _evaluate_candidate(self, x, y, yaw_deg):
        """Evaluate one session-bound pose candidate."""
        self._localization_time_remaining('evaluating a candidate')
        request = ReLocalization.Request()
        q = self.euler_to_quaternion(0, 0, yaw_deg)
        request.initial_pose.header.frame_id = 'map'
        request.initial_pose.header.stamp = self.get_clock().now().to_msg()
        request.initial_pose.pose.pose.position.x = float(x)
        request.initial_pose.pose.pose.position.y = float(y)
        request.initial_pose.pose.pose.position.z = 0.0
        request.initial_pose.pose.pose.orientation.x = q[0]
        request.initial_pose.pose.pose.orientation.y = q[1]
        request.initial_pose.pose.pose.orientation.z = q[2]
        request.initial_pose.pose.pose.orientation.w = q[3]
        request.session_id = self.localization_session_id
        request.map_id = self.map_name
        request.session_start.sec = self.session_start_ns // 1_000_000_000
        request.session_start.nanosec = self.session_start_ns % 1_000_000_000
        request.observation_id = int(self.observation_id)
        request.refresh_cloud = not self.observation_has_cloud
        min_stamp = self._observation_min_cloud_stamp
        if min_stamp is None:
            request.min_cloud_stamp.sec = (
                self.session_start_ns // 1_000_000_000)
            request.min_cloud_stamp.nanosec = (
                self.session_start_ns % 1_000_000_000)
        else:
            request.min_cloud_stamp = min_stamp
        future = self.re_localization_client.call_async(request)
        try:
            remaining = self._localization_time_remaining(
                'waiting for /re_localization response')
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=remaining)
            if not future.done():
                raise LocalizationSearchAborted(
                    'localization exceeded 120 second deadline while '
                    'waiting for /re_localization response')
            self._localization_time_remaining(
                'waiting for /re_localization response')
        except LocalizationSearchAborted:
            # This cancels only our local pending future, not a vehicle command
            # or the server's already-running registration callback.
            future.cancel()
            raise
        try:
            response = future.result()
            if (not response.success
                    and response.message in self.SERVICE_NOT_READY_MESSAGES):
                raise LocalizationSearchAborted(
                    'localization service not ready: ' + response.message)
            response_cloud_ns = self._stamp_to_ns(response.cloud_stamp)
            if (int(response.observation_id) == self.observation_id
                    and response_cloud_ns >= self.session_start_ns):
                # Registration can fail after the observation has already been
                # cached.  Later candidates must reuse that exact observation.
                self.observation_has_cloud = True
            if response.success and self._response_matches_session(response):
                return (
                    float(response.fitness_score),
                    response.pose.pose.pose,
                    response.cloud_stamp,
                )
        except LocalizationSearchAborted:
            raise
        except Exception as exc:
            self.get_logger().warning(
                f'Localization service call failed: {exc}')
        return float('inf'), None, None

    def seeded_search(self):
        """Validate a map-switch prior in a bounded local neighborhood."""
        center_x = self.initial_x
        center_y = self.initial_y
        center_yaw = math.degrees(self.initial_yaw)
        offsets = (-1.0, -0.5, 0.0, 0.5, 1.0)
        yaw_offsets = (0.0, -10.0, 10.0, -20.0, 20.0, -30.0, 30.0)
        best = (float('inf'), center_x, center_y, center_yaw, None, None)

        self.get_logger().info(
            '使用地图切换先验做局部重定位: '
            f'({center_x:.2f}, {center_y:.2f}, {center_yaw:.1f}deg)')
        for dx in offsets:
            for dy in offsets:
                for yaw_offset in yaw_offsets:
                    yaw = (center_yaw + yaw_offset) % 360.0
                    score, pose, cloud_stamp = self._evaluate_candidate(
                        center_x + dx, center_y + dy, yaw)
                    if score < best[0]:
                        best = (
                            score, center_x + dx, center_y + dy,
                            yaw, pose, cloud_stamp)
                    if score < self.fitness_score_threshold:
                        self.localization_cloud_stamp = cloud_stamp
                        return best[:5]

        self.localization_cloud_stamp = best[5]
        return best[:5]

    def pso_search(self, center_x, center_y):
        """
        粒子群优化搜索全局最优位置，避免局部最优
        输出 最优分数score; 最优位置x,y; 最优角度angle; 最优位姿pose
        """
        n = self.pso_particles
        r = self.pso_search_radius
        w = self.pso_w
        c1 = self.pso_c1
        c2 = self.pso_c2

        # 初始化粒子位置（在搜索范围内均匀随机分布）
        np.random.seed(42)
        # 随机生成n个粒子，每个粒子位置在搜索半径r内，中心点为(center_x, center_y)的方形区域内
        pos = np.random.uniform(-r, r, (n, 2)) + np.array([center_x, center_y])
        # 每个粒子速度在[-1.0, 1.0]范围内
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
            # 每轮迭代都生成两个随机矩阵，个体学习随机因子和群体学习随机因子
            r1 = np.random.random((n, 2))
            r2 = np.random.random((n, 2))
            vel = (w * vel
                   + c1 * r1 * (pbest_pos - pos)
                   + c2 * r2 * (gbest_pos - pos))
            # 限制速度防止粒子飞出边界，单步速度最多允许到 3 米
            vel = np.clip(vel, -r * 0.3, r * 0.3)
            # 更新粒子位置
            pos = pos + vel
            # 边界约束（反弹）
            lo = np.array([center_x - r, center_y - r])
            hi = np.array([center_x + r, center_y + r])
            # 找出越界的粒子
            mask_lo = pos < lo
            mask_hi = pos > hi
            # 越界的粒子速度反向，速度减半
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
                # 如果8个方向都没有更优点，则缩小步长
                step_size *= self.step_reduction

        # 精细角度搜索
        fine_score, fine_angle, fine_pose = self.find_best_angle(current_x, current_y, self.angle_fine_step)
        if fine_score < best_score:
            best_score, best_angle, best_pose = fine_score, fine_angle, fine_pose

        return best_score, current_x, current_y, best_angle, best_pose

    def execute_logic(self):
        """Bound the entire search and publish explicit infrastructure failures."""
        self.localization_deadline = (
            time.monotonic() + self.max_localization_duration)
        try:
            return self._execute_search()
        except LocalizationSearchAborted as exc:
            reason = str(exc)
            self.get_logger().error(reason)
            self.publish_localization_result(
                False, float('inf'), reason=reason)
            return False

    def _execute_search(self):
        """主执行逻辑 - PSO全局搜索 + 精细爬山"""

        # 1. 等待服务
        while True:
            remaining = self._localization_time_remaining(
                'waiting for /re_localization service')
            if self.re_localization_client.wait_for_service(
                    timeout_sec=min(1.0, remaining)):
                break
            self.get_logger().info('等待 /re_localization 服务...')
            remaining = self._localization_time_remaining(
                'waiting for /re_localization service')
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

        if self.use_initial_pose:
            score, x, y, yaw, pose = self.seeded_search()
            if pose is not None and math.isfinite(score):
                self.get_logger().info(
                    '地图切换先验验证成功: '
                    f'({x:.2f}, {y:.2f}), yaw={yaw:.1f}, '
                    f'fitness={score:.6f}')
                return self.validate_and_accept(
                    pose,
                    score,
                    prior_x=self.initial_x,
                    prior_y=self.initial_y,
                    prior_yaw=self.initial_yaw,
                    prior_distance_limit=1.5,
                    prior_yaw_limit=math.radians(35.0),
                )
            self.get_logger().error(
                f'地图切换先验附近重定位失败: fitness={score:.6f}')
            self.publish_localization_result(
                False, score,
                reason='map-switch prior failed bounded localization')
            return False

        # 2. 等待 GPS
        self.get_logger().info('等待 GPS 数据...')
        while self.latest_gps is None:
            if time.monotonic() >= self.localization_deadline:
                self.publish_localization_result(
                    False, float('inf'), reason='GPS wait exceeded 120 seconds')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        lat = self.latest_gps.latitude
        lon = self.latest_gps.longitude
        gps_x, gps_y = self.converter.gps_to_map(lat, lon)
        self.get_logger().info(f"✅ GPS起始点: ({gps_x:.2f}, {gps_y:.2f})")

        # 3. PSO全局搜索
        pso_score, pso_x, pso_y, pso_angle, pso_pose = self.pso_search(gps_x, gps_y)

        if pso_pose is None:
            self.get_logger().error("❌ PSO搜索无有效结果")
            self.publish_localization_result(
                False, pso_score, reason='PSO returned no valid pose')
            return False

        if pso_score <= self.fitness_score_threshold:
            self.get_logger().info(f"🔥 PSO已达阈值，直接采用: ({pso_x:.2f},{pso_y:.2f}) score={pso_score:.4f}")
            return self.validate_and_accept(
                pso_pose,
                pso_score,
                prior_x=gps_x,
                prior_y=gps_y,
                prior_distance_limit=15.0,
            )

        # 4. 精细爬山（从PSO最优点出发）
        self.get_logger().info(f"🏃 从PSO最优点({pso_x:.2f},{pso_y:.2f})开始精细爬山...")
        final_score, final_x, final_y, final_angle, final_pose = \
            self.hill_climb(pso_x, pso_y, pso_score, pso_angle, pso_pose)

        # Keep the final acceptance gate consistent with the search threshold.
        if final_pose is not None and math.isfinite(final_score):
            self.get_logger().info(f"🏆 最终结果: ({final_x:.2f},{final_y:.2f}) 角度={final_angle}° fitness={final_score:.4f}")
            return self.validate_and_accept(
                final_pose,
                final_score,
                prior_x=gps_x,
                prior_y=gps_y,
                prior_distance_limit=15.0,
            )
        else:
            self.get_logger().error(f"❌ 搜索失败 fitness={final_score:.4f}")
            self.publish_localization_result(
                False, final_score,
                reason='fitness score did not meet acceptance threshold')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = PoseInitNode()
    exit_code = 0
    
    try:
        # 使用 execute_logic 接管流程
        if node.execute_logic():
            # 结果由独立 map_odom_authority 校验并维护 TF。保留本节点，
            # 使进程管理器能持续确认本轮定位 worker 的健康状态。
            print("[System] 初始化成功，定位结果已提交给 TF 唯一发布器...")
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=1.0)
        else:
            exit_code = 2
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)
