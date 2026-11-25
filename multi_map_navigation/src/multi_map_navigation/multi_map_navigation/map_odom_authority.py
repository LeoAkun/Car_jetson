#!/usr/bin/env python3
"""Own the single authoritative ``map -> odom`` transform.

The localization worker only publishes a session-bound result.  This node
accepts that result for the currently active localization session and is the
only long-lived broadcaster of ``map -> odom``.  Starting a new session clears
the previous transform immediately, so a delayed result or a stale TF cannot
make a new navigation stack appear initialized.
"""

import math
import threading
import fcntl
import os

import rclpy
from geometry_msgs.msg import TransformStamped
from multi_map_navigation.static_footprint_gate import StaticMapFootprintChecker
from multi_map_navigation_msgs.msg import (
    LioMapOdomCorrection,
    LocalizationResult,
    LocalizationSession,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformBroadcaster


def _normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid quaternion')
    return tuple(value / norm for value in quaternion)


def _quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return _normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def _rotate_vector(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _compose_transform(left, right):
    left_t, left_q = left
    right_t, right_q = right
    rotated = _rotate_vector(left_q, right_t)
    return (
        tuple(left_t[index] + rotated[index] for index in range(3)),
        _quaternion_multiply(left_q, right_q),
    )


def _inverse_transform(transform):
    translation, quaternion = transform
    inverse_q = (-quaternion[0], -quaternion[1],
                 -quaternion[2], quaternion[3])
    inverse_t = _rotate_vector(
        inverse_q, tuple(-value for value in translation))
    return inverse_t, inverse_q


def _quaternion_slerp(start, end, fraction):
    start = _normalize_quaternion(start)
    end = _normalize_quaternion(end)
    dot = sum(a * b for a, b in zip(start, end))
    if dot < 0.0:
        end = tuple(-value for value in end)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return _normalize_quaternion(tuple(
            start[index] + fraction * (end[index] - start[index])
            for index in range(4)))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    return tuple(
        (math.sin((1.0 - fraction) * theta) * start[index]
         + math.sin(fraction * theta) * end[index]) / sin_theta
        for index in range(4))


class MapOdomAuthority(Node):
    """Validate localization generations and broadcast their accepted TF."""

    MAX_FITNESS_SCORE = 0.05
    DIRECT_ACCEPT_THRESHOLD = 0.025
    MAX_RESULT_CLOUD_AGE_NS = 120_000_000_000
    # The continuous odom layer can drift by more than 5 cm/s even while the
    # scan-to-map pose remains stable.  Follow accepted global corrections at
    # 10 cm/s; the static-footprint gate still validates every published step.
    MAX_TRANSLATION_RATE = 0.10
    MAX_ROTATION_RATE = math.radians(0.5)
    SMALL_CORRECTION_TRANSLATION = 0.05
    SMALL_CORRECTION_ROTATION = math.radians(1.0)

    def __init__(self):
        super().__init__('map_odom_authority')
        self._acquire_authority_lock()
        self._lock = threading.RLock()
        self._active_session_id = ''
        self._active_map_id = ''
        self._session_start_ns = 0
        self._accepted_pose = None
        self._current_map_odom = None
        self._pending_result = None
        self._map_lio_transform = None
        self._target_map_odom = None
        self._active_lio_identity = None
        self._last_correction_epoch = 0
        self._rejected_correction_epochs = set()
        self._dynamic_fault_latched = False
        self._last_update_ns = 0
        self._footprint_checkers = {}
        self._latest_odom_base = None
        self.declare_parameter(
            'map_directory',
            '/home/akun/workspace/Car_jetson/multi_map_navigation/src/map')
        self._map_directory = self.get_parameter('map_directory').value

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._session_sub = self.create_subscription(
            LocalizationSession,
            '/localization_session',
            self.localization_session_callback,
            latched_qos,
        )
        self._result_sub = self.create_subscription(
            LocalizationResult,
            '/localization_result',
            self.localization_result_callback,
            latched_qos,
        )
        self._correction_sub = self.create_subscription(
            LioMapOdomCorrection,
            '/lio_sam/map_odom_correction',
            self.lio_correction_callback,
            20,
        )
        self._fault_pub = self.create_publisher(
            String, '/lio_sam/localization_fault', 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(0.1, self.publish_transform)
        self.get_logger().info('map->odom 唯一发布器已启动，等待定位会话')

    def _acquire_authority_lock(self):
        lock_path = '/tmp/multi_map_navigation_map_odom_authority.lock'
        lock_file = open(lock_path, 'a+', encoding='utf-8')
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            lock_file.close()
            raise RuntimeError('另一个 map_odom_authority 已持有 TF 所有权') from exc
        lock_file.seek(0)
        lock_file.truncate()
        lock_file.write(f'{os.getpid()}\n')
        lock_file.flush()
        self._authority_lock_file = lock_file

    @staticmethod
    def _stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _valid_pose(pose):
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return False
        norm_sq = (
            pose.orientation.x * pose.orientation.x
            + pose.orientation.y * pose.orientation.y
            + pose.orientation.z * pose.orientation.z
            + pose.orientation.w * pose.orientation.w
        )
        return 0.5 <= norm_sq <= 1.5

    @staticmethod
    def _pose_to_transform(pose):
        return (
            (pose.position.x, pose.position.y, pose.position.z),
            _normalize_quaternion((
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )),
        )

    @staticmethod
    def _message_transform_to_tuple(transform):
        return (
            (transform.translation.x,
             transform.translation.y,
             transform.translation.z),
            _normalize_quaternion((
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )),
        )

    @staticmethod
    def _transform_distance(left, right):
        relative = _compose_transform(_inverse_transform(left), right)
        translation = math.sqrt(sum(
            value * value for value in relative[0]))
        angle = 2.0 * math.acos(min(1.0, abs(relative[1][3])))
        return translation, angle

    @classmethod
    def _step_toward(cls, current, target, dt):
        translation_distance, rotation_distance = cls._transform_distance(
            current, target)
        if translation_distance < 1e-9 and rotation_distance < 1e-9:
            return target
        fraction = 1.0
        if translation_distance > 1e-9:
            fraction = min(
                fraction,
                cls.MAX_TRANSLATION_RATE * dt / translation_distance)
        if rotation_distance > 1e-9:
            fraction = min(
                fraction,
                cls.MAX_ROTATION_RATE * dt / rotation_distance)
        fraction = min(1.0, max(0.0, fraction))
        translation = tuple(
            current[0][index]
            + fraction * (target[0][index] - current[0][index])
            for index in range(3))
        quaternion = _quaternion_slerp(
            current[1], target[1], fraction)
        return translation, quaternion

    def _footprint_checker_locked(self):
        if not self._active_map_id:
            return None
        checker = self._footprint_checkers.get(self._active_map_id)
        if checker is None:
            yaml_path = os.path.join(
                self._map_directory,
                self._active_map_id,
                f'{self._active_map_id}_clean.yaml',
            )
            checker = StaticMapFootprintChecker(yaml_path)
            self._footprint_checkers[self._active_map_id] = checker
        return checker

    def _candidate_footprint_legal_locked(self, map_odom):
        try:
            odom_base_message = self._tf_buffer.lookup_transform(
                'odom', 'base_link', Time())
            odom_base = self._message_transform_to_tuple(
                odom_base_message.transform)
            self._latest_odom_base = odom_base
        except Exception:
            odom_base = self._latest_odom_base
        if odom_base is None:
            return False, 'odom -> base_link unavailable for footprint gate'
        map_base = _compose_transform(map_odom, odom_base)
        x, y, _ = map_base[0]
        qx, qy, qz, qw = map_base[1]
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        try:
            checker = self._footprint_checker_locked()
            if checker is None:
                return False, 'active map is unavailable'
            return checker.check_rectangular_footprint(x, y, yaw)
        except (OSError, ValueError, KeyError) as error:
            return False, f'static footprint checker failed: {error}'

    def _publish_dynamic_fault_locked(self, reason):
        message = String()
        message.data = reason
        self._fault_pub.publish(message)
        self.get_logger().error(reason)

    def lio_correction_callback(self, message):
        with self._lock:
            if not self._active_session_id or self._accepted_pose is None:
                return False
            if self._stamp_to_ns(message.header.stamp) < self._session_start_ns:
                return False
            identity = (message.instance_id, int(message.generation))
            if not identity[0] or identity[1] <= 0:
                return False
            if self._active_lio_identity is None:
                self._active_lio_identity = identity
            elif identity != self._active_lio_identity:
                self.get_logger().warning(
                    '拒绝非当前 LIO 实例的动态修正: '
                    f'{identity} != {self._active_lio_identity}')
                return False
            epoch = int(message.correction_epoch)
            if epoch < self._last_correction_epoch:
                return False
            if epoch == self._last_correction_epoch and epoch != 0:
                return True
            if self._dynamic_fault_latched:
                return False
            if not message.accepted:
                self._dynamic_fault_latched = True
                if epoch not in self._rejected_correction_epochs:
                    self._rejected_correction_epochs.add(epoch)
                    self._publish_dynamic_fault_locked(
                        'MAP_ODOM_CORRECTION_REJECTED: '
                        f'epoch={epoch}; state={message.state}; '
                        f'reason={message.reason}')
                return False

            try:
                lio_map_odom = self._message_transform_to_tuple(
                    message.lio_map_to_odom_target)
            except ValueError as error:
                self._dynamic_fault_latched = True
                self._publish_dynamic_fault_locked(
                    f'MAP_ODOM_CORRECTION_INVALID: {error}')
                return False

            current_map_odom = (
                self._current_map_odom
                or self._pose_to_transform(self._accepted_pose))
            if self._map_lio_transform is None:
                self._map_lio_transform = _compose_transform(
                    current_map_odom, _inverse_transform(lio_map_odom))
            target = _compose_transform(
                self._map_lio_transform, lio_map_odom)
            translation, rotation = self._transform_distance(
                current_map_odom, target)

            # This is the backlog between the currently rate-limited map->odom
            # transform and its accepted target.  It is allowed to accumulate
            # while odom drifts and must not be classified as a single pose
            # jump.  The instantaneous scan-to-map residual is validated by
            # LIO before this message is accepted.

            residual_translation = abs(float(message.translation_residual))
            residual_rotation = abs(float(message.yaw_residual))
            is_medium = (
                residual_translation > self.SMALL_CORRECTION_TRANSLATION
                or residual_rotation > self.SMALL_CORRECTION_ROTATION
            )
            if is_medium:
                self.get_logger().warning(
                    '接受中等动态修正并限速跟随: '
                    f'epoch={epoch}, residual_translation='
                    f'{residual_translation:.6f}, residual_rotation='
                    f'{residual_rotation:.6f}, target_backlog_translation='
                    f'{translation:.6f}, target_backlog_rotation='
                    f'{rotation:.6f}')

            self._target_map_odom = target
            if translation < 1e-9 and rotation < 1e-9:
                self._target_map_odom = None
            self._last_correction_epoch = epoch
            return True

    def localization_session_callback(self, message):
        with self._lock:
            if not message.active:
                if (not message.session_id
                        or message.session_id == self._active_session_id):
                    self._clear_session_locked()
                    self.get_logger().info('定位会话已关闭，停止发布 map->odom')
                return

            changed = (
                message.session_id != self._active_session_id
                or message.map_id != self._active_map_id
                or int(message.session_start_ns) != self._session_start_ns
            )
            if changed:
                self._accepted_pose = None
                self._current_map_odom = None
                self._map_lio_transform = None
                self._target_map_odom = None
                self._active_lio_identity = None
                self._last_correction_epoch = 0
                self._rejected_correction_epochs.clear()
                self._dynamic_fault_latched = False
                self._last_update_ns = 0
            self._active_session_id = message.session_id
            self._active_map_id = message.map_id
            self._session_start_ns = int(message.session_start_ns)
            self.get_logger().info(
                '激活定位会话: '
                f'session={message.session_id}, map={message.map_id}')

            pending = self._pending_result
            self._pending_result = None
            if pending is not None:
                self._accept_result_locked(pending)

    def localization_result_callback(self, message):
        with self._lock:
            if not self._active_session_id:
                # Both topics are transient-local, but DDS discovery does not
                # guarantee callback order.  Hold at most the latest result.
                self._pending_result = message
                return
            self._accept_result_locked(message)

    def _accept_result_locked(self, message):
        if (message.session_id != self._active_session_id
                or message.map_id != self._active_map_id):
            self.get_logger().warning(
                '拒绝旧定位结果: '
                f'session={message.session_id}, map={message.map_id}')
            return False
        if not message.success:
            self.get_logger().error(
                f'本轮重定位失败，不发布 TF: {message.reason}')
            self._accepted_pose = None
            return False
        cloud_stamp_ns = self._stamp_to_ns(message.cloud_stamp)
        if cloud_stamp_ns < self._session_start_ns:
            self.get_logger().error(
                '拒绝使用会话开始前的点云: '
                f'{cloud_stamp_ns} < {self._session_start_ns}')
            return False
        if not math.isfinite(message.fitness_score):
            self.get_logger().error('拒绝非有限定位分数')
            return False
        if float(message.fitness_score) > self.MAX_FITNESS_SCORE:
            self.get_logger().error(
                '拒绝超过硬阈值的定位分数: '
                f'{message.fitness_score:.6f} > {self.MAX_FITNESS_SCORE:.6f}')
            return False
        if int(message.observation_id) <= 0:
            self.get_logger().error('拒绝缺少 observation_id 的定位结果')
            return False
        if int(message.confirmation_count) < 1:
            self.get_logger().error('拒绝缺少确认计数的定位结果')
            return False
        if not math.isfinite(float(message.first_fitness_score)):
            self.get_logger().error('拒绝缺少第一帧有限分数的定位结果')
            return False
        if (max(
                float(message.fitness_score),
                float(message.first_fitness_score),
        ) >= self.DIRECT_ACCEPT_THRESHOLD
                and int(message.confirmation_count) < 2):
            self.get_logger().error('拒绝未经独立点云二次确认的弱定位结果')
            return False
        if int(message.confirmation_count) >= 2:
            first_cloud_ns = self._stamp_to_ns(message.first_cloud_stamp)
            if first_cloud_ns <= 0 or cloud_stamp_ns <= first_cloud_ns:
                self.get_logger().error('拒绝未使用更新点云的二次确认结果')
                return False
        try:
            cloud_age_ns = self.get_clock().now().nanoseconds - cloud_stamp_ns
        except (AttributeError, RuntimeError):
            cloud_age_ns = 0
        if (cloud_age_ns < -500_000_000
                or cloud_age_ns > self.MAX_RESULT_CLOUD_AGE_NS):
            self.get_logger().error(
                f'拒绝时间异常或过期的定位点云: age={cloud_age_ns / 1e9:.3f}s')
            return False
        if not self._valid_pose(message.pose):
            self.get_logger().error('拒绝无效定位位姿')
            return False

        self._accepted_pose = message.pose
        self._current_map_odom = self._pose_to_transform(message.pose)
        self._target_map_odom = None
        self._map_lio_transform = None
        self._active_lio_identity = None
        self._last_correction_epoch = 0
        self._rejected_correction_epochs.clear()
        self._dynamic_fault_latched = False
        self._last_update_ns = 0
        self.get_logger().info(
            '定位结果已成为唯一 map->odom 来源: '
            f'session={message.session_id}, map={message.map_id}, '
            f'fitness={message.fitness_score:.6f}')
        return True

    def _clear_session_locked(self):
        self._active_session_id = ''
        self._active_map_id = ''
        self._session_start_ns = 0
        self._accepted_pose = None
        self._current_map_odom = None
        self._pending_result = None
        self._map_lio_transform = None
        self._target_map_odom = None
        self._active_lio_identity = None
        self._last_correction_epoch = 0
        self._rejected_correction_epochs.clear()
        self._dynamic_fault_latched = False
        self._last_update_ns = 0

    def publish_transform(self):
        with self._lock:
            current = self._current_map_odom
            session_id = self._active_session_id
            map_id = self._active_map_id
            if current is None or not session_id:
                return

            now_ns = self.get_clock().now().nanoseconds
            if self._target_map_odom is not None:
                dt = 0.1 if self._last_update_ns <= 0 else max(
                    0.0, min(0.5, (now_ns - self._last_update_ns) / 1e9))
                candidate = self._step_toward(
                    current, self._target_map_odom, dt)
                legal, reason = self._candidate_footprint_legal_locked(
                    candidate)
                if legal:
                    current = candidate
                    self._current_map_odom = current
                    remaining = self._transform_distance(
                        current, self._target_map_odom)
                    if remaining[0] < 1e-6 and remaining[1] < 1e-6:
                        self._target_map_odom = None
                else:
                    if reason.startswith('odom -> base_link unavailable'):
                        self.get_logger().warning(reason)
                    else:
                        self._target_map_odom = None
                        self._dynamic_fault_latched = True
                        self._publish_dynamic_fault_locked(
                            'MAP_ODOM_CORRECTION_FOOTPRINT_REJECTED: '
                            f'{reason}')
            self._last_update_ns = now_ns

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = current[0][0]
        transform.transform.translation.y = current[0][1]
        transform.transform.translation.z = current[0][2]
        transform.transform.rotation.x = current[1][0]
        transform.transform.rotation.y = current[1][1]
        transform.transform.rotation.z = current[1][2]
        transform.transform.rotation.w = current[1][3]
        self._tf_broadcaster.sendTransform(transform)
        self.get_logger().debug(
            f'发布 map->odom: session={session_id}, map={map_id}')


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomAuthority()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
