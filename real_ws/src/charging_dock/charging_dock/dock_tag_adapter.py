# Copyright 2026 akun
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections import Counter
import math

from apriltag_msgs.msg import AprilTagDetectionArray
from charging_dock_msgs.msg import DockTagObservation
from charging_dock_msgs.msg import DockTagObservationArray
from geometry_msgs.msg import Point32
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from charging_dock.tag_config import build_tag_registry
from charging_dock.tag_config import duplicate_registered_ids
from charging_dock.tag_config import normalize_family
from charging_dock.tag_config import TagConfigError


class DockTagAdapter(Node):
    """Convert version-specific AprilTag results into project observations."""

    def __init__(self):
        super().__init__('dock_tag_adapter')

        self.declare_parameter('family', '36h11')
        self.declare_parameter('tag.ids', [0])
        self.declare_parameter('tag.frames', ['dock_tag_0'])
        self.declare_parameter('tag.sizes', [0.16])
        self.declare_parameter('max_hamming', 0)
        self.declare_parameter('min_decision_margin', 0.0)
        self.declare_parameter('max_data_age_sec', 0.20)
        self.declare_parameter('future_tolerance_sec', 0.05)
        self.declare_parameter('tf_lookup_timeout_sec', 0.05)
        self.declare_parameter('statistics_period_sec', 10.0)
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter(
            'observations_topic', '/charging_dock/tag_observations'
        )

        self._family = normalize_family(
            self.get_parameter('family').get_parameter_value().string_value
        )
        if not self._family:
            raise TagConfigError('family must not be empty')

        self._registry = build_tag_registry(
            self.get_parameter('tag.ids').value,
            self.get_parameter('tag.frames').value,
            self.get_parameter('tag.sizes').value,
        )
        self._max_hamming = int(self.get_parameter('max_hamming').value)
        self._min_margin = float(self.get_parameter('min_decision_margin').value)
        self._max_age = float(self.get_parameter('max_data_age_sec').value)
        self._future_tolerance = float(
            self.get_parameter('future_tolerance_sec').value
        )
        self._tf_timeout = float(self.get_parameter('tf_lookup_timeout_sec').value)
        statistics_period = float(
            self.get_parameter('statistics_period_sec').value
        )
        self._validate_thresholds(statistics_period)

        detection_topic = str(self.get_parameter('detections_topic').value)
        observation_topic = str(self.get_parameter('observations_topic').value)
        if not detection_topic or not observation_topic:
            raise TagConfigError('input and output topic names must not be empty')

        self._tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._counters = Counter()

        detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            DockTagObservationArray, observation_topic, output_qos
        )
        self._subscription = self.create_subscription(
            AprilTagDetectionArray,
            detection_topic,
            self._on_detections,
            detection_qos,
        )
        self._statistics_timer = self.create_timer(
            statistics_period, self._log_statistics
        )
        self.get_logger().info(
            'ready: family=%s tags=%s input=%s output=%s'
            % (self._family, sorted(self._registry), detection_topic, observation_topic)
        )

    def _validate_thresholds(self, statistics_period):
        finite_values = {
            'min_decision_margin': self._min_margin,
            'max_data_age_sec': self._max_age,
            'future_tolerance_sec': self._future_tolerance,
            'tf_lookup_timeout_sec': self._tf_timeout,
            'statistics_period_sec': statistics_period,
        }
        for name, value in finite_values.items():
            if not math.isfinite(value):
                raise TagConfigError('%s must be finite' % name)
        if self._max_hamming < 0:
            raise TagConfigError('max_hamming must be non-negative')
        if self._min_margin < 0.0:
            raise TagConfigError('min_decision_margin must be non-negative')
        if self._max_age <= 0.0 or self._future_tolerance < 0.0:
            raise TagConfigError('data age thresholds are invalid')
        if self._tf_timeout < 0.0 or statistics_period <= 0.0:
            raise TagConfigError('timeout and statistics period are invalid')

    def _on_detections(self, message):
        output = DockTagObservationArray()
        output.header = message.header
        self._counters['frames'] += 1
        self._counters['raw_detections'] += len(message.detections)

        age = self._message_age(message)
        if age is None:
            self._publisher.publish(output)
            return

        duplicate_ids = duplicate_registered_ids(
            message.detections, self._registry.keys()
        )
        if duplicate_ids:
            self._counters['duplicate_id'] += len(duplicate_ids)
            self.get_logger().error(
                'duplicate registered tag IDs in one frame: %s'
                % sorted(duplicate_ids),
                throttle_duration_sec=5.0,
            )

        for detection in message.detections:
            tag_id = int(detection.id)
            if tag_id in duplicate_ids:
                continue
            spec = self._registry.get(tag_id)
            if spec is None:
                self._counters['unknown_id'] += 1
                self.get_logger().warning(
                    'ignoring unregistered tag ID %d' % tag_id,
                    throttle_duration_sec=5.0,
                )
                continue
            rejection = self._invalid_detection_reason(detection)
            if rejection:
                self._counters[rejection] += 1
                continue

            observation = self._make_observation(message, detection, spec, age)
            output.observations.append(observation)
            self._counters['published'] += 1

        self._publisher.publish(output)

    def _message_age(self, message):
        if not message.header.frame_id.strip():
            self._counters['empty_frame'] += 1
            return None
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds == 0:
            self._counters['zero_stamp'] += 1
            return None
        age_ns = self.get_clock().now().nanoseconds - stamp.nanoseconds
        age_sec = age_ns / 1e9
        if age_sec < -self._future_tolerance:
            self._counters['future_stamp'] += 1
            return None
        if age_sec > self._max_age:
            self._counters['stale_frame'] += 1
        return Duration(nanoseconds=max(0, age_ns))

    def _invalid_detection_reason(self, detection):
        if normalize_family(detection.family) != self._family:
            return 'wrong_family'
        if detection.hamming < 0 or detection.hamming > self._max_hamming:
            return 'hamming_rejected'
        quality = (detection.goodness, detection.decision_margin)
        if not all(math.isfinite(value) for value in quality):
            return 'invalid_quality'
        if detection.decision_margin < self._min_margin:
            return 'low_margin'
        points = [detection.centre, *detection.corners]
        if len(detection.corners) != 4 or not all(
            math.isfinite(point.x) and math.isfinite(point.y) for point in points
        ):
            return 'invalid_corners'
        if len(detection.homography) != 9 or not all(
            math.isfinite(value) for value in detection.homography
        ):
            return 'invalid_homography'
        return None

    def _make_observation(self, message, detection, spec, age):
        observation = DockTagObservation()
        observation.header = message.header
        observation.family = normalize_family(detection.family)
        observation.tag_id = spec.tag_id
        observation.tag_size_m = spec.size_m
        observation.hamming = int(detection.hamming)
        observation.goodness = float(detection.goodness)
        observation.decision_margin = float(detection.decision_margin)
        observation.centre = Point32(
            x=float(detection.centre.x), y=float(detection.centre.y), z=0.0
        )
        observation.corners = [
            Point32(x=float(corner.x), y=float(corner.y), z=0.0)
            for corner in detection.corners
        ]
        observation.homography = [float(value) for value in detection.homography]
        observation.tag_frame = spec.frame
        observation.data_age = age.to_msg()
        observation.data_fresh = age.nanoseconds <= self._max_age * 1e9

        try:
            transform = self._tf_buffer.lookup_transform(
                message.header.frame_id,
                spec.frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as error:
            self._counters['tf_unavailable'] += 1
            self.get_logger().warning(
                'TF unavailable for tag %d at image stamp: %s' % (spec.tag_id, error),
                throttle_duration_sec=5.0,
            )
            return observation

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self._counters['invalid_pose'] += 1
            return observation

        observation.pose.position.x = translation.x
        observation.pose.position.y = translation.y
        observation.pose.position.z = translation.z
        observation.pose.orientation = rotation
        observation.pose_valid = True
        return observation

    def _log_statistics(self):
        values = ' '.join(
            '%s=%d' % item for item in sorted(self._counters.items())
        )
        self.get_logger().info('adapter statistics: %s' % (values or 'no data'))


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = DockTagAdapter()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except TagConfigError as error:
        rclpy.logging.get_logger('dock_tag_adapter').fatal(str(error))
        raise SystemExit(2) from error
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
