"""Read-only measurements for the indoor, stationary navigation startup check.

The caller owns the executor and JSONL files. Add ``IndoorStartupObserver`` to
an executor, call ``set_phase(name)`` at each startup boundary, and serialize
``snapshot()`` once per second and once at shutdown. No incoming message is
modified or retained, and this module never publishes or sends an action goal.
All rates use monotonic arrival times; header and TF ages use the node clock.
"""

import math
import threading
import time
from datetime import datetime, timezone

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def _finite(value):
    value = float(value)
    return value if math.isfinite(value) else None


def _norm(values):
    return _finite(math.hypot(*(float(value) for value in values)))


def _intervals():
    return {
        "count": 0, "sum_sec": 0.0, "min_sec": None, "max_sec": None,
        "nonpositive_count": 0, "gap_count": 0, "last_gap_sec": None,
    }


def _add_interval(stats, seconds, gap_sec):
    if seconds is None:
        return
    stats["count"] += 1
    stats["sum_sec"] += seconds
    stats["min_sec"] = seconds if stats["min_sec"] is None else min(stats["min_sec"], seconds)
    stats["max_sec"] = seconds if stats["max_sec"] is None else max(stats["max_sec"], seconds)
    stats["nonpositive_count"] += int(seconds <= 0.0)
    if seconds > gap_sec:
        stats["gap_count"] += 1
        stats["last_gap_sec"] = seconds


class _TopicStats:
    def __init__(self):
        self.count = 0
        self.first_arrival = None
        self.last_arrival = None
        self.last_header_ns = None
        self.arrival_intervals = _intervals()
        self.header_intervals = _intervals()
        self.counters = {}
        self.latest = {}

    def observe(self, now, header_ns, arrival_delta, header_delta, gap_sec):
        if self.first_arrival is None:
            self.first_arrival = now
        self.count += 1
        self.last_arrival = now
        self.last_header_ns = header_ns
        _add_interval(self.arrival_intervals, arrival_delta, gap_sec)
        _add_interval(self.header_intervals, header_delta, gap_sec)

    def increment(self, name, amount=1):
        self.counters[name] = self.counters.get(name, 0) + amount

    def snapshot(self, now, ros_now_ns):
        span = None if self.count < 2 else self.last_arrival - self.first_arrival
        result = {
            "count": self.count,
            "arrival_hz": (self.count - 1) / span if span and span > 0.0 else None,
            "last_arrival_age_sec": None if self.last_arrival is None else now - self.last_arrival,
            "last_header_stamp_ns": self.last_header_ns,
            "last_header_age_sec": None if self.last_header_ns is None else (ros_now_ns - self.last_header_ns) / 1e9,
            "counters": dict(self.counters),
            "latest": dict(self.latest),
        }
        for name, intervals in (("arrival_intervals", self.arrival_intervals),
                                ("header_intervals", self.header_intervals)):
            values = dict(intervals)
            values["mean_sec"] = values["sum_sec"] / values["count"] if values["count"] else None
            result[name] = values
        return result


class IndoorStartupObserver(Node):
    """Collect bounded aggregate statistics without controlling the system.

    ``snapshot()`` returns cumulative ``topics``, per-phase ``phases``, and
    current ``tf`` checks. IMU gap counters refer to intervals strictly greater
    than ``imu_gap_sec``; an interval crossing a phase boundary is attributed to
    the phase in which its second message arrives. ``expected_static`` TF links
    are checked for existence/finite values, while dynamic links also require a
    recent timestamp. Static transform timestamps are reported without treating
    their old timestamps as stale.
    """

    ACTION_STATUS_TOPICS = (
        "/navigate_to_pose/_action/status",
        "/navigate_through_poses/_action/status",
        "/follow_waypoints/_action/status",
        "/follow_path/_action/status",
        "/spin/_action/status",
        "/backup/_action/status",
        "/drive_on_heading/_action/status",
    )

    def __init__(self, *, tf_max_age_sec=2.0, imu_gap_sec=0.2,
                 static_tf_links=(("map", "odom"),), node_name="indoor_startup_observer"):
        super().__init__(node_name, enable_rosout=False, start_parameter_services=False)
        self._lock = threading.RLock()
        self._started = time.monotonic()
        self._phase = "sensor_baseline"
        self._phase_since = self._started
        self._phase_times = {self._phase: {"first_start_elapsed_sec": 0.0, "completed_duration_sec": 0.0, "entries": 1}}
        self._totals = {}
        self._phases = {self._phase: {}}
        self._observer_subscriptions = []
        self._topic_subscriptions = {}
        self._previous_snapshot = {}
        self._imu_gap_sec = float(imu_gap_sec)
        self._tf_max_age_sec = float(tf_max_age_sec)
        self._static_tf_links = set(tuple(link) for link in static_tf_links)
        self._ready = None
        self._ready_ever_true = False
        self._ready_transitions = 0
        self._ready_falls = 0
        self._last_ready_change = None
        sensor_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        cloud_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        specs = [
            (Imu, "/imu", sensor_qos, "imu"),
            (PointCloud2, "/rslidar_Pointcloud2", cloud_qos, "cloud"),
            (LaserScan, "/laser/scan_merge_laser", sensor_qos, "scan"),
            (Odometry, "/lio_sam/mapping/odometry", sensor_qos, "odom"),
            (Odometry, "/lio_sam/mapping/odometry_incremental", sensor_qos, "odom"),
            (Odometry, "/odom", sensor_qos, "odom"),
            (Odometry, "/odom_incremental", sensor_qos, "odom"),
            (Bool, "/lio_sam/imu_preintegration_ready", reliable_qos, "ready"),
            (OccupancyGrid, "/map", map_qos, "map"),
            (OccupancyGrid, "/global_costmap/costmap", reliable_qos, "map"),
            (OccupancyGrid, "/local_costmap/costmap", reliable_qos, "map"),
        ]
        specs.extend((Twist, topic, reliable_qos, "velocity") for topic in (
            "/indoor_startup/cmd_vel", "/indoor_startup/cmd_vel_nav",
            "/indoor_startup/cmd_vel_smoothed"))
        specs.extend((GoalStatusArray, topic, map_qos, "action_status")
                     for topic in self.ACTION_STATUS_TOPICS)
        for message_type, topic, qos, kind in specs:
            self._totals[topic] = _TopicStats()
            callback = lambda msg, topic=topic, kind=kind: self._observe(topic, kind, msg)
            subscription = self.create_subscription(message_type, topic, callback, qos)
            self._observer_subscriptions.append(subscription)
            self._topic_subscriptions[topic] = subscription
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    def set_phase(self, name):
        """Mark a startup boundary; repeated calls for the same name do nothing."""
        if not isinstance(name, str) or not name:
            raise ValueError("phase name must be a nonempty string")
        with self._lock:
            if name == self._phase:
                return
            now = time.monotonic()
            self._phase_times[self._phase]["completed_duration_sec"] += now - self._phase_since
            self._phase = name
            self._phase_since = now
            metadata = self._phase_times.setdefault(name, {
                "first_start_elapsed_sec": now - self._started,
                "completed_duration_sec": 0.0, "entries": 0,
            })
            metadata["entries"] += 1
            self._phases.setdefault(name, {})

    def _observe(self, topic, kind, msg):
        now = time.monotonic()
        header = getattr(msg, "header", None)
        stamp = None if header is None else header.stamp
        header_ns = None if stamp is None else stamp.sec * 1_000_000_000 + stamp.nanosec
        with self._lock:
            total = self._totals[topic]
            arrival_delta = None if total.last_arrival is None else now - total.last_arrival
            header_delta = None if header_ns is None or total.last_header_ns is None else (header_ns - total.last_header_ns) / 1e9
            phase_stats = self._phases[self._phase].setdefault(topic, _TopicStats())
            for bucket in (total, phase_stats):
                bucket.observe(now, header_ns, arrival_delta, header_delta, self._imu_gap_sec)
            if kind == "imu":
                q = msg.orientation
                orientation = (q.x, q.y, q.z, q.w)
                a = msg.linear_acceleration
                acceleration = (a.x, a.y, a.z)
                w = msg.angular_velocity
                angular = (w.x, w.y, w.z)
                q_norm = _norm(orientation)
                for bucket in (total, phase_stats):
                    bucket.increment("quaternion_nonfinite_count", int(not all(math.isfinite(v) for v in orientation)))
                    bucket.increment("quaternion_norm_nonfinite_count", int(q_norm is None))
                    bucket.increment("quaternion_zero_norm_count", int(q_norm is not None and q_norm < 1e-6))
                    bucket.increment("quaternion_norm_outside_0_9_to_1_1_count", int(q_norm is not None and abs(q_norm - 1.0) > 0.1))
                    bucket.increment("acceleration_nonfinite_count", int(not all(math.isfinite(v) for v in acceleration)))
                    bucket.increment("angular_velocity_nonfinite_count", int(not all(math.isfinite(v) for v in angular)))
                    bucket.latest = {"quaternion_norm": q_norm,
                                     "acceleration_norm": _norm(acceleration),
                                     "angular_velocity_norm": _norm(angular),
                                     "orientation_declared_unavailable": bool(msg.orientation_covariance[0] < 0.0),
                                     "frame_id": msg.header.frame_id}
            elif kind == "ready":
                value = bool(msg.data)
                if value != self._ready:
                    self._ready_transitions += int(self._ready is not None)
                    self._ready_falls += int(self._ready is True and not value)
                    self._last_ready_change = now
                self._ready = value
                self._ready_ever_true = self._ready_ever_true or value
                for bucket in (total, phase_stats):
                    bucket.increment("true_count" if value else "false_count")
                    bucket.latest = {"ready": value}
            elif kind == "velocity":
                values = (msg.linear.x, msg.linear.y, msg.linear.z,
                          msg.angular.x, msg.angular.y, msg.angular.z)
                nonfinite = not all(math.isfinite(v) for v in values)
                nonzero = any(math.isfinite(v) and abs(v) > 1e-6 for v in values)
                for bucket in (total, phase_stats):
                    bucket.increment("nonzero_count", int(nonzero))
                    bucket.increment("nonfinite_count", int(nonfinite))
                    bucket.latest = {"linear_x": _finite(msg.linear.x), "angular_z": _finite(msg.angular.z)}
            elif kind == "action_status":
                active = sum(status.status in (1, 2, 3) for status in msg.status_list)
                for bucket in (total, phase_stats):
                    bucket.increment("nonempty_messages", int(bool(msg.status_list)))
                    bucket.increment("active_messages", int(active > 0))
                    bucket.increment("active_entries", active)
                    bucket.latest = {"status_entries": len(msg.status_list), "active_entries": active}
            elif kind == "odom":
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
                for bucket in (total, phase_stats):
                    bucket.increment("pose_nonfinite_count", int(not all(math.isfinite(v) for v in values)))
                    bucket.latest = {"x": _finite(p.x), "y": _finite(p.y), "z": _finite(p.z),
                                     "frame_id": msg.header.frame_id, "child_frame_id": msg.child_frame_id}
            elif kind == "map":
                for bucket in (total, phase_stats):
                    bucket.latest = {"width": msg.info.width, "height": msg.info.height,
                                     "resolution": _finite(msg.info.resolution), "frame_id": msg.header.frame_id}
            elif kind == "cloud":
                for bucket in (total, phase_stats):
                    bucket.latest = {"points": msg.width * msg.height, "frame_id": msg.header.frame_id}
            elif kind == "scan":
                for bucket in (total, phase_stats):
                    bucket.latest = {"ranges": len(msg.ranges), "frame_id": msg.header.frame_id}

    def _tf_snapshot(self, ros_now_ns):
        checks = {}
        for parent, child in (("map", "odom"), ("odom", "base_link"), ("map", "base_link")):
            expected_static = (parent, child) in self._static_tf_links
            entry = {"available": False, "expected_static": expected_static, "fresh": False}
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.0))
                stamp = transform.header.stamp
                age = (ros_now_ns - (stamp.sec * 1_000_000_000 + stamp.nanosec)) / 1e9
                p, q = transform.transform.translation, transform.transform.rotation
                finite = all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                entry.update({"available": True, "header_age_sec": age, "finite": finite,
                              "quaternion_norm": _norm((q.x, q.y, q.z, q.w)),
                              "fresh": finite and (expected_static or -0.5 <= age <= self._tf_max_age_sec)})
            except TransformException as exc:
                entry["error"] = str(exc)
            checks[parent + "->" + child] = entry
        return checks

    def snapshot(self):
        """Return JSON-safe aggregates; never retains ROS messages or raw data."""
        now = time.monotonic()
        ros_now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            topics = {}
            for topic, stats in self._totals.items():
                item = stats.snapshot(now, ros_now_ns)
                previous = self._previous_snapshot.get(topic)
                item["interval_arrival_hz"] = None if previous is None else (stats.count - previous[1]) / max(1e-9, now - previous[0])
                self._previous_snapshot[topic] = (now, stats.count)
                try:
                    item["publisher_count"] = self.count_publishers(topic)
                except Exception as exc:
                    item["publisher_count_error"] = str(exc)
                topics[topic] = item
            phases = {}
            for phase, statistics in self._phases.items():
                metadata = dict(self._phase_times[phase])
                metadata["duration_sec"] = metadata.pop("completed_duration_sec")
                if phase == self._phase:
                    metadata["duration_sec"] += now - self._phase_since
                phases[phase] = {**metadata, "topics": {
                    topic: stats.snapshot(now, ros_now_ns) for topic, stats in statistics.items()
                }}
            result = {
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "elapsed_sec": now - self._started, "phase": self._phase,
                "ros_time_ns": ros_now_ns,
                "thresholds": {"imu_gap_sec": self._imu_gap_sec, "tf_max_age_sec": self._tf_max_age_sec},
                "topics": topics, "phases": phases,
                "imu_preintegration": {
                    "ready": self._ready, "ever_ready": self._ready_ever_true,
                    "transitions": self._ready_transitions, "ready_to_not_ready_count": self._ready_falls,
                    "last_transition_age_sec": None if self._last_ready_change is None else now - self._last_ready_change,
                },
            }
        result["tf"] = self._tf_snapshot(ros_now_ns)
        return result
