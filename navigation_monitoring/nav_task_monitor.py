#!/usr/bin/env python3
"""Low-overhead ROS 2 navigation task monitor with a thermal fail-safe."""

import argparse
import glob
import json
import math
import os
import signal
import time
from datetime import datetime, timezone

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from multi_map_navigation_msgs.msg import (
    EpochOdometry,
    LioCorrectionEvent,
    LioMapOdomCorrection,
    RobotStatus,
    WaypointList,
)
from multi_map_navigation_msgs.srv import ShutdownProcess
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from yunle_msgs.msg import VehicleStatus


LEGACY_FRONT_FILTER = {
    "raw_angle_min_deg": 132.0,
    "raw_angle_max_deg": 135.0,
    "range_min_m": 0.60,
    "range_max_m": 0.83,
    "laser_alpha_deg": 45.0,
    "laser_x_offset_m": 0.43,
    "laser_y_offset_m": 0.28,
    "transformed_x_min_m": -0.40,
    "transformed_x_max_m": -0.18,
    "transformed_y_min_m": 0.28,
    "transformed_y_max_m": 0.33,
}
FILTER_RETIREMENT_MIN_ZERO_HIT_SESSIONS = 3
FILTER_RETIREMENT_MIN_OBSERVATION_SECONDS = 3600.0
DEFAULT_FILTER_TRACKER_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "filter_retirement_tracker.json"
)


def iso_now():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def read_text(path, default=""):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return handle.read().strip()
    except OSError:
        return default


def legacy_front_filter_hits(scan):
    """Count points that would be removed by the first front-lidar filter."""
    if not scan.ranges or not math.isfinite(float(scan.angle_increment)):
        return 0
    increment = float(scan.angle_increment)
    if increment == 0.0:
        return 0

    angle_min = float(scan.angle_min)
    lower = math.radians(LEGACY_FRONT_FILTER["raw_angle_min_deg"])
    upper = math.radians(LEGACY_FRONT_FILTER["raw_angle_max_deg"])
    first = max(0, int(math.ceil((lower - angle_min) / increment)))
    last = min(len(scan.ranges) - 1, int(math.floor((upper - angle_min) / increment)))
    if first > last:
        return 0

    alpha = math.radians(LEGACY_FRONT_FILTER["laser_alpha_deg"])
    hits = 0
    for index in range(first, last + 1):
        distance = float(scan.ranges[index])
        if not math.isfinite(distance) or not (
            LEGACY_FRONT_FILTER["range_min_m"]
            <= distance
            <= LEGACY_FRONT_FILTER["range_max_m"]
        ):
            continue
        transformed_angle = angle_min + index * increment + alpha
        x = LEGACY_FRONT_FILTER["laser_x_offset_m"] + distance * math.cos(
            transformed_angle
        )
        y = LEGACY_FRONT_FILTER["laser_y_offset_m"] + distance * math.sin(
            transformed_angle
        )
        if (
            LEGACY_FRONT_FILTER["transformed_x_min_m"]
            <= x
            <= LEGACY_FRONT_FILTER["transformed_x_max_m"]
            and LEGACY_FRONT_FILTER["transformed_y_min_m"]
            <= y
            <= LEGACY_FRONT_FILTER["transformed_y_max_m"]
        ):
            hits += 1
    return hits


def update_filter_retirement_tracker(path, session):
    """Persist passive observations and return the updated retirement state."""
    state = {
        "schema_version": 1,
        "baseline_at": iso_now(),
        "policy": {
            "minimum_consecutive_zero_hit_sessions": (
                FILTER_RETIREMENT_MIN_ZERO_HIT_SESSIONS
            ),
            "minimum_cumulative_observation_seconds": (
                FILTER_RETIREMENT_MIN_OBSERVATION_SECONDS
            ),
            "action": "remind_only_never_auto_remove",
        },
        "consecutive_zero_hit_sessions": 0,
        "consecutive_zero_hit_observation_seconds": 0.0,
        "last_candidate_seen_at": None,
        "removal_reminder_due": False,
        "recent_sessions": [],
    }
    try:
        with open(path, "r", encoding="utf-8") as handle:
            loaded = json.load(handle)
        if isinstance(loaded, dict) and loaded.get("schema_version") == 1:
            state.update(loaded)
    except (OSError, ValueError, TypeError):
        pass

    observed_seconds = max(0.0, float(session["observation_seconds"]))
    candidate_points = max(0, int(session["candidate_points"]))
    candidate_frames = max(0, int(session["candidate_frames"]))
    if candidate_points:
        state["consecutive_zero_hit_sessions"] = 0
        state["consecutive_zero_hit_observation_seconds"] = 0.0
        state["last_candidate_seen_at"] = session["ended_at"]
    else:
        state["consecutive_zero_hit_sessions"] = int(
            state.get("consecutive_zero_hit_sessions", 0)
        ) + 1
        state["consecutive_zero_hit_observation_seconds"] = round(
            float(state.get("consecutive_zero_hit_observation_seconds", 0.0))
            + observed_seconds,
            3,
        )

    state["removal_reminder_due"] = bool(
        state["consecutive_zero_hit_sessions"]
        >= FILTER_RETIREMENT_MIN_ZERO_HIT_SESSIONS
        and state["consecutive_zero_hit_observation_seconds"]
        >= FILTER_RETIREMENT_MIN_OBSERVATION_SECONDS
    )
    state["updated_at"] = iso_now()
    recent = list(state.get("recent_sessions", []))[-19:]
    recent.append(
        {
            **session,
            "observation_seconds": round(observed_seconds, 3),
            "candidate_points": candidate_points,
            "candidate_frames": candidate_frames,
        }
    )
    state["recent_sessions"] = recent

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    temporary_path = f"{path}.tmp.{os.getpid()}"
    with open(temporary_path, "w", encoding="utf-8") as handle:
        json.dump(state, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary_path, path)
    return state


class NavigationTaskMonitor(Node):
    PROCESS_MARKERS = {
        "process_manager": "/multi_map_navigation/process_manager",
        "navigation_manager": "/multi_map_navigation/navigation_manager",
        "re_localization": "re_localization",
        "nav2_init_pose": "nav2_init_pose",
        "liosam": "lio_sam_",
        "nav2": "component_container_isolated",
    }

    def __init__(
        self,
        output_path,
        enable_protection=False,
        stop_on_terminal=False,
        filter_tracker_path=DEFAULT_FILTER_TRACKER_PATH,
    ):
        super().__init__("navigation_task_monitor")
        self.output_path = output_path
        self.output = open(output_path, "a", encoding="utf-8", buffering=1)
        self.counts = {
            "imu": 0,
            "scan": 0,
            "filtered_scan": 0,
            "front_scan": 0,
            "cmd_vel": 0,
            "odom": 0,
            "epoch_odom": 0,
            "correction_event": 0,
            "map_odom_correction": 0,
            "tf": 0,
        }
        self.window_counts = dict(self.counts)
        self.window_started = time.monotonic()
        self.last_seen = {}
        self.last_cmd = [0.0, 0.0]
        self.vehicle = {
            "speed_kmh": None,
            "steer_deg": None,
            "drive_mode": None,
            "is_autodrive": None,
        }
        self.robot_state = None
        self.robot_status = None
        self.task_seen = False
        self.task_id = None
        self.task_started_at = None
        self.goal_statuses = []
        self.fault = None
        self.last_lio_epoch = None
        self.last_lio_correction = None
        self.process_state = {}
        self.nav2_seen = False
        self.liosam_seen = False
        self.terminal_since = None
        self.terminal_kind = None
        self.terminal_reported = False
        self.shutdown_requested = False
        self.protective_abort = False
        self.enable_protection = enable_protection
        self.stop_on_terminal = stop_on_terminal
        self.thermal_critical_samples = 0
        self.memory_critical_samples = 0
        self.last_cpu = self.read_cpu_ticks()
        self.last_snapshot = time.monotonic()
        self.last_fsync = time.monotonic()
        self.filter_tracker_path = filter_tracker_path
        self.front_scan_first_seen = None
        self.front_scan_last_seen = None
        self.legacy_filter_candidate_frames = 0
        self.legacy_filter_candidate_points = 0

        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        normal_qos = QoSProfile(depth=20)

        self.create_subscription(Imu, "/imu", self.on_imu, sensor_qos)
        self.create_subscription(
            LaserScan, "/laser/scan_front", self.on_front_scan, sensor_qos
        )
        self.create_subscription(LaserScan, "/laser/scan_merge_laser", self.on_scan, sensor_qos)
        self.create_subscription(
            LaserScan,
            "/laser/scan_merge_laser_filtered",
            self.on_filtered_scan,
            sensor_qos,
        )
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, normal_qos)
        self.create_subscription(VehicleStatus, "/vehicle_status", self.on_vehicle, normal_qos)
        self.create_subscription(RobotStatus, "/robot_status", self.on_robot_status, normal_qos)
        self.create_subscription(String, "/robot_state", self.on_robot_state, normal_qos)
        self.create_subscription(String, "/process_manager/fault", self.on_fault, normal_qos)
        self.create_subscription(WaypointList, "/waypoint_list", self.on_waypoints, normal_qos)
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.on_goal_status,
            normal_qos,
        )
        self.create_subscription(
            Odometry,
            "/lio_sam/mapping/odometry",
            self.on_odom,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            "/lio_sam/mapping/odometry_incremental",
            self.on_odom,
            sensor_qos,
        )
        self.create_subscription(
            EpochOdometry,
            "/lio_sam/epoch_odometry",
            self.on_epoch_odom,
            sensor_qos,
        )
        self.create_subscription(
            LioCorrectionEvent,
            "/lio_sam/correction_event",
            self.on_correction_event,
            normal_qos,
        )
        self.create_subscription(
            LioMapOdomCorrection,
            "/lio_sam/map_odom_correction",
            self.on_map_odom_correction,
            normal_qos,
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, sensor_qos)

        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", normal_qos)
        self.fault_pub = self.create_publisher(String, "/process_manager/fault", normal_qos)
        self.shutdown_client = self.create_client(
            ShutdownProcess, "/process_manager/shutdown_process"
        )
        self.create_timer(1.0, self.on_timer)
        self.write_event(
            "monitor_started",
            boot_id=read_text("/proc/sys/kernel/random/boot_id"),
            pid=os.getpid(),
            protection_enabled=self.enable_protection,
            stop_on_terminal=self.stop_on_terminal,
        )

    def write_event(self, event, **fields):
        record = {"time": iso_now(), "event": event, **fields}
        self.output.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
        if event != "snapshot" or time.monotonic() - self.last_fsync >= 5.0:
            self.output.flush()
            os.fsync(self.output.fileno())
            self.last_fsync = time.monotonic()

    def mark_seen(self, name):
        self.counts[name] += 1
        self.last_seen[name] = time.monotonic()

    def on_imu(self, _msg):
        self.mark_seen("imu")

    def on_scan(self, _msg):
        self.mark_seen("scan")

    def on_front_scan(self, msg):
        now = time.monotonic()
        self.mark_seen("front_scan")
        if self.front_scan_first_seen is None:
            self.front_scan_first_seen = now
        self.front_scan_last_seen = now
        hits = legacy_front_filter_hits(msg)
        if hits:
            self.legacy_filter_candidate_frames += 1
            self.legacy_filter_candidate_points += hits

    def on_filtered_scan(self, _msg):
        self.mark_seen("filtered_scan")

    def on_cmd_vel(self, msg):
        self.mark_seen("cmd_vel")
        self.last_cmd = [round(float(msg.linear.x), 4), round(float(msg.angular.z), 4)]

    def on_odom(self, _msg):
        self.mark_seen("odom")

    def on_epoch_odom(self, msg):
        self.mark_seen("epoch_odom")
        self.last_lio_epoch = {
            "instance_id": msg.instance_id,
            "generation": int(msg.generation),
            "correction_epoch": int(msg.correction_epoch),
        }

    def on_correction_event(self, msg):
        self.mark_seen("correction_event")
        source_score = float(msg.source_score)
        self.last_lio_correction = {
            "instance_id": msg.instance_id,
            "generation": int(msg.generation),
            "correction_epoch": int(msg.correction_epoch),
            "source": int(msg.source),
            "translation_residual_m": round(float(msg.translation_residual), 6),
            "yaw_residual_rad": round(float(msg.yaw_residual), 6),
            "source_score": (
                round(source_score, 6) if math.isfinite(source_score) else None
            ),
            "arrival_delay_sec": round(float(msg.arrival_delay_sec), 6),
            "degenerate": bool(msg.degenerate),
            "accepted": bool(msg.accepted),
            "reason": msg.reason,
        }

    def on_map_odom_correction(self, msg):
        self.mark_seen("map_odom_correction")
        self.last_map_odom_correction = {
            "instance_id": msg.instance_id,
            "generation": int(msg.generation),
            "correction_epoch": int(msg.correction_epoch),
            "translation_residual_m": round(
                float(msg.translation_residual), 6),
            "yaw_residual_rad": round(float(msg.yaw_residual), 6),
            "state": int(msg.state),
            "accepted": bool(msg.accepted),
            "reason": msg.reason,
        }

    def on_tf(self, msg):
        self.mark_seen("tf")
        for transform in msg.transforms:
            key = f"{transform.header.frame_id}->{transform.child_frame_id}"
            if key in ("map->odom", "odom->base_link"):
                self.last_seen[key] = time.monotonic()

    def on_vehicle(self, msg):
        self.last_seen["vehicle_status"] = time.monotonic()
        self.vehicle = {
            "speed_kmh": round(float(msg.cur_speed), 3),
            "steer_deg": round(float(msg.cur_steer), 3),
            "drive_mode": int(msg.drive_mode),
            "is_autodrive": bool(msg.is_autodrive),
        }

    def start_task(self, source):
        if self.task_seen:
            return
        self.task_seen = True
        self.task_started_at = iso_now()
        self.write_event("task_started", source=source, task_id=self.task_id)

    def on_robot_status(self, msg):
        self.last_seen["robot_status"] = time.monotonic()
        new_status = int(msg.task_status)
        if new_status != self.robot_status:
            self.robot_status = new_status
            self.write_event("robot_status_changed", task_status=new_status)
        if new_status == 1:
            self.start_task("robot_status")

    def on_robot_state(self, msg):
        state = msg.data.strip().lower()
        if state != self.robot_state:
            self.robot_state = state
            self.write_event("robot_state_changed", state=state)
        if state == "running":
            self.start_task("robot_state")

    def on_fault(self, msg):
        self.fault = msg.data
        self.write_event("process_fault", reason=msg.data)

    def on_waypoints(self, msg):
        self.task_id = msg.task_id
        self.write_event(
            "waypoint_task",
            task_id=msg.task_id,
            total_waypoints=int(msg.total_waypoints),
            start_map=msg.start_map_name,
            waypoint_ids=[int(item.id) for item in msg.waypoints],
        )
        self.start_task("waypoint_list")

    def on_goal_status(self, msg):
        statuses = sorted({int(item.status) for item in msg.status_list})
        if statuses != self.goal_statuses:
            self.goal_statuses = statuses
            self.write_event("goal_status_changed", statuses=statuses)

    @staticmethod
    def read_cpu_ticks():
        fields = read_text("/proc/stat").splitlines()[0].split()[1:]
        values = [int(value) for value in fields]
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return sum(values), idle

    def cpu_percent(self):
        current = self.read_cpu_ticks()
        total_delta = current[0] - self.last_cpu[0]
        idle_delta = current[1] - self.last_cpu[1]
        self.last_cpu = current
        if total_delta <= 0:
            return 0.0
        return round(100.0 * (total_delta - idle_delta) / total_delta, 1)

    @staticmethod
    def memory_available_mb():
        for line in read_text("/proc/meminfo").splitlines():
            if line.startswith("MemAvailable:"):
                return round(int(line.split()[1]) / 1024.0, 1)
        return None

    @staticmethod
    def temperatures():
        result = {}
        for zone in glob.glob("/sys/class/thermal/thermal_zone*"):
            kind = read_text(os.path.join(zone, "type"), os.path.basename(zone))
            raw = read_text(os.path.join(zone, "temp"))
            try:
                result[kind] = round(float(raw) / 1000.0, 1)
            except ValueError:
                continue
        return result

    @classmethod
    def scan_processes(cls):
        found = {name: [] for name in cls.PROCESS_MARKERS}
        for path in glob.glob("/proc/[0-9]*/cmdline"):
            try:
                pid = int(path.split("/")[2])
                command = read_text(path).replace("\x00", " ")
            except (OSError, ValueError):
                continue
            for name, marker in cls.PROCESS_MARKERS.items():
                if marker in command:
                    found[name].append(pid)
        return found

    def trigger_protective_abort(self, reason):
        if self.protective_abort:
            return
        self.protective_abort = True
        self.write_event("protective_abort", reason=reason)
        stop = Twist()
        for _ in range(3):
            self.stop_pub.publish(stop)
        fault = String()
        fault.data = f"navigation monitor protective abort: {reason}"
        for _ in range(3):
            self.fault_pub.publish(fault)
        if self.shutdown_client.service_is_ready():
            for name in ("navigation2", "liosam", "nav2_init_pose", "re_localization"):
                request = ShutdownProcess.Request()
                request.process_name = name
                self.shutdown_client.call_async(request)

    def update_terminal_state(self, now):
        if not self.task_seen:
            return
        subprocesses_down = not any(
            self.process_state.get(name)
            for name in ("re_localization", "nav2_init_pose", "liosam", "nav2")
        )
        managers_alive = bool(self.process_state.get("process_manager")) and bool(
            self.process_state.get("navigation_manager")
        )
        candidate = None
        if not managers_alive:
            candidate = "manager_lost"
        elif self.robot_state == "fault" and subprocesses_down:
            candidate = "fault"
        elif self.robot_state == "idle" and subprocesses_down:
            candidate = "idle"

        if candidate != self.terminal_kind:
            self.terminal_kind = candidate
            self.terminal_since = now if candidate else None
            self.terminal_reported = False
            if candidate:
                self.write_event("terminal_candidate", kind=candidate)
        elif (
            candidate
            and not self.terminal_reported
            and now - self.terminal_since >= 15.0
        ):
            self.write_event(
                "monitor_terminal",
                kind=candidate,
                task_id=self.task_id,
                fault=self.fault,
                protective_abort=self.protective_abort,
                vehicle=self.vehicle,
                goal_statuses=self.goal_statuses,
            )
            self.terminal_reported = True
            if self.stop_on_terminal:
                self.shutdown_requested = True

    def on_timer(self):
        now = time.monotonic()
        process_state = self.scan_processes()
        if process_state != self.process_state:
            self.process_state = process_state
            self.nav2_seen = self.nav2_seen or bool(process_state["nav2"])
            self.liosam_seen = self.liosam_seen or bool(process_state["liosam"])
            self.write_event("processes_changed", processes=process_state)

        temps = self.temperatures()
        max_temp = max(temps.values()) if temps else None
        mem_mb = self.memory_available_mb()
        self.thermal_critical_samples = (
            self.thermal_critical_samples + 1 if max_temp is not None and max_temp >= 88.0 else 0
        )
        self.memory_critical_samples = (
            self.memory_critical_samples + 1 if mem_mb is not None and mem_mb < 500.0 else 0
        )
        if self.enable_protection and self.task_seen and self.thermal_critical_samples >= 2:
            self.trigger_protective_abort(f"temperature {max_temp:.1f}C")
        if self.enable_protection and self.task_seen and self.memory_critical_samples >= 3:
            self.trigger_protective_abort(f"memory available {mem_mb:.1f}MiB")

        if now - self.window_started >= 5.0:
            elapsed = now - self.window_started
            rates = {
                name: round((count - self.window_counts[name]) / elapsed, 2)
                for name, count in self.counts.items()
            }
            freshness = {
                name: (round(now - seen, 2) if seen else None)
                for name, seen in self.last_seen.items()
            }
            self.write_event(
                "snapshot",
                task_seen=self.task_seen,
                task_id=self.task_id,
                robot_state=self.robot_state,
                robot_status=self.robot_status,
                processes=self.process_state,
                rates_hz=rates,
                freshness_sec=freshness,
                cmd_vel=self.last_cmd,
                vehicle=self.vehicle,
                goal_statuses=self.goal_statuses,
                lio_epoch=self.last_lio_epoch,
                lio_correction=self.last_lio_correction,
                cpu_percent=self.cpu_percent(),
                load=[float(value) for value in read_text("/proc/loadavg").split()[:3]],
                memory_available_mb=mem_mb,
                temperatures_c=temps,
            )
            self.window_counts = dict(self.counts)
            self.window_started = now
        self.update_terminal_state(now)

    def close(self, reason):
        observation_seconds = 0.0
        if self.front_scan_first_seen is not None and self.front_scan_last_seen is not None:
            observation_seconds = max(
                0.0, self.front_scan_last_seen - self.front_scan_first_seen
            )
        filter_session = {
            "ended_at": iso_now(),
            "monitor_output": os.path.abspath(self.output_path),
            "task_id": self.task_id,
            "front_scan_frames": self.counts["front_scan"],
            "observation_seconds": observation_seconds,
            "candidate_frames": self.legacy_filter_candidate_frames,
            "candidate_points": self.legacy_filter_candidate_points,
        }
        if self.counts["front_scan"] > 0:
            try:
                tracker = update_filter_retirement_tracker(
                    self.filter_tracker_path, filter_session
                )
                self.write_event(
                    "legacy_filter_observation",
                    session=filter_session,
                    tracker_path=os.path.abspath(self.filter_tracker_path),
                    consecutive_zero_hit_sessions=tracker[
                        "consecutive_zero_hit_sessions"
                    ],
                    consecutive_zero_hit_observation_seconds=tracker[
                        "consecutive_zero_hit_observation_seconds"
                    ],
                    removal_reminder_due=tracker["removal_reminder_due"],
                )
                if tracker["removal_reminder_due"]:
                    self.write_event(
                        "legacy_filter_removal_reminder",
                        message=(
                            "第一组前雷达滤波已连续多轮零命中；请复核后人工删除，"
                            "监控程序不会自动修改配置"
                        ),
                        tracker_path=os.path.abspath(self.filter_tracker_path),
                    )
            except (OSError, ValueError, TypeError, KeyError) as error:
                self.write_event(
                    "legacy_filter_tracker_error",
                    tracker_path=os.path.abspath(self.filter_tracker_path),
                    error=str(error),
                )
        self.write_event("monitor_stopped", reason=reason)
        self.output.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--enable-protection", action="store_true")
    parser.add_argument("--stop-on-terminal", action="store_true")
    parser.add_argument("--filter-tracker", default=DEFAULT_FILTER_TRACKER_PATH)
    args = parser.parse_args()
    rclpy.init()
    monitor = NavigationTaskMonitor(
        args.output,
        enable_protection=args.enable_protection,
        stop_on_terminal=args.stop_on_terminal,
        filter_tracker_path=args.filter_tracker,
    )
    stop_reason = "rclpy_shutdown"

    def handle_signal(signum, _frame):
        nonlocal stop_reason
        stop_reason = f"signal_{signum}"
        monitor.shutdown_requested = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    executor = SingleThreadedExecutor()
    try:
        executor.add_node(monitor)
        while rclpy.ok() and not monitor.shutdown_requested:
            executor.spin_once(timeout_sec=0.5)
    except KeyboardInterrupt:
        stop_reason = "keyboard_interrupt"
    finally:
        executor.remove_node(monitor)
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        monitor.close(stop_reason)
        monitor.destroy_node()


if __name__ == "__main__":
    main()
