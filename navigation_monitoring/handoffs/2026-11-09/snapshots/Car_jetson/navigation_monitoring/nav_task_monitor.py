#!/usr/bin/env python3
"""Low-overhead ROS 2 navigation task monitor with a thermal fail-safe."""

import argparse
import glob
import json
import os
import signal
import time
from datetime import datetime, timezone

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from multi_map_navigation_msgs.msg import RobotStatus, WaypointList
from multi_map_navigation_msgs.srv import ShutdownProcess
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from yunle_msgs.msg import VehicleStatus


def iso_now():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def read_text(path, default=""):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return handle.read().strip()
    except OSError:
        return default


class NavigationTaskMonitor(Node):
    PROCESS_MARKERS = {
        "process_manager": "/multi_map_navigation/process_manager",
        "navigation_manager": "/multi_map_navigation/navigation_manager",
        "re_localization": "re_localization",
        "nav2_init_pose": "nav2_init_pose",
        "liosam": "lio_sam_",
        "nav2": "component_container_isolated",
    }

    def __init__(self, output_path, enable_protection=False):
        super().__init__("navigation_task_monitor")
        self.output_path = output_path
        self.output = open(output_path, "a", encoding="utf-8", buffering=1)
        self.counts = {"imu": 0, "scan": 0, "cmd_vel": 0, "odom": 0, "tf": 0}
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
        self.process_state = {}
        self.nav2_seen = False
        self.liosam_seen = False
        self.terminal_since = None
        self.terminal_kind = None
        self.shutdown_requested = False
        self.protective_abort = False
        self.enable_protection = enable_protection
        self.thermal_critical_samples = 0
        self.memory_critical_samples = 0
        self.last_cpu = self.read_cpu_ticks()
        self.last_snapshot = time.monotonic()
        self.last_fsync = time.monotonic()

        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        normal_qos = QoSProfile(depth=20)

        self.create_subscription(Imu, "/imu", self.on_imu, sensor_qos)
        self.create_subscription(LaserScan, "/laser/scan_merge_laser", self.on_scan, sensor_qos)
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

    def on_cmd_vel(self, msg):
        self.mark_seen("cmd_vel")
        self.last_cmd = [round(float(msg.linear.x), 4), round(float(msg.angular.z), 4)]

    def on_odom(self, _msg):
        self.mark_seen("odom")

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
            if candidate:
                self.write_event("terminal_candidate", kind=candidate)
        elif candidate and now - self.terminal_since >= 15.0:
            self.write_event(
                "monitor_terminal",
                kind=candidate,
                task_id=self.task_id,
                fault=self.fault,
                protective_abort=self.protective_abort,
                vehicle=self.vehicle,
                goal_statuses=self.goal_statuses,
            )
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
                cpu_percent=self.cpu_percent(),
                load=[float(value) for value in read_text("/proc/loadavg").split()[:3]],
                memory_available_mb=mem_mb,
                temperatures_c=temps,
            )
            self.window_counts = dict(self.counts)
            self.window_started = now
        self.update_terminal_state(now)

    def close(self, reason):
        self.write_event("monitor_stopped", reason=reason)
        self.output.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--enable-protection", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    monitor = NavigationTaskMonitor(args.output, enable_protection=args.enable_protection)
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
