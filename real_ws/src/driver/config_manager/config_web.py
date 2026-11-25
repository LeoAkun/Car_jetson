#!/usr/bin/env python3
"""Local web UI for building the workspace and editing driver configuration."""

from __future__ import annotations

import json
import math
import os
import re
import shlex
import signal
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parents[3]


def rel(path: str) -> Path:
    return ROOT / path


FIELDS: list[dict[str, Any]] = [
    {
        "id": "car_control",
        "title": "car_control 底盘控制",
        "description": "底盘 CAN 通信和 /cmd_vel 到 /ecu 转换。",
        "items": [
            {"key": "canbus_type", "label": "CAN 类型", "type": "int", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "main_can_id", "label": "USB2CAN 主通道", "type": "int", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "debug_can_id", "label": "USB2CAN 调试通道", "type": "int", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "can_eth_card", "label": "以太网 CAN 网卡", "type": "str", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "can1_remote_ip", "label": "CAN1 远端 IP", "type": "str", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "can1_remote_port", "label": "CAN1 远端端口", "type": "int", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "can2_remote_ip", "label": "CAN2 远端 IP", "type": "str", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "can2_remote_port", "label": "CAN2 远端端口", "type": "int", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "local_ip", "label": "本机绑定 IP", "type": "str", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "local_port", "label": "本机监听端口", "type": "int", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "debug_mode", "label": "CAN 调试模式", "type": "bool", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "show_sending_msg", "label": "打印发送 CAN 帧", "type": "bool", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "car_type", "label": "车型协议", "type": "str", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "vehicle_weight", "label": "车辆重量", "type": "float", "file": "src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py", "writer": "py_dict"},
            {"key": "L", "label": "轴距 L", "type": "float", "file": "src/driver/car_control/cmdvel_2_ecu/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "LOW_SPEED_THRESH", "label": "低速补偿阈值", "type": "float", "file": "src/driver/car_control/cmdvel_2_ecu/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "STOP_THRESH", "label": "停车阈值", "type": "float", "file": "src/driver/car_control/cmdvel_2_ecu/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "OMEGA_DEADZONE", "label": "角速度死区", "type": "float", "file": "src/driver/car_control/cmdvel_2_ecu/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "MAX_STEER_DEG", "label": "最大转角", "type": "float", "file": "src/driver/car_control/cmdvel_2_ecu/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "OMEGA_FULL_STEER", "label": "原地转向灵敏度", "type": "float", "file": "src/driver/car_control/cmdvel_2_ecu/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "rear_wheel_flag", "label": "双阿克曼/后轮转向", "type": "bool", "file": "src/driver/car_control/cmdvel_2_ecu/config/config.yaml", "writer": "yaml_scalar"},
        ],
    },
    {
        "id": "imu",
        "title": "imu IMU",
        "description": "FDILink IMU 串口、话题和坐标系。",
        "items": [
            {"key": "serial_port_", "label": "IMU 串口", "type": "str", "file": "src/driver/imu/fdilink_ahrs_ROS2/launch/ahrs_driver.launch.py", "writer": "py_dict"},
            {"key": "serial_baud_", "label": "IMU 波特率", "type": "int", "file": "src/driver/imu/fdilink_ahrs_ROS2/launch/ahrs_driver.launch.py", "writer": "py_dict"},
            {"key": "imu_topic", "label": "IMU 话题", "type": "str", "file": "src/driver/imu/fdilink_ahrs_ROS2/launch/ahrs_driver.launch.py", "writer": "py_dict"},
            {"key": "imu_frame_id_", "label": "IMU frame_id", "type": "str", "file": "src/driver/imu/fdilink_ahrs_ROS2/launch/ahrs_driver.launch.py", "writer": "py_dict"},
            {"key": "device_type_", "label": "设备类型", "type": "int", "file": "src/driver/imu/fdilink_ahrs_ROS2/launch/ahrs_driver.launch.py", "writer": "py_dict"},
            {"key": "if_debug_", "label": "调试输出", "type": "bool", "file": "src/driver/imu/fdilink_ahrs_ROS2/launch/ahrs_driver.launch.py", "writer": "py_dict"},
        ],
    },
    {
        "id": "rslidar",
        "title": "rslidar 速腾雷达",
        "description": "RoboSense 雷达型号、UDP 端口、输出话题和点云过滤。",
        "items": [
            {"key": "msg_source", "label": "数据源", "type": "int", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "send_packet_ros", "label": "发布原始 packet", "type": "bool", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "send_point_cloud_ros", "label": "发布点云", "type": "bool", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "lidar_type", "label": "雷达型号", "type": "str", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "msop_port", "label": "MSOP 端口", "type": "int", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "difop_port", "label": "DIFOP 端口", "type": "int", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "imu_port", "label": "雷达 IMU 端口", "type": "int", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "min_distance", "label": "最小距离", "type": "float", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "max_distance", "label": "最大距离", "type": "float", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "use_lidar_clock", "label": "使用雷达时钟", "type": "bool", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "dense_points", "label": "丢弃 NaN 点", "type": "bool", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "start_angle", "label": "起始角", "type": "float", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "end_angle", "label": "结束角", "type": "float", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "pcap_path", "label": "PCAP 路径", "type": "str", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "ros_frame_id", "label": "frame_id", "type": "str", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
            {"key": "ros_send_point_cloud_topic", "label": "点云输出话题", "type": "str", "file": "src/driver/rslidar/rslidar_sdk/config/config.yaml", "writer": "yaml_scalar"},
        ],
    },
    {
        "id": "livox",
        "title": "livox Mid360 雷达",
        "description": "Livox Mid360 单独启动和点云数据查看；参数仍使用原始 JSON 配置文件。",
        "items": [],
    },
    {
        "id": "scan_laser",
        "title": "scan_laser 单线雷达",
        "description": "Lakibeam 前后雷达监听参数和融合外参。",
        "items": [
            {"key": "frame_id", "label": "Lakibeam frame_id", "type": "str", "file": "src/driver/scan_laser/Lakibeam_ROS2_Driver-main/launch/lakibeam1_scan_dual_lidar.launch.py", "writer": "launch_arg"},
            {"key": "output_topic0", "label": "前雷达话题", "type": "str", "file": "src/driver/scan_laser/Lakibeam_ROS2_Driver-main/launch/lakibeam1_scan_dual_lidar.launch.py", "writer": "launch_arg"},
            {"key": "output_topic1", "label": "后雷达话题", "type": "str", "file": "src/driver/scan_laser/Lakibeam_ROS2_Driver-main/launch/lakibeam1_scan_dual_lidar.launch.py", "writer": "launch_arg"},
            {"key": "hostip", "label": "本机监听 IP", "type": "str", "file": "src/driver/scan_laser/Lakibeam_ROS2_Driver-main/launch/lakibeam1_scan_dual_lidar.launch.py", "writer": "launch_arg"},
            {"key": "port_front", "label": "前雷达 UDP 端口", "type": "str", "file": "src/driver/scan_laser/Lakibeam_ROS2_Driver-main/launch/lakibeam1_scan_dual_lidar.launch.py", "writer": "launch_arg"},
            {"key": "port_back", "label": "后雷达 UDP 端口", "type": "str", "file": "src/driver/scan_laser/Lakibeam_ROS2_Driver-main/launch/lakibeam1_scan_dual_lidar.launch.py", "writer": "launch_arg"},
            {"key": "inverted", "label": "倒序扫描点", "type": "bool", "file": "src/driver/scan_laser/Lakibeam_ROS2_Driver-main/launch/lakibeam1_scan_dual_lidar.launch.py", "writer": "launch_arg"},
            {"key": "angle_offset", "label": "角度偏移", "type": "int", "file": "src/driver/scan_laser/Lakibeam_ROS2_Driver-main/launch/lakibeam1_scan_dual_lidar.launch.py", "writer": "launch_arg"},
            {"key": "scanTopic1", "label": "融合输入 1", "type": "str", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "scanTopic2", "label": "融合输入 2", "type": "str", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "laser1XOff", "label": "前雷达 X 偏移", "type": "float", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "laser1YOff", "label": "前雷达 Y 偏移", "type": "float", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "laser1ZOff", "label": "前雷达 Z 偏移", "type": "float", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "laser1Alpha", "label": "前雷达安装角", "type": "float", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "laser2XOff", "label": "后雷达 X 偏移", "type": "float", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "laser2YOff", "label": "后雷达 Y 偏移", "type": "float", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "laser2ZOff", "label": "后雷达 Z 偏移", "type": "float", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "laser2Alpha", "label": "后雷达安装角", "type": "float", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "flip1", "label": "前雷达镜像", "type": "bool", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "flip2", "label": "后雷达镜像", "type": "bool", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "inverse1", "label": "前雷达逆序", "type": "bool", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "inverse2", "label": "后雷达逆序", "type": "bool", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "pointCloudTopic", "label": "合并点云话题", "type": "str", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "pointCloutFrameId", "label": "合并点云 frame", "type": "str", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "range_min", "label": "LaserScan 最小距离", "type": "float", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "range_max", "label": "LaserScan 最大距离", "type": "float", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
            {"key": "target_frame", "label": "LaserScan target_frame", "type": "str", "file": "src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml", "writer": "yaml_scalar"},
        ],
    },
    {
        "id": "gps",
        "title": "gps GNSS",
        "description": "GNSS 串口和坐标系。",
        "items": [
            {"key": "serial_port", "label": "GNSS 串口", "type": "str", "file": "src/driver/gps/config/gnss_converter.yaml", "writer": "yaml_scalar"},
            {"key": "baudrate", "label": "GNSS 波特率", "type": "int", "file": "src/driver/gps/config/gnss_converter.yaml", "writer": "yaml_scalar"},
            {"key": "frame_id", "label": "GNSS frame_id", "type": "str", "file": "src/driver/gps/config/gnss_converter.yaml", "writer": "yaml_scalar"},
        ],
    },
    {
        "id": "lock_controller",
        "title": "lock_controller 门锁",
        "description": "485 锁板串口、波特率和默认板地址。",
        "items": [
            {"key": "serial_port", "label": "门锁串口", "type": "str", "file": "src/driver/lock_controller/config/lock_controller.yaml", "writer": "yaml_scalar"},
            {"key": "baud_rate", "label": "门锁波特率", "type": "int", "file": "src/driver/lock_controller/config/lock_controller.yaml", "writer": "yaml_scalar"},
            {"key": "board_address", "label": "默认板地址", "type": "int", "file": "src/driver/lock_controller/config/lock_controller.yaml", "writer": "yaml_scalar"},
        ],
    },
    {
        "id": "alarm_controller",
        "title": "alarm_controller 报警灯",
        "description": "报警灯串口和波特率。",
        "items": [
            {"key": "port", "label": "报警灯串口", "type": "str", "file": "src/driver/alarm_controller/alarm_controller/config/alarm_controller.yaml", "writer": "yaml_scalar"},
            {"key": "baudrate", "label": "报警灯波特率", "type": "int", "file": "src/driver/alarm_controller/alarm_controller/config/alarm_controller.yaml", "writer": "yaml_scalar"},
        ],
    },
]


class BuildState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.running = False
        has_install = (ROOT / "install/setup.bash").exists()
        self.returncode: int | None = 0 if has_install else None
        self.started_at: float | None = None
        self.finished_at: float | None = time.time() if has_install else None
        self.lines: list[str] = [
            "Detected install/setup.bash. Existing install space is available.\n"
            if has_install
            else ""
        ]

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "running": self.running,
                "returncode": self.returncode,
                "started_at": self.started_at,
                "finished_at": self.finished_at,
                "success": self.returncode == 0,
                "log": "".join(self.lines[-800:]),
            }

    def append(self, text: str) -> None:
        with self.lock:
            self.lines.append(text)
            if len(self.lines) > 2000:
                self.lines = self.lines[-1200:]


BUILD = BuildState()


def ros_prefix() -> str:
    return (
        "if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi; "
        "if [ -f install/setup.bash ]; then source install/setup.bash; fi"
    )


def ros_command(command: str) -> str:
    return f"{ros_prefix()}; {command}"


RUNTIME_TARGETS: dict[str, dict[str, Any]] = {
    "car_control": {
        "label": "底盘控制",
        "command": (
            "ros2 launch chassis_driver chassis_driver.launch.py & "
            "ros2 run cmdvel_2_ecu cmdvel_2_ecu; wait"
        ),
    },
    "imu": {
        "label": "IMU",
        "command": "ros2 launch fdilink_ahrs ahrs_driver.launch.py",
    },
    "rslidar": {
        "label": "速腾雷达",
        "command": "ros2 launch rslidar_sdk start.py",
    },
    "livox": {
        "label": "Livox Mid360",
        "command": (
            "ros2 launch livox_ros_driver2 msg_MID360_launch.py & "
            "ros2 run convert_laser convert; wait"
        ),
    },
    "scan_laser": {
        "label": "单线雷达",
        "command": (
            "ros2 launch lakibeam1 lakibeam1_scan_dual_lidar.launch.py & "
            "ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py; wait"
        ),
    },
    "gps": {
        "label": "GNSS",
        "command": "ros2 launch gnss_converter gnss_converter.launch.py",
    },
    "lock_controller": {
        "label": "门锁",
        "command": "ros2 launch lock_controller lock_controller.launch.py",
    },
    "alarm_controller": {
        "label": "报警灯",
        "command": (
            "ros2 run alarm_controller alarm_controller --ros-args --params-file "
            "src/driver/alarm_controller/alarm_controller/config/alarm_controller.yaml"
        ),
    },
}


class ManagedProcess:
    def __init__(self, target: str, label: str, command: str) -> None:
        self.target = target
        self.label = label
        self.command = command
        self.lock = threading.Lock()
        self.process: subprocess.Popen[str] | None = None
        self.started_at: float | None = None
        self.finished_at: float | None = None
        self.returncode: int | None = None
        self.lines: list[str] = []

    def running_unlocked(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            if self.process is not None and self.process.poll() is not None:
                self.returncode = self.process.returncode
                self.finished_at = self.finished_at or time.time()
            return {
                "target": self.target,
                "label": self.label,
                "running": self.running_unlocked(),
                "returncode": self.returncode,
                "started_at": self.started_at,
                "finished_at": self.finished_at,
                "log": "".join(self.lines[-400:]),
            }

    def append(self, text: str) -> None:
        with self.lock:
            self.lines.append(text)
            if len(self.lines) > 1000:
                self.lines = self.lines[-600:]

    def start(self) -> dict[str, Any]:
        with self.lock:
            if self.running_unlocked():
                already_running = True
            else:
                already_running = False
        if already_running:
            return {"started": False, "message": f"{self.label} 已在运行", "process": self.snapshot()}
        with self.lock:
            self.lines = []
            self.returncode = None
            self.started_at = time.time()
            self.finished_at = None
            full_cmd = ros_command(self.command)
            self.lines.append(f"$ cd {ROOT}\n$ bash -lc {shlex.quote(full_cmd)}\n\n")
            self.process = subprocess.Popen(
                ["bash", "-lc", full_cmd],
                cwd=ROOT,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
            )
            process = self.process
        threading.Thread(target=self._reader, args=(process,), daemon=True).start()
        return {"started": True, "message": f"{self.label} 启动中", "process": self.snapshot()}

    def _reader(self, process: subprocess.Popen[str]) -> None:
        assert process.stdout is not None
        for line in process.stdout:
            self.append(line)
        returncode = process.wait()
        with self.lock:
            if self.process is process:
                self.returncode = returncode
                self.finished_at = time.time()
        self.append(f"\nProcess finished with return code {returncode}\n")

    def stop(self) -> dict[str, Any]:
        with self.lock:
            process = self.process
            if process is None or process.poll() is not None:
                not_running = True
            else:
                not_running = False
                pid = process.pid
        if not_running:
            return {"stopped": False, "message": f"{self.label} 未运行", "process": self.snapshot()}
        assert process is not None
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(pid), signal.SIGTERM)
                process.wait(timeout=5)
        except ProcessLookupError:
            pass
        with self.lock:
            self.returncode = process.returncode
            self.finished_at = time.time()
        self.append(f"\nStopped {self.label}\n")
        return {"stopped": True, "message": f"{self.label} 已停止", "process": self.snapshot()}


RUNTIME = {
    target: ManagedProcess(target, cfg["label"], cfg["command"])
    for target, cfg in RUNTIME_TARGETS.items()
}


def runtime_snapshot() -> dict[str, Any]:
    return {target: manager.snapshot() for target, manager in RUNTIME.items()}


def handle_runtime(payload: dict[str, Any]) -> dict[str, Any]:
    target = str(payload.get("target", ""))
    action = str(payload.get("action", ""))
    manager = RUNTIME.get(target)
    if manager is None:
        return {"ok": False, "message": f"未知目标: {target}", "runtime": runtime_snapshot()}
    if action == "start":
        result = manager.start()
    elif action == "stop":
        result = manager.stop()
    else:
        result = {"ok": False, "message": f"未知动作: {action}", "process": manager.snapshot()}
    return {**result, "runtime": runtime_snapshot()}


def run_ros_cli(command: str, timeout: float = 8.0) -> dict[str, Any]:
    full_cmd = ros_command(command)
    try:
        completed = subprocess.run(
            ["bash", "-lc", full_cmd],
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
        )
        return {
            "ok": completed.returncode == 0,
            "returncode": completed.returncode,
            "output": completed.stdout.strip() or "(no output)",
        }
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout if isinstance(exc.stdout, str) else ""
        return {"ok": False, "returncode": 124, "output": (output.strip() + "\n命令超时，可能暂时没有收到数据。").strip()}


def bounded_float(value: Any, default: float, low: float, high: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = default
    if not math.isfinite(number):
        number = default
    return max(low, min(high, number))


def handle_ros_action(payload: dict[str, Any]) -> dict[str, Any]:
    action = str(payload.get("action", ""))
    topic_commands = {
        "rslidar_echo": "timeout 6s ros2 topic echo --once --no-arr /rslidar_points",
        "rslidar_hz": "timeout 6s ros2 topic hz /rslidar_points",
        "livox_echo": "timeout 6s ros2 topic echo --once --no-arr /livox_Pointcloud2",
        "livox_hz": "timeout 6s ros2 topic hz /livox_Pointcloud2",
        "scan_laser_echo": "timeout 6s ros2 topic echo --once --no-arr /laser/scan_merge",
        "scan_laser_hz": "timeout 6s ros2 topic hz /laser/scan_merge",
        "imu_echo": "timeout 6s ros2 topic echo --once /imu",
        "gnss_echo": "timeout 6s ros2 topic echo --once /sensing/gnss/pose_with_covariance",
    }
    if action in topic_commands:
        return run_ros_cli(topic_commands[action], timeout=9.0)
    if action == "alarm_on":
        return run_ros_cli("ros2 service call /control_alarm alarm_interface/srv/Alarm '{operate: true}'", timeout=8.0)
    if action == "alarm_off":
        return run_ros_cli("ros2 service call /control_alarm alarm_interface/srv/Alarm '{operate: false}'", timeout=8.0)
    if action == "lock_unlock":
        board = int(bounded_float(payload.get("board_address"), 1, 1, 32))
        lock = int(bounded_float(payload.get("lock_address"), 1, 0, 24))
        return run_ros_cli(
            f"ros2 service call /unlock lock_controller/srv/UnlockCommand '{{board_address: {board}, lock_address: {lock}}}'",
            timeout=8.0,
        )
    if action == "lock_power_on" or action == "lock_power_off":
        board = int(bounded_float(payload.get("board_address"), 1, 1, 32))
        lock = int(bounded_float(payload.get("lock_address"), 1, 1, 24))
        enable = "true" if action == "lock_power_on" else "false"
        return run_ros_cli(
            f"ros2 service call /long_power_control lock_controller/srv/LongPowerControl "
            f"'{{board_address: {board}, lock_address: {lock}, enable: {enable}}}'",
            timeout=8.0,
        )
    if action == "lock_query":
        board = int(bounded_float(payload.get("board_address"), 1, 1, 32))
        lock = int(bounded_float(payload.get("lock_address"), 0, 0, 24))
        return run_ros_cli(
            f"ros2 service call /query_status lock_controller/srv/QueryLockStatus "
            f"'{{board_address: {board}, lock_address: {lock}}}'",
            timeout=8.0,
        )
    if action == "cmd_vel":
        linear = bounded_float(payload.get("linear_x"), 0.0, -1.0, 1.0)
        angular = bounded_float(payload.get("angular_z"), 0.0, -1.5, 1.5)
        twist = (
            "{linear: {x: %.3f, y: 0.0, z: 0.0}, "
            "angular: {x: 0.0, y: 0.0, z: %.3f}}" % (linear, angular)
        )
        return run_ros_cli(f"ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{twist}'", timeout=8.0)
    return {"ok": False, "returncode": 1, "output": f"未知 ROS 动作: {action}"}


def decode_value(raw: str, value_type: str) -> Any:
    value = raw.strip()
    if "#" in value:
        value = value.split("#", 1)[0].strip()
    if value_type == "bool":
        return value.lower() in {"true", "1", "yes", "on"}
    if value_type == "int":
        return int(float(value.strip("'\"")))
    if value_type == "float":
        return float(value.strip("'\""))
    return value.strip("'\"")


def coerce_value(value: Any, value_type: str) -> Any:
    if value_type == "bool":
        if isinstance(value, bool):
            return value
        return str(value).lower() in {"true", "1", "yes", "on"}
    if value_type == "int":
        return int(value)
    if value_type == "float":
        return float(value)
    return str(value)


def format_value(value: Any, value_type: str, writer: str) -> str:
    value = coerce_value(value, value_type)
    if value_type == "bool":
        if writer == "py_dict":
            return "True" if value else "False"
        return "true" if value else "false"
    if value_type in {"int", "float"}:
        return str(value)
    text = str(value)
    if writer == "py_dict":
        return repr(text)
    return text


def read_yaml_scalar(path: Path, key: str, value_type: str) -> Any:
    text = path.read_text(encoding="utf-8")
    match = re.search(rf"^(\s*{re.escape(key)}\s*:\s*)(.+)$", text, flags=re.MULTILINE)
    if not match:
        raise KeyError(f"{key} not found in {path}")
    return decode_value(match.group(2), value_type)


def write_yaml_scalar(path: Path, key: str, value: Any, value_type: str) -> None:
    text = path.read_text(encoding="utf-8")
    replacement = format_value(value, value_type, "yaml_scalar")

    def repl(match: re.Match[str]) -> str:
        old_value = match.group(2)
        comment = ""
        if "#" in old_value:
            comment = " #" + old_value.split("#", 1)[1]
        return f"{match.group(1)}{replacement}{comment}"

    new_text, count = re.subn(
        rf"^(\s*{re.escape(key)}\s*:\s*)(.+)$",
        repl,
        text,
        count=1,
        flags=re.MULTILINE,
    )
    if count != 1:
        raise KeyError(f"{key} not found in {path}")
    path.write_text(new_text, encoding="utf-8")


def read_py_dict(path: Path, key: str, value_type: str) -> Any:
    text = path.read_text(encoding="utf-8")
    match = re.search(rf"(['\"]{re.escape(key)}['\"]\s*:\s*)([^,\n}}]+)", text)
    if not match:
        raise KeyError(f"{key} not found in {path}")
    return decode_value(match.group(2), value_type)


def write_py_dict(path: Path, key: str, value: Any, value_type: str) -> None:
    text = path.read_text(encoding="utf-8")
    replacement = format_value(value, value_type, "py_dict")
    new_text, count = re.subn(
        rf"(['\"]{re.escape(key)}['\"]\s*:\s*)([^,\n}}]+)",
        rf"\g<1>{replacement}",
        text,
        count=1,
    )
    if count != 1:
        raise KeyError(f"{key} not found in {path}")
    path.write_text(new_text, encoding="utf-8")


def read_launch_arg(path: Path, key: str, value_type: str) -> Any:
    text = path.read_text(encoding="utf-8")
    pattern = rf"DeclareLaunchArgument\(\s*['\"]{re.escape(key)}['\"][\s\S]*?default_value\s*=\s*(['\"])(.*?)\1"
    match = re.search(pattern, text)
    if not match:
        raise KeyError(f"{key} not found in {path}")
    return decode_value(match.group(2), value_type)


def write_launch_arg(path: Path, key: str, value: Any, value_type: str) -> None:
    text = path.read_text(encoding="utf-8")
    replacement = str(coerce_value(value, value_type))
    if value_type == "bool":
        replacement = "true" if coerce_value(value, value_type) else "false"
    pattern = rf"(DeclareLaunchArgument\(\s*['\"]{re.escape(key)}['\"][\s\S]*?default_value\s*=\s*)(['\"])(.*?)\2"

    def repl(match: re.Match[str]) -> str:
        return f"{match.group(1)}'{replacement}'"

    new_text, count = re.subn(pattern, repl, text, count=1)
    if count != 1:
        raise KeyError(f"{key} not found in {path}")
    path.write_text(new_text, encoding="utf-8")


READERS = {
    "yaml_scalar": read_yaml_scalar,
    "py_dict": read_py_dict,
    "launch_arg": read_launch_arg,
}

WRITERS = {
    "yaml_scalar": write_yaml_scalar,
    "py_dict": write_py_dict,
    "launch_arg": write_launch_arg,
}


def field_id(group: dict[str, Any], item: dict[str, Any]) -> str:
    return f"{group['id']}::{item['file']}::{item['key']}"


def export_config() -> dict[str, Any]:
    groups: list[dict[str, Any]] = []
    for group in FIELDS:
        out_items = []
        for item in group["items"]:
            path = rel(item["file"])
            try:
                value = READERS[item["writer"]](path, item["key"], item["type"])
                error = None
            except Exception as exc:  # keep the UI usable if one file is malformed
                value = ""
                error = str(exc)
            out_items.append({**item, "id": field_id(group, item), "value": value, "error": error})
        groups.append({k: group[k] for k in ("id", "title", "description")} | {"items": out_items})
    return {"workspace": str(ROOT), "groups": groups, "build": BUILD.snapshot()}


def save_config(values: dict[str, Any]) -> dict[str, Any]:
    if BUILD.snapshot()["returncode"] != 0 and os.environ.get("CONFIG_MANAGER_ALLOW_SAVE_BEFORE_BUILD") != "1":
        return {"saved": 0, "errors": ["请先点击“编译安装”，并等待编译成功后再保存配置。"]}
    errors: list[str] = []
    saved = 0
    by_id = {field_id(group, item): item for group in FIELDS for item in group["items"]}
    for fid, value in values.items():
        item = by_id.get(fid)
        if not item:
            errors.append(f"Unknown field: {fid}")
            continue
        try:
            WRITERS[item["writer"]](rel(item["file"]), item["key"], value, item["type"])
            saved += 1
        except Exception as exc:
            errors.append(f"{item['file']}:{item['key']}: {exc}")
    return {"saved": saved, "errors": errors}


def build_command() -> str:
    livox_dir = ROOT / "src/driver/lidar/livox_ros_driver2/src/livox_ros_driver2"
    setup_ros = "if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi"
    prepare_livox = ""
    if (livox_dir / "package_ROS2.xml").exists():
        prepare_livox = (
            f"cp -f {shlex.quote(str(livox_dir / 'package_ROS2.xml'))} "
            f"{shlex.quote(str(livox_dir / 'package.xml'))} && "
            f"cp -rf {shlex.quote(str(livox_dir / 'launch_ROS2'))} "
            f"{shlex.quote(str(livox_dir / 'launch'))}"
        )
    steps = [
        "set -e",
        setup_ros,
    ]
    if prepare_livox:
        steps.append(prepare_livox)
    steps.extend(
        [
            "colcon build --packages-select livox_ros_driver2 serial",
            "source install/setup.bash",
            "colcon build",
        ]
    )
    return " && ".join(steps)


def run_build() -> None:
    with BUILD.lock:
        if BUILD.running:
            return
        BUILD.running = True
        BUILD.returncode = None
        BUILD.started_at = time.time()
        BUILD.finished_at = None
        BUILD.lines = []
    cmd = build_command()
    BUILD.append(f"$ cd {ROOT}\n$ bash -lc {shlex.quote(cmd)}\n\n")
    process = subprocess.Popen(
        ["bash", "-lc", cmd],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert process.stdout is not None
    for line in process.stdout:
        BUILD.append(line)
    returncode = process.wait()
    with BUILD.lock:
        BUILD.running = False
        BUILD.returncode = returncode
        BUILD.finished_at = time.time()
    BUILD.append(f"\nBuild finished with return code {returncode}\n")


def start_build() -> dict[str, Any]:
    with BUILD.lock:
        if BUILD.running:
            running = True
        else:
            running = False
    if running:
        return {"started": False, "message": "build already running", "build": BUILD.snapshot()}
    thread = threading.Thread(target=run_build, daemon=True)
    thread.start()
    return {"started": True, "message": "build started", "build": BUILD.snapshot()}


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>real_ws 驱动配置</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f5f7fb;
      --panel: #ffffff;
      --line: #d9e0ea;
      --text: #19202a;
      --muted: #637083;
      --blue: #1967d2;
      --green: #177245;
      --red: #b42318;
      --amber: #925c00;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--bg);
      color: var(--text);
      letter-spacing: 0;
    }
    .shell {
      min-height: 100vh;
      display: grid;
      grid-template-columns: 260px minmax(0, 1fr);
    }
    aside {
      background: #ffffff;
      border-right: 1px solid var(--line);
      padding: 18px 14px;
      position: sticky;
      top: 0;
      height: 100vh;
      overflow: auto;
    }
    h1 { font-size: 18px; margin: 0 4px 16px; font-weight: 650; }
    main { min-width: 0; padding: 20px 24px; }
    .topbar {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
      margin-bottom: 16px;
    }
    .page-title { min-width: 0; }
    .page-title h2 { font-size: 20px; margin: 0 0 4px; }
    .page-title p { margin: 0; color: var(--muted); font-size: 13px; }
    button {
      height: 36px;
      border-radius: 6px;
      border: 1px solid var(--line);
      background: #fff;
      color: var(--text);
      padding: 0 14px;
      cursor: pointer;
      font-weight: 600;
    }
    button.primary { background: var(--blue); border-color: var(--blue); color: #fff; }
    button:disabled { opacity: .55; cursor: not-allowed; }
    .toolbar { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
    .nav {
      display: grid;
      gap: 6px;
    }
    .nav button {
      width: 100%;
      justify-content: flex-start;
      text-align: left;
      height: 38px;
      font-weight: 600;
    }
    .nav button.active {
      background: #e8f0fe;
      color: var(--blue);
      border-color: #bad0f7;
    }
    .status { font-size: 13px; color: var(--muted); }
    .status.ok { color: var(--green); }
    .status.err { color: var(--red); }
    .build {
      background: #101828;
      color: #d0d5dd;
      border-radius: 6px;
      padding: 12px;
      min-height: 170px;
      max-height: 320px;
      overflow: auto;
      white-space: pre-wrap;
      font: 12px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      border: 1px solid #243044;
      margin-bottom: 18px;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      margin-bottom: 14px;
      overflow: hidden;
    }
    section h2 {
      font-size: 16px;
      margin: 0;
      padding: 14px 16px 4px;
    }
    section p {
      margin: 0;
      padding: 0 16px 12px;
      color: var(--muted);
      font-size: 13px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
      gap: 12px;
      padding: 14px 16px 16px;
      border-top: 1px solid var(--line);
    }
    .grid.single {
      grid-template-columns: 1fr;
    }
    .split {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px;
      padding: 14px 16px 16px;
      border-top: 1px solid var(--line);
    }
    .column {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      background: #fbfcfe;
    }
    .column h3 {
      margin: 0 0 10px;
      font-size: 15px;
      font-weight: 650;
    }
    .column .fields {
      display: grid;
      gap: 12px;
    }
    .block-title {
      padding: 14px 16px 0;
      color: var(--text);
      font-size: 14px;
      font-weight: 650;
    }
    .car-layout {
      display: grid;
      gap: 16px;
      padding: 16px;
      border-top: 1px solid var(--line);
    }
    .car-block {
      display: grid;
      gap: 12px;
    }
    .car-block h3 {
      margin: 0;
      font-size: 15px;
      font-weight: 650;
      color: var(--text);
    }
    .car-fields {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(210px, 1fr));
      gap: 12px;
    }
    .car-fields.primary {
      grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
    }
    .can-pair {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px;
    }
    .can-panel {
      display: grid;
      gap: 12px;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      background: #fbfcfe;
    }
    .can-panel h4 {
      margin: 0;
      font-size: 14px;
      font-weight: 650;
      color: var(--text);
    }
    .can-panel .fields {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }
    .car-layout .path {
      display: none;
    }
    .runtime {
      display: grid;
      gap: 12px;
      padding: 14px 16px 16px;
      border-top: 1px solid var(--line);
      background: #fbfcfe;
    }
    .runtime-head {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: center;
      flex-wrap: wrap;
    }
    .runtime-title {
      margin: 0;
      font-size: 15px;
      font-weight: 650;
      color: var(--text);
    }
    .runtime-actions {
      display: flex;
      gap: 8px;
      align-items: center;
      flex-wrap: wrap;
    }
    .runtime-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 12px;
    }
    .runtime-log {
      background: #101828;
      color: #d0d5dd;
      border: 1px solid #243044;
      border-radius: 6px;
      min-height: 92px;
      max-height: 190px;
      overflow: auto;
      white-space: pre-wrap;
      padding: 10px;
      font: 12px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    .drive-pad {
      display: grid;
      grid-template-columns: repeat(3, minmax(74px, 1fr));
      gap: 8px;
      max-width: 310px;
    }
    .drive-pad button {
      padding: 0 8px;
    }
    .danger {
      border-color: var(--red);
      color: var(--red);
      background: #fff;
    }
    label {
      display: grid;
      gap: 6px;
      font-size: 13px;
      color: var(--muted);
    }
    input, select {
      width: 100%;
      min-height: 36px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 7px 10px;
      font-size: 14px;
      background: #fff;
      color: var(--text);
    }
    .path {
      color: #7a8699;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
      font-size: 11px;
    }
    .error { color: var(--red); font-size: 12px; }
    .notice {
      border-left: 4px solid var(--amber);
      background: #fff8e8;
      padding: 10px 12px;
      border-radius: 6px;
      margin-bottom: 16px;
      color: #4a3600;
      font-size: 13px;
    }
    .pane { display: none; }
    .pane.active { display: block; }
    .build-panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
      margin-bottom: 14px;
    }
    @media (max-width: 760px) {
      .shell { grid-template-columns: 1fr; }
      aside {
        position: static;
        height: auto;
        border-right: 0;
        border-bottom: 1px solid var(--line);
      }
      .nav {
        grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
      }
      main { padding: 16px; }
      .topbar { align-items: flex-start; flex-direction: column; }
      .split { grid-template-columns: 1fr; }
      .can-pair { grid-template-columns: 1fr; }
      .can-panel .fields { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="shell">
    <aside>
      <h1>real_ws 驱动配置</h1>
      <nav id="nav" class="nav"></nav>
    </aside>
    <main>
      <div class="topbar">
        <div class="page-title">
          <h2 id="pageHeading">编译安装</h2>
          <p id="pageDescription">执行工作区完整编译安装流程。</p>
        </div>
        <div class="toolbar">
          <span id="buildStatus" class="status">未编译</span>
          <button id="buildBtn" class="primary">编译安装</button>
          <button id="reloadBtn">重新加载</button>
          <button id="saveBtn" class="primary">保存配置</button>
        </div>
      </div>
      <div class="notice">不包含 camera 参数。Livox 页面提供单独启动和点云查看，参数仍使用原始 JSON 文件；保存会直接修改源码区配置文件。</div>
      <div id="buildPane" class="pane active">
        <div class="build-panel">
          <div class="toolbar">
            <button id="buildBtnPanel" class="primary">开始编译安装</button>
            <span id="buildStatusPanel" class="status">未编译</span>
          </div>
        </div>
        <pre id="buildLog" class="build">等待操作...</pre>
      </div>
      <form id="configForm"></form>
    </main>
  </div>
<script>
const form = document.getElementById('configForm');
const buildLog = document.getElementById('buildLog');
const buildStatus = document.getElementById('buildStatus');
const buildStatusPanel = document.getElementById('buildStatusPanel');
const pageHeading = document.getElementById('pageHeading');
const pageDescription = document.getElementById('pageDescription');
const nav = document.getElementById('nav');
const buildPane = document.getElementById('buildPane');
let activePane = 'build';
let groups = [];
let pollTimer = null;
let runtime = {};
let runtimeTimer = null;

function inputFor(item) {
  if (item.type === 'bool') {
    const select = document.createElement('select');
    select.name = item.id;
    select.dataset.type = item.type;
    select.dataset.config = '1';
    select.innerHTML = '<option value="true">true</option><option value="false">false</option>';
    select.value = String(Boolean(item.value));
    return select;
  }
  const input = document.createElement('input');
  input.name = item.id;
  input.dataset.type = item.type;
  input.dataset.config = '1';
  input.value = item.value ?? '';
  input.type = item.type === 'str' ? 'text' : 'number';
  if (item.type === 'float') input.step = 'any';
  return input;
}

function fieldFor(item) {
  const label = document.createElement('label');
  label.textContent = item.label;
  label.appendChild(inputFor(item));
  const path = document.createElement('div');
  path.className = 'path';
  path.textContent = `${item.file} : ${item.key}`;
  label.appendChild(path);
  if (item.error) {
    const error = document.createElement('div');
    error.className = 'error';
    error.textContent = item.error;
    label.appendChild(error);
  }
  return label;
}

function runtimeStatusText(target) {
  const state = runtime[target];
  if (!state) return '未启动';
  if (state.running) return '运行中';
  if (state.returncode === 0) return '已停止';
  if (state.returncode !== null && state.returncode !== undefined) return '异常退出';
  return '未启动';
}

function runtimeStatusClass(target) {
  const state = runtime[target];
  if (!state) return 'status';
  if (state.running) return 'status ok';
  if (state.returncode !== null && state.returncode !== undefined && state.returncode !== 0) return 'status err';
  return 'status';
}

function button(text, className = '') {
  const el = document.createElement('button');
  el.type = 'button';
  el.textContent = text;
  if (className) el.className = className;
  return el;
}

function numberInput(value, step = '0.1', min = null, max = null) {
  const input = document.createElement('input');
  input.type = 'number';
  input.value = value;
  input.step = step;
  if (min !== null) input.min = min;
  if (max !== null) input.max = max;
  return input;
}

async function callRuntime(target, action) {
  const res = await fetch('/api/runtime', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({target, action})
  });
  const data = await res.json();
  runtime = data.runtime || runtime;
  const log = document.querySelector(`[data-runtime-log="${target}"]`);
  if (log) log.dataset.showProcessLog = '1';
  updateRuntimePanels();
}

async function callRosAction(action, payload = {}, outputEl = null) {
  if (outputEl) {
    outputEl.dataset.showProcessLog = '0';
    outputEl.textContent = '执行中...';
  }
  const res = await fetch('/api/ros', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action, ...payload})
  });
  const data = await res.json();
  if (outputEl) {
    outputEl.textContent = `$ ${action}\nreturncode: ${data.returncode}\n\n${data.output || ''}`;
    outputEl.scrollTop = outputEl.scrollHeight;
  }
}

function appendRuntimeControls(target, controls, output) {
  if (target === 'car_control') {
    const grid = document.createElement('div');
    grid.className = 'runtime-grid';
    const linear = numberInput('0.20', '0.05', '-1.0', '1.0');
    const angular = numberInput('0.50', '0.05', '-1.5', '1.5');
    const linearLabel = document.createElement('label');
    linearLabel.textContent = '线速度 x';
    linearLabel.appendChild(linear);
    const angularLabel = document.createElement('label');
    angularLabel.textContent = '角速度 z';
    angularLabel.appendChild(angular);
    grid.appendChild(linearLabel);
    grid.appendChild(angularLabel);
    controls.appendChild(grid);

    const pad = document.createElement('div');
    pad.className = 'drive-pad';
    const forward = button('前进');
    const back = button('后退');
    const left = button('左转');
    const stop = button('停止', 'danger');
    const right = button('右转');
    const blank1 = document.createElement('span');
    const blank2 = document.createElement('span');
    forward.addEventListener('click', () => callRosAction('cmd_vel', {linear_x: linear.value, angular_z: 0}, output));
    back.addEventListener('click', () => callRosAction('cmd_vel', {linear_x: -Math.abs(Number(linear.value || 0)), angular_z: 0}, output));
    left.addEventListener('click', () => callRosAction('cmd_vel', {linear_x: 0, angular_z: Math.abs(Number(angular.value || 0))}, output));
    right.addEventListener('click', () => callRosAction('cmd_vel', {linear_x: 0, angular_z: -Math.abs(Number(angular.value || 0))}, output));
    stop.addEventListener('click', () => callRosAction('cmd_vel', {linear_x: 0, angular_z: 0}, output));
    pad.append(blank1, forward, blank2, left, stop, right);
    controls.appendChild(pad);
  } else if (target === 'rslidar') {
    const echo = button('查看点云数据');
    const hz = button('查看点云频率');
    echo.addEventListener('click', () => callRosAction('rslidar_echo', {}, output));
    hz.addEventListener('click', () => callRosAction('rslidar_hz', {}, output));
    controls.append(echo, hz);
  } else if (target === 'livox') {
    const echo = button('查看点云数据');
    const hz = button('查看点云频率');
    echo.addEventListener('click', () => callRosAction('livox_echo', {}, output));
    hz.addEventListener('click', () => callRosAction('livox_hz', {}, output));
    controls.append(echo, hz);
  } else if (target === 'scan_laser') {
    const echo = button('查看合并点云');
    const hz = button('查看点云频率');
    echo.addEventListener('click', () => callRosAction('scan_laser_echo', {}, output));
    hz.addEventListener('click', () => callRosAction('scan_laser_hz', {}, output));
    controls.append(echo, hz);
  } else if (target === 'imu') {
    const echo = button('查看 IMU 数据');
    echo.addEventListener('click', () => callRosAction('imu_echo', {}, output));
    controls.appendChild(echo);
  } else if (target === 'gps') {
    const echo = button('查看 GNSS 数据');
    echo.addEventListener('click', () => callRosAction('gnss_echo', {}, output));
    controls.appendChild(echo);
  } else if (target === 'alarm_controller') {
    const on = button('打开报警灯', 'primary');
    const off = button('关闭报警灯');
    on.addEventListener('click', () => callRosAction('alarm_on', {}, output));
    off.addEventListener('click', () => callRosAction('alarm_off', {}, output));
    controls.append(on, off);
  } else if (target === 'lock_controller') {
    const grid = document.createElement('div');
    grid.className = 'runtime-grid';
    const board = numberInput('1', '1', '1', '32');
    const lock = numberInput('1', '1', '0', '24');
    const boardLabel = document.createElement('label');
    boardLabel.textContent = '板地址';
    boardLabel.appendChild(board);
    const lockLabel = document.createElement('label');
    lockLabel.textContent = '锁地址';
    lockLabel.appendChild(lock);
    grid.append(boardLabel, lockLabel);
    controls.appendChild(grid);
    const unlock = button('开锁', 'primary');
    const powerOn = button('长通电开');
    const powerOff = button('长通电关');
    const query = button('查询状态');
    const payload = () => ({board_address: board.value, lock_address: lock.value});
    unlock.addEventListener('click', () => callRosAction('lock_unlock', payload(), output));
    powerOn.addEventListener('click', () => callRosAction('lock_power_on', payload(), output));
    powerOff.addEventListener('click', () => callRosAction('lock_power_off', payload(), output));
    query.addEventListener('click', () => callRosAction('lock_query', payload(), output));
    controls.append(unlock, powerOn, powerOff, query);
  }
}

function renderRuntimePanel(section, group) {
  if (!['car_control', 'imu', 'rslidar', 'livox', 'scan_laser', 'gps', 'lock_controller', 'alarm_controller'].includes(group.id)) return;
  const panel = document.createElement('div');
  panel.className = 'runtime';
  panel.dataset.runtimeTarget = group.id;
  const head = document.createElement('div');
  head.className = 'runtime-head';
  const title = document.createElement('div');
  title.className = 'runtime-title';
  title.textContent = '运行控制';
  const actions = document.createElement('div');
  actions.className = 'runtime-actions';
  const status = document.createElement('span');
  status.className = runtimeStatusClass(group.id);
  status.dataset.runtimeStatus = group.id;
  status.textContent = runtimeStatusText(group.id);
  const start = button('启动', 'primary');
  const stop = button('停止');
  start.addEventListener('click', () => callRuntime(group.id, 'start'));
  stop.addEventListener('click', () => callRuntime(group.id, 'stop'));
  actions.append(status, start, stop);
  head.append(title, actions);
  const controls = document.createElement('div');
  controls.className = 'runtime-actions';
  const output = document.createElement('pre');
  output.className = 'runtime-log';
  output.dataset.runtimeLog = group.id;
  output.textContent = '等待操作...';
  appendRuntimeControls(group.id, controls, output);
  panel.append(head, controls, output);
  section.appendChild(panel);
}

function updateRuntimePanels() {
  document.querySelectorAll('[data-runtime-status]').forEach(el => {
    const target = el.dataset.runtimeStatus;
    el.textContent = runtimeStatusText(target);
    el.className = runtimeStatusClass(target);
  });
  document.querySelectorAll('[data-runtime-log]').forEach(el => {
    const target = el.dataset.runtimeLog;
    const state = runtime[target];
    if (!state || !state.log) return;
    if (el.textContent === '等待操作...' || el.dataset.showProcessLog === '1') {
      el.textContent = state.log;
      el.dataset.showProcessLog = '1';
      el.scrollTop = el.scrollHeight;
    }
  });
}

async function loadRuntime() {
  const res = await fetch('/api/runtime');
  runtime = await res.json();
  updateRuntimePanels();
}

function renderStandardGroup(section, group) {
  if (!group.items.length) return;
  const grid = document.createElement('div');
  grid.className = 'grid';
  group.items.forEach(item => grid.appendChild(fieldFor(item)));
  section.appendChild(grid);
}

function renderGridBlock(section, title, items) {
  if (!items.length) return;
  const blockTitle = document.createElement('div');
  blockTitle.className = 'block-title';
  blockTitle.textContent = title;
  section.appendChild(blockTitle);
  const grid = document.createElement('div');
  grid.className = 'grid';
  items.forEach(item => grid.appendChild(fieldFor(item)));
  section.appendChild(grid);
}

function renderSingleBlock(section, title, items) {
  if (!items.length) return;
  const blockTitle = document.createElement('div');
  blockTitle.className = 'block-title';
  blockTitle.textContent = title;
  section.appendChild(blockTitle);
  const grid = document.createElement('div');
  grid.className = 'grid single';
  items.forEach(item => grid.appendChild(fieldFor(item)));
  section.appendChild(grid);
}

function renderCarBlock(title, items, className = '') {
  const block = document.createElement('div');
  block.className = 'car-block';
  const heading = document.createElement('h3');
  heading.textContent = title;
  block.appendChild(heading);
  const fields = document.createElement('div');
  fields.className = `car-fields ${className}`.trim();
  items.forEach(item => fields.appendChild(fieldFor(item)));
  block.appendChild(fields);
  return block;
}

function renderCanPanel(title, items) {
  const panel = document.createElement('div');
  panel.className = 'can-panel';
  const heading = document.createElement('h4');
  heading.textContent = title;
  panel.appendChild(heading);
  const fields = document.createElement('div');
  fields.className = 'fields';
  items.forEach(item => fields.appendChild(fieldFor(item)));
  panel.appendChild(fields);
  return panel;
}

function renderLaserColumn(title, items) {
  const column = document.createElement('div');
  column.className = 'column';
  const heading = document.createElement('h3');
  heading.textContent = title;
  column.appendChild(heading);
  const fields = document.createElement('div');
  fields.className = 'fields';
  items.forEach(item => fields.appendChild(fieldFor(item)));
  column.appendChild(fields);
  return column;
}

function renderScanLaserGroup(section, group) {
  const byKey = Object.fromEntries(group.items.map(item => [item.key, item]));
  const pick = keys => keys.map(key => byKey[key]).filter(Boolean);
  const common = pick(['frame_id', 'hostip', 'inverted', 'angle_offset']);
  const front = pick(['output_topic0', 'port_front', 'scanTopic1', 'laser1XOff', 'laser1YOff', 'laser1ZOff', 'laser1Alpha', 'flip1', 'inverse1']);
  const back = pick(['output_topic1', 'port_back', 'scanTopic2', 'laser2XOff', 'laser2YOff', 'laser2ZOff', 'laser2Alpha', 'flip2', 'inverse2']);
  const output = pick(['pointCloudTopic', 'pointCloutFrameId', 'range_min', 'range_max', 'target_frame']);

  renderGridBlock(section, '公共参数', common);
  const split = document.createElement('div');
  split.className = 'split';
  split.appendChild(renderLaserColumn('前单线雷达', front));
  split.appendChild(renderLaserColumn('后单线雷达', back));
  section.appendChild(split);
  renderGridBlock(section, '融合输出', output);
}

function renderCarControlGroup(section, group) {
  const byKey = Object.fromEntries(group.items.map(item => [item.key, item]));
  const pick = keys => keys.map(key => byKey[key]).filter(Boolean);
  const canCommon = pick([
    'canbus_type', 'main_can_id', 'debug_can_id', 'can_eth_card',
    'local_ip', 'local_port', 'debug_mode', 'show_sending_msg',
    'car_type', 'vehicle_weight'
  ]);
  const can1 = pick(['can1_remote_ip', 'can1_remote_port']);
  const can2 = pick(['can2_remote_ip', 'can2_remote_port']);
  const motion = pick([
    'L', 'LOW_SPEED_THRESH', 'STOP_THRESH', 'OMEGA_DEADZONE',
    'MAX_STEER_DEG', 'OMEGA_FULL_STEER', 'rear_wheel_flag'
  ]);

  const layout = document.createElement('div');
  layout.className = 'car-layout';
  layout.appendChild(renderCarBlock('CAN 通信公共参数', canCommon, 'primary'));
  const canPair = document.createElement('div');
  canPair.className = 'can-pair';
  canPair.appendChild(renderCanPanel('CAN1 远端', can1));
  canPair.appendChild(renderCanPanel('CAN2 远端', can2));
  layout.appendChild(canPair);
  layout.appendChild(renderCarBlock('运动控制参数', motion));
  section.appendChild(layout);
}

function render(data) {
  groups = data.groups;
  renderNav();
  form.innerHTML = '';
  data.groups.forEach(group => {
    const section = document.createElement('section');
    section.id = `pane-${group.id}`;
    section.className = 'pane';
    section.innerHTML = `<h2>${group.title}</h2><p>${group.description}</p>`;
    renderRuntimePanel(section, group);
    if (group.id === 'scan_laser') renderScanLaserGroup(section, group);
    else if (group.id === 'car_control') renderCarControlGroup(section, group);
    else renderStandardGroup(section, group);
    form.appendChild(section);
  });
  updateBuild(data.build);
  showPane(activePane);
  loadRuntime();
}

function updateBuild(build) {
  buildLog.textContent = build.log || '等待操作...';
  buildLog.scrollTop = buildLog.scrollHeight;
  if (build.running) {
    buildStatus.textContent = '编译中';
    buildStatusPanel.textContent = '编译中';
    buildStatus.className = 'status';
    buildStatusPanel.className = 'status';
    document.getElementById('buildBtn').disabled = true;
    document.getElementById('buildBtnPanel').disabled = true;
  } else if (build.returncode === 0) {
    buildStatus.textContent = '编译成功';
    buildStatusPanel.textContent = '编译成功';
    buildStatus.className = 'status ok';
    buildStatusPanel.className = 'status ok';
    document.getElementById('buildBtn').disabled = false;
    document.getElementById('buildBtnPanel').disabled = false;
  } else if (build.returncode !== null) {
    buildStatus.textContent = '编译失败';
    buildStatusPanel.textContent = '编译失败';
    buildStatus.className = 'status err';
    buildStatusPanel.className = 'status err';
    document.getElementById('buildBtn').disabled = false;
    document.getElementById('buildBtnPanel').disabled = false;
  } else {
    buildStatus.textContent = '未编译';
    buildStatusPanel.textContent = '未编译';
    buildStatus.className = 'status';
    buildStatusPanel.className = 'status';
    document.getElementById('buildBtn').disabled = false;
    document.getElementById('buildBtnPanel').disabled = false;
  }
  document.getElementById('saveBtn').disabled = build.returncode !== 0;
}

function renderNav() {
  nav.innerHTML = '';
  const buildButton = navButton('build', '编译安装');
  nav.appendChild(buildButton);
  groups.forEach(group => nav.appendChild(navButton(group.id, group.title)));
}

function navButton(target, text) {
  const button = document.createElement('button');
  button.type = 'button';
  button.dataset.target = target;
  button.textContent = text;
  button.addEventListener('click', () => showPane(target));
  return button;
}

function showPane(target) {
  activePane = target;
  buildPane.classList.toggle('active', target === 'build');
  document.querySelectorAll('#configForm .pane').forEach(pane => {
    pane.classList.toggle('active', pane.id === `pane-${target}`);
  });
  document.querySelectorAll('#nav button').forEach(button => {
    button.classList.toggle('active', button.dataset.target === target);
  });
  const group = groups.find(item => item.id === target);
  if (target === 'build') {
    pageHeading.textContent = '编译安装';
    pageDescription.textContent = '执行工作区完整编译安装流程。';
    document.getElementById('saveBtn').style.display = 'none';
  } else if (group) {
    pageHeading.textContent = group.title;
    pageDescription.textContent = group.description;
    document.getElementById('saveBtn').style.display = '';
  }
}

async function loadConfig() {
  const res = await fetch('/api/config');
  render(await res.json());
}

async function pollBuild() {
  const res = await fetch('/api/build');
  const data = await res.json();
  updateBuild(data);
  if (!data.running && pollTimer) {
    clearInterval(pollTimer);
    pollTimer = null;
  }
}

document.getElementById('buildBtn').addEventListener('click', async () => {
  const res = await fetch('/api/build', {method: 'POST'});
  updateBuild((await res.json()).build);
  if (!pollTimer) pollTimer = setInterval(pollBuild, 1200);
});

document.getElementById('buildBtnPanel').addEventListener('click', async () => {
  const res = await fetch('/api/build', {method: 'POST'});
  updateBuild((await res.json()).build);
  if (!pollTimer) pollTimer = setInterval(pollBuild, 1200);
});

document.getElementById('reloadBtn').addEventListener('click', loadConfig);

document.getElementById('saveBtn').addEventListener('click', async () => {
  const values = {};
  const activeFormPane = document.getElementById(`pane-${activePane}`);
  if (!activeFormPane) return;
  [...activeFormPane.querySelectorAll('input[data-config], select[data-config]')].forEach(el => {
    if (!el.name) return;
    if (el.dataset.type === 'bool') values[el.name] = el.value === 'true';
    else values[el.name] = el.value;
  });
  const res = await fetch('/api/config', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({values})
  });
  const data = await res.json();
  if (data.errors && data.errors.length) {
    alert('部分配置保存失败:\n' + data.errors.join('\n'));
  } else {
    alert(`已保存 ${data.saved} 个字段`);
  }
  await loadConfig();
});

loadConfig();
loadRuntime();
runtimeTimer = setInterval(loadRuntime, 2000);
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def send_no_cache_headers(self) -> None:
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")

    def send_json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_no_cache_headers()
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            body = HTML.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_no_cache_headers()
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == "/api/config":
            self.send_json(export_config())
            return
        if parsed.path == "/api/build":
            self.send_json(BUILD.snapshot())
            return
        if parsed.path == "/api/runtime":
            self.send_json(runtime_snapshot())
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            payload = {}
        parsed = urlparse(self.path)
        if parsed.path == "/api/config":
            self.send_json(save_config(payload.get("values", {})))
            return
        if parsed.path == "/api/build":
            self.send_json(start_build())
            return
        if parsed.path == "/api/runtime":
            self.send_json(handle_runtime(payload))
            return
        if parsed.path == "/api/ros":
            self.send_json(handle_ros_action(payload))
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[config_manager] {self.address_string()} - {fmt % args}")


def main() -> None:
    host = os.environ.get("CONFIG_MANAGER_HOST", "127.0.0.1")
    port = int(os.environ.get("CONFIG_MANAGER_PORT", "8765"))
    server = ThreadingHTTPServer((host, port), Handler)
    print(f"Config manager: http://{host}:{port}")
    print(f"Workspace: {ROOT}")
    server.serve_forever()


if __name__ == "__main__":
    main()
