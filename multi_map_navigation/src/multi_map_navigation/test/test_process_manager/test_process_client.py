#!/usr/bin/env python3
"""
进程管理器服务测试客户端
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header

from multi_map_navigation_msgs.srv import StartProcess, ShutdownProcess, GetProcessStatus


# 与 process_manager 中管理的子进程一一对应；shutdown 模式按此顺序依次关闭。
PROCESSES = ["re_localization", "nav2_init_pose", "liosam", "navigation2"]

# 将逻辑进程名映射到 GetProcessStatus 响应中的布尔字段名
STATUS_FIELDS = {
    "re_localization": "re_localization_running",
    "liosam": "liosam_running",
    "nav2_init_pose": "nav2_init_pose_running",
    "navigation2": "navigation2_running",
}


def status_field_name(process_name: str) -> str:
    """将逻辑进程名映射到 GetProcessStatus 响应中的布尔字段名。"""
    return STATUS_FIELDS[process_name]


def call_service(node: Node, client, request, timeout_sec: float):
    """同步封装异步服务调用：spin 直到完成或超时，超时返回 None。"""
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    return future.result() if future.done() else None


def status_to_dict(status_res):
    """将 GetProcessStatus 响应转换为便于日志输出的字典。"""
    if status_res is None:
        return None
    return {
        "re_localization": status_res.re_localization_running,
        "nav2_init_pose": status_res.nav2_init_pose_running,
        "liosam": status_res.liosam_running,
        "navigation2": status_res.navigation2_running,
    }


def main():
    rclpy.init(args=None)
    node = Node("test_process_client")
    # 命令行开关
    double_start_requested = "--double-start" in sys.argv[1:]
    shutdown_status_only = "--shutdown-status-only" in sys.argv[1:]
    status_only = "--status-only" in sys.argv[1:]
    double_start_delay_ms = 0 # 双启动延迟时间
    for arg in sys.argv[1:]:
        if arg.startswith("--double-start-delay-ms="):
            try:
                double_start_delay_ms = max(0, int(arg.split("=", 1)[1]))
            except ValueError:
                pass

    # 创建三个客户端
    start_client = node.create_client(StartProcess, "/process_manager/start_process")
    shutdown_client = node.create_client(ShutdownProcess, "/process_manager/shutdown_process")
    status_client = node.create_client(GetProcessStatus, "/process_manager/get_status")
    # 等待服务可用
    for client_obj, _service_name in (
        (start_client, "start_process"),
        (shutdown_client, "shutdown_process"),
        (status_client, "get_status"),
    ):
        while not client_obj.wait_for_service(timeout_sec=1.0):
            pass

    # 默认用 map1
    map_name = "map1"
    for arg in sys.argv[1:]:
        if not arg.startswith("--"):
            map_name = arg
            break

    if not shutdown_status_only:
        # 启动 nav2_init_pose 前持续给GPS，避免其卡在“等待 GPS 数据”
        gps_pub = node.create_publisher(NavSatFix, "/sensing/gnss/pose_with_covariance", 10)
        # 发布伪造 GPS 数据
        def publish_fake_gps():
            msg = NavSatFix()
            msg.header = Header()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "gps_link"
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.longitude = 7.0
            msg.latitude = 1.0
            msg.altitude = 10.0
            msg.position_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            gps_pub.publish(msg)

        node.create_timer(1.0, publish_fake_gps)
    # 测试模式
    mode = "start"
    if status_only:
        mode = "status-only"
    elif shutdown_status_only:
        mode = "shutdown-status-only"

    mode_desc = f"mode={mode}"
    if double_start_requested:
        mode_desc += ", double-start"
        mode_desc += f", delay={double_start_delay_ms}ms"
    node.get_logger().info(mode_desc)

    if status_only:
        # 单独测试 get_status 服务
        res = call_service(node, status_client, GetProcessStatus.Request(), timeout_sec=10.0)
        node.get_logger().info(f"status: {status_to_dict(res)}")
        node.destroy_node()
        rclpy.shutdown()
        return

    if shutdown_status_only:
        # 单独测试 shutdown + get_status（关闭前/后各查一次）
        before = call_service(node, status_client, GetProcessStatus.Request(), timeout_sec=10.0)
        shutdown_summary = []
        # 按顺序关闭进程
        for process_name in PROCESSES:
            req = ShutdownProcess.Request()
            req.process_name = process_name
            res = call_service(node, shutdown_client, req, timeout_sec=20.0)
            shutdown_summary.append(
                f"{process_name}:{False if res is None else bool(res.success)}"
            )
        # 关闭后查询状态
        after = call_service(node, status_client, GetProcessStatus.Request(), timeout_sec=10.0)
        node.get_logger().info(
            f"shutdown summary: {'; '.join(shutdown_summary)} | "
            f"before={status_to_dict(before)} | "
            f"after={status_to_dict(after)}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    start_plan = [
        ("re_localization", map_name),
        ("nav2_init_pose", map_name),
        ("liosam", ""),
        ("navigation2", map_name),
    ]
    # 启动日志
    start_summary = []
    running_list = []

    for process_name, req_map in start_plan:
        if double_start_requested:
            # 双启动测试：先发第一次，再按配置延时后发第二次
            req1 = StartProcess.Request()
            req1.process_name = process_name
            req1.map_name = req_map
            req2 = StartProcess.Request()
            req2.process_name = process_name
            req2.map_name = req_map
            future1 = start_client.call_async(req1)
            # 按配置延时后发第二次
            if double_start_delay_ms > 0:
                time.sleep(double_start_delay_ms / 1000.0)
            future2 = start_client.call_async(req2)
            # 等待第一、二次启动完成
            rclpy.spin_until_future_complete(node, future1, timeout_sec=20.0)
            rclpy.spin_until_future_complete(node, future2, timeout_sec=20.0)
            res1 = future1.result() if future1.done() else None
            res2 = future2.result() if future2.done() else None
            first_ok = (res1 is not None and bool(res1.success))
            second_ok = (res2 is not None and bool(res2.success))
            start_summary.append(f"{process_name}[1st={first_ok};2nd={second_ok}]")
            if not first_ok:
                continue
        # 普通单次启动
        else:
            req = StartProcess.Request()
            req.process_name = process_name
            req.map_name = req_map
            res = call_service(node, start_client, req, timeout_sec=15.0)
            ok = (res is not None and bool(res.success))
            start_summary.append(f"{process_name}[{ok}]")
            if not ok:
                continue
        # 轮询状态服务，确认进程真的进入运行态，避免启动失败
        end_time = time.time() + 120.0
        field = status_field_name(process_name)
        while time.time() < end_time:
            status_res = call_service(node, status_client, GetProcessStatus.Request(), timeout_sec=5.0)
            if status_res is not None and getattr(status_res, field):
                running_list.append(process_name)
                break
            time.sleep(0.5)
    # 最后查询状态
    final_status = call_service(node, status_client, GetProcessStatus.Request(), timeout_sec=10.0)
    node.get_logger().info(
        f"start summary: {'; '.join(start_summary)} | "
        f"running={running_list if running_list else []} | "
        f"status={status_to_dict(final_status)}"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()