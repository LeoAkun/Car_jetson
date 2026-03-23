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


PROCESSES = ["re_localization", "nav2_init_pose", "liosam", "navigation2"]


def _status_field(process_name: str) -> str:
    return {
        "re_localization": "re_localization_running",
        "liosam": "liosam_running",
        "nav2_init_pose": "nav2_init_pose_running",
        "navigation2": "navigation2_running",
    }[process_name]


def _call(node: Node, client, request, timeout_sec: float):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    return future.result() if future.done() else None


def main():
    rclpy.init(args=None)
    node = Node("test_process_client")
    double_start_requested = "--double-start" in sys.argv[1:]
    shutdown_status_only = "--shutdown-status-only" in sys.argv[1:]
    status_only = "--status-only" in sys.argv[1:]
    double_start_delay_ms = 0
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

    for client in (
        (start_client, "start_process"),
        (shutdown_client, "shutdown_process"),
        (status_client, "get_status"),
    ):
        while not client[0].wait_for_service(timeout_sec=1.0):
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

    node.get_logger().info(
        "mode="
        + ("status-only" if status_only else "shutdown-status-only" if shutdown_status_only else "start")
        + (", double-start" if double_start_requested else "")
        + (f", delay={double_start_delay_ms}ms" if double_start_requested else "")
    )

    if status_only:
        # 单独测试 get_status 服务
        res = _call(node, status_client, GetProcessStatus.Request(), timeout_sec=10.0)
        node.get_logger().info(
            f"status: {None if res is None else {'re_localization': res.re_localization_running, 'nav2_init_pose': res.nav2_init_pose_running, 'liosam': res.liosam_running, 'navigation2': res.navigation2_running}}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    if shutdown_status_only:
        # 单独测试 shutdown + get_status（关闭前/后各查一次）
        before = _call(node, status_client, GetProcessStatus.Request(), timeout_sec=10.0)
        shutdown_summary = []
        for process_name in PROCESSES:
            req = ShutdownProcess.Request()
            req.process_name = process_name
            res = _call(node, shutdown_client, req, timeout_sec=20.0)
            shutdown_summary.append(
                f"{process_name}:{False if res is None else bool(res.success)}"
            )
        after = _call(node, status_client, GetProcessStatus.Request(), timeout_sec=10.0)
        node.get_logger().info(
            f"shutdown summary: {'; '.join(shutdown_summary)} | "
            f"before={None if before is None else {'re_localization': before.re_localization_running, 'nav2_init_pose': before.nav2_init_pose_running, 'liosam': before.liosam_running, 'navigation2': before.navigation2_running}} | "
            f"after={None if after is None else {'re_localization': after.re_localization_running, 'nav2_init_pose': after.nav2_init_pose_running, 'liosam': after.liosam_running, 'navigation2': after.navigation2_running}}"
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
            if double_start_delay_ms > 0:
                time.sleep(double_start_delay_ms / 1000.0)
            future2 = start_client.call_async(req2)
            rclpy.spin_until_future_complete(node, future1, timeout_sec=20.0)
            rclpy.spin_until_future_complete(node, future2, timeout_sec=20.0)
            res1 = future1.result() if future1.done() else None
            res2 = future2.result() if future2.done() else None
            first_ok = (res1 is not None and bool(res1.success))
            second_ok = (res2 is not None and bool(res2.success))
            start_summary.append(f"{process_name}[1st={first_ok};2nd={second_ok}]")
            if not first_ok:
                continue
        else:
            # 普通单次启动
            req = StartProcess.Request()
            req.process_name = process_name
            req.map_name = req_map
            res = _call(node, start_client, req, timeout_sec=15.0)
            ok = (res is not None and bool(res.success))
            start_summary.append(f"{process_name}[{ok}]")
            if not ok:
                continue

        end_time = time.time() + 120.0
        field = _status_field(process_name)
        # 轮询状态服务，确认进程真的进入运行态
        while time.time() < end_time:
            status_res = _call(node, status_client, GetProcessStatus.Request(), timeout_sec=5.0)
            if status_res is not None and getattr(status_res, field):
                running_list.append(process_name)
                break
            time.sleep(0.5)

    final_status = _call(node, status_client, GetProcessStatus.Request(), timeout_sec=10.0)
    node.get_logger().info(
        f"start summary: {'; '.join(start_summary)} | "
        f"running={running_list if running_list else []} | "
        f"status={None if final_status is None else {'re_localization': final_status.re_localization_running, 'nav2_init_pose': final_status.nav2_init_pose_running, 'liosam': final_status.liosam_running, 'navigation2': final_status.navigation2_running}}"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()