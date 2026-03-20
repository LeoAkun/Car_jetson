#!/usr/bin/env python3
"""
测试进程管理器服务接口（启动/关闭/查询状态）

用一个最小ROS2客户端节点直接调用：
1) /process_manager/start_process
2) /process_manager/shutdown_process
3) /process_manager/get_status
"""

import sys
import time

import rclpy
from rclpy.node import Node

from multi_map_navigation_msgs.srv import StartProcess, ShutdownProcess, GetProcessStatus


def _running_field_name(process_name: str) -> str:
    # 对应 GetProcessStatus.srv 返回字段名
    mapping = {
        "re_localization": "re_localization_running",
        "liosam": "liosam_running",
        "nav2_init_pose": "nav2_init_pose_running",
        "navigation2": "navigation2_running",
    }
    return mapping[process_name]


def _wait_for_running(node: Node, status_client, process_name: str, want_running: bool, timeout_sec: float) -> bool:
    end_time = time.time() + timeout_sec
    req = GetProcessStatus.Request()
    field = _running_field_name(process_name)

    while time.time() < end_time:
        future = status_client.call_async(req)
        completed = rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not completed:
            continue

        res = future.result()
        if res is not None and getattr(res, field) == want_running:
            return True
        time.sleep(0.5)
    return False


def _call_service(node: Node, client, request, timeout_sec: float):
    future = client.call_async(request)
    completed = rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not completed:
        return None
    return future.result()


def main():
    rclpy.init(args=None)
    node = Node("test_process_client")

    start_client = node.create_client(StartProcess, "/process_manager/start_process")
    shutdown_client = node.create_client(ShutdownProcess, "/process_manager/shutdown_process")
    status_client = node.create_client(GetProcessStatus, "/process_manager/get_status")

    for client, name in (
        (start_client, "start_process"),
        (shutdown_client, "shutdown_process"),
        (status_client, "get_status"),
    ):
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f"等待服务可用: {name}")

    shutdown_requested = "--shutdown" in sys.argv[1:]

    # 默认用 map1；也可以用命令行覆盖：python3 .../test_process_client.py map2 [--shutdown]
    map_name = "map1"
    for arg in sys.argv[1:]:
        if not arg.startswith("--"):
            map_name = arg
            break
    node.get_logger().info(f"测试使用地图: {map_name}")

    # 先启动所有堆栈进程
    start_plan = [
        ("re_localization", map_name),
        ("nav2_init_pose", map_name),
        ("liosam", ""),
        ("navigation2", map_name),
    ]

    running_list = []
    for process_name, req_map in start_plan:
        node.get_logger().info(f"请求启动: {process_name}")

        req = StartProcess.Request()
        req.process_name = process_name
        req.map_name = req_map

        res = _call_service(node, start_client, req, timeout_sec=15.0)
        if res is None:
            node.get_logger().error(f"启动服务未返回: {process_name}")
            continue

        node.get_logger().info(f"启动结果: success={res.success}, message={res.message}")
        if not res.success:
            continue

        # 通过查询状态确认“进程管理器认为它仍在运行”
        if _wait_for_running(node, status_client, process_name, True, timeout_sec=120.0):
            running_list.append(process_name)
            node.get_logger().info(f"状态确认：{process_name} 正在运行")
        else:
            node.get_logger().error(f"状态确认失败：{process_name} 未在限定时间内进入运行状态")

    # 打印一次完整状态
    node.get_logger().info("查询当前进程状态...")
    status_res = _call_service(node, status_client, GetProcessStatus.Request(), timeout_sec=10.0)
    if status_res is not None:
        node.get_logger().info(
            "status: "
            f"re_localization={status_res.re_localization_running}, "
            f"liosam={status_res.liosam_running}, "
            f"nav2_init_pose={status_res.nav2_init_pose_running}, "
            f"navigation2={status_res.navigation2_running}"
        )

    if shutdown_requested:
        # 再关闭所有“确认正在运行”的进程
        node.get_logger().info("请求关闭进程（按反向顺序）...")
        for process_name in reversed(running_list):
            node.get_logger().info(f"请求关闭: {process_name}")

            req = ShutdownProcess.Request()
            req.process_name = process_name
            res = _call_service(node, shutdown_client, req, timeout_sec=15.0)
            if res is None:
                node.get_logger().error(f"关闭服务未返回: {process_name}")
                continue

            node.get_logger().info(f"关闭结果: success={res.success}, message={res.message}")

            _wait_for_running(node, status_client, process_name, False, timeout_sec=60.0)
    else:
        node.get_logger().info("未请求关闭进程：进程将保持运行。")

    node.get_logger().info("测试结束。")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()