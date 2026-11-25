#!/usr/bin/env python3
"""Resolve catalog waypoints and dispatch a StartEndGraph task."""

import argparse
import heapq
import math
from pathlib import Path
import sys
import time

import yaml


DEFAULT_CATALOG = Path(__file__).with_name("waypoint_catalog.yaml")
SAFE_TRANSIT_TYPES = {1, 4}


def load_catalog(path=DEFAULT_CATALOG):
    with Path(path).open(encoding="utf-8") as stream:
        catalog = yaml.safe_load(stream)
    validate_catalog(catalog)
    return catalog


def validate_catalog(catalog):
    maps = catalog.get("maps", {})
    if not maps:
        raise ValueError("航点目录中没有地图")
    for map_name, map_data in maps.items():
        points = map_data.get("points", [])
        ids = [int(point["id"]) for point in points]
        names = [str(point["name"]) for point in points]
        if len(ids) != len(set(ids)):
            raise ValueError(f"地图 {map_name} 存在重复航点编号")
        if len(names) != len(set(names)):
            raise ValueError(f"地图 {map_name} 存在重复航点名称")
        known_ids = set(ids)
        for start_id, end_id, weight in map_data.get("edges", []):
            if int(start_id) not in known_ids or int(end_id) not in known_ids:
                raise ValueError(f"地图 {map_name} 的边引用了未知航点")
            if float(weight) <= 0:
                raise ValueError(f"地图 {map_name} 存在非正数边权重")


def point_direction(yaw):
    degrees = (math.degrees(float(yaw)) + 360.0) % 360.0
    directions = [
        ("东", "->"), ("东北", "/^"), ("北", "^"), ("西北", "^\\"),
        ("西", "<-"), ("西南", "\\v"), ("南", "v"), ("东南", "v/"),
    ]
    index = int((degrees + 22.5) // 45.0) % 8
    label, arrow = directions[index]
    return f"{label} {arrow} ({degrees:.0f} deg)"


def resolve_point(catalog, selector, map_name=None):
    selector_text = str(selector).strip()
    maps = catalog["maps"]
    if map_name is not None and map_name not in maps:
        raise ValueError(f"未知地图: {map_name}")
    selected_maps = {map_name: maps[map_name]} if map_name else maps

    numeric_id = None
    try:
        numeric_id = int(selector_text)
    except ValueError:
        pass

    candidates = []
    for candidate_map, map_data in selected_maps.items():
        for point in map_data["points"]:
            id_match = numeric_id is not None and int(point["id"]) == numeric_id
            name_match = str(point["name"]).casefold() == selector_text.casefold()
            if id_match or name_match:
                candidates.append((candidate_map, point))

    if not candidates and numeric_id is None:
        for candidate_map, map_data in selected_maps.items():
            for point in map_data["points"]:
                if selector_text.casefold() in str(point["name"]).casefold():
                    candidates.append((candidate_map, point))

    if not candidates:
        suffix = f"（地图 {map_name}）" if map_name else ""
        raise ValueError(f"找不到航点 {selector_text}{suffix}")
    if len(candidates) > 1:
        choices = ", ".join(
            f"{candidate_map}:{point['id']}:{point['name']}"
            for candidate_map, point in candidates
        )
        raise ValueError(f"航点不唯一，请指定地图或完整名称: {choices}")
    return candidates[0]


def safe_shortest_path(map_data, start_id, goal_id):
    points = {int(point["id"]): point for point in map_data["points"]}
    start_id = int(start_id)
    goal_id = int(goal_id)
    if int(points[start_id].get("type", 0)) not in SAFE_TRANSIT_TYPES:
        raise ValueError("起点类型尚不支持自动导航")

    allowed_ids = {
        point_id for point_id, point in points.items()
        if int(point.get("type", 0)) in SAFE_TRANSIT_TYPES or point_id == goal_id
    }
    graph = {point_id: [] for point_id in allowed_ids}
    for raw_start, raw_end, raw_weight in map_data["edges"]:
        edge_start, edge_end = int(raw_start), int(raw_end)
        if edge_start in allowed_ids and edge_end in allowed_ids:
            weight = float(raw_weight)
            graph[edge_start].append((edge_end, weight))
            graph[edge_end].append((edge_start, weight))

    queue = [(0.0, start_id, [])]
    best = {}
    while queue:
        cost, point_id, prefix = heapq.heappop(queue)
        if cost >= best.get(point_id, math.inf):
            continue
        best[point_id] = cost
        path = prefix + [point_id]
        if point_id == goal_id:
            return path, cost, allowed_ids
        for next_id, weight in graph.get(point_id, []):
            heapq.heappush(queue, (cost + weight, next_id, path))
    raise ValueError("起点和目标之间没有安全的同地图路线")


def haversine_m(lat1, lng1, lat2, lng2):
    radius = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lng = math.radians(lng2 - lng1)
    value = (
        math.sin(delta_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lng / 2.0) ** 2
    )
    return 2.0 * radius * math.atan2(math.sqrt(value), math.sqrt(1.0 - value))


def nearest_start(map_data, latitude, longitude):
    candidates = [
        point for point in map_data["points"]
        if int(point.get("type", 0)) in SAFE_TRANSIT_TYPES
    ]
    return min(
        (
            haversine_m(latitude, longitude, float(point["lat"]), float(point["lng"])),
            point,
        )
        for point in candidates
    )


def describe_point(map_name, point):
    return (
        f"{map_name} / {point['id']} / {point['name']} / "
        f"x={point['x']:.2f}, y={point['y']:.2f}, yaw={point['yaw']:.2f} / "
        f"{point_direction(point['yaw'])}"
    )


def wait_for_gps(node, timeout_sec):
    from sensor_msgs.msg import NavSatFix
    import rclpy

    latest = []
    subscription = node.create_subscription(
        NavSatFix,
        "/sensing/gnss/pose_with_covariance",
        lambda message: latest.append(message),
        10,
    )
    deadline = time.monotonic() + timeout_sec
    while not latest and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(subscription)
    if not latest:
        raise RuntimeError("未收到GNSS，无法自动选择起点；请显式指定 --start")
    return float(latest[-1].latitude), float(latest[-1].longitude)


def build_ros_graph(node, map_name, map_data, start_point, goal_point, allowed_ids):
    from multi_map_navigation_msgs.msg import Edge, StartEndGraph, Waypoint

    def make_waypoint(point):
        message = Waypoint()
        message.name = str(point["name"])
        message.id = int(point["id"])
        message.map_name = map_name
        message.lng = float(point["lng"])
        message.lat = float(point["lat"])
        message.x = float(point["x"])
        message.y = float(point["y"])
        message.yaw = float(point["yaw"])
        message.type = int(point.get("type", 1))
        message.tolerance = float(point.get("tolerance", 0.5))
        message.next_map_name = ""
        message.next_x = 0.0
        message.next_y = 0.0
        message.next_yaw = 0.0
        return message

    graph = StartEndGraph()
    graph.header.stamp = node.get_clock().now().to_msg()
    graph.header.frame_id = "map"
    point_messages = {
        int(point["id"]): make_waypoint(point)
        for point in map_data["points"]
        if int(point["id"]) in allowed_ids
    }
    graph.nodes = list(point_messages.values())
    graph.start = point_messages[int(start_point["id"])]
    graph.end = point_messages[int(goal_point["id"])]
    for raw_start, raw_end, raw_weight in map_data["edges"]:
        edge_start, edge_end = int(raw_start), int(raw_end)
        if edge_start not in allowed_ids or edge_end not in allowed_ids:
            continue
        edge = Edge()
        edge.start_node_id = edge_start
        edge.end_node_id = edge_end
        edge.weight = float(raw_weight)
        graph.edges.append(edge)
    return graph


def dispatch(args, catalog):
    import rclpy
    from std_msgs.msg import String
    from multi_map_navigation_msgs.msg import StartEndGraph

    goal_map, goal_point = resolve_point(catalog, args.goal, args.map)
    map_data = catalog["maps"][goal_map]
    if not map_data.get("dispatch_ready", False):
        raise ValueError(f"地图 {goal_map} 尚未允许自动派单")

    rclpy.init()
    node = rclpy.create_node("catalog_navigation_dispatcher")
    robot_states = []
    node.create_subscription(String, "/robot_state", lambda msg: robot_states.append(msg.data), 10)
    publisher = node.create_publisher(StartEndGraph, "/start_end_graph", 10)
    try:
        if args.start:
            start_map, start_point = resolve_point(catalog, args.start, goal_map)
            if start_map != goal_map:
                raise ValueError("当前工具禁止跨地图自动派单")
            start_distance = None
        else:
            latitude, longitude = wait_for_gps(node, args.gps_timeout)
            start_distance, start_point = nearest_start(map_data, latitude, longitude)
            if start_distance > args.max_start_distance:
                raise ValueError(
                    f"最近起点仍相距 {start_distance:.1f}m，超过安全限制；请指定 --start"
                )

        path, weight, allowed_ids = safe_shortest_path(
            map_data, start_point["id"], goal_point["id"])
        points_by_id = {int(point["id"]): point for point in map_data["points"]}
        route = " -> ".join(f"{point_id}:{points_by_id[point_id]['name']}" for point_id in path)
        print(f"起点: {describe_point(goal_map, start_point)}")
        if start_distance is not None:
            print(f"GNSS自动选择起点，距离 {start_distance:.1f}m")
        print(f"目标: {describe_point(goal_map, goal_point)}")
        print(f"规划: {route}（图权重 {weight:.1f}）")

        if args.dry_run:
            return 0

        deadline = time.monotonic() + args.subscriber_timeout
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("/start_end_graph 没有订阅者，route_planner 尚未就绪")
        if robot_states and robot_states[-1] == "running" and not args.force:
            raise RuntimeError("已有导航任务处于 running；如确认覆盖请使用 --force")

        graph = build_ros_graph(
            node, goal_map, map_data, start_point, goal_point, allowed_ids)
        for _ in range(3):
            publisher.publish(graph)
            rclpy.spin_once(node, timeout_sec=0.5)

        deadline = time.monotonic() + args.accept_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if robot_states and robot_states[-1] == "running":
                print("任务已被 navigation_manager 接受，robot_state=running")
                return 0
        raise RuntimeError("任务已发布，但未在规定时间内确认 robot_state=running")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def print_catalog(catalog, map_name=None):
    maps = catalog["maps"]
    selected = {map_name: maps[map_name]} if map_name else maps
    for current_map, map_data in selected.items():
        print(f"[{current_map}] {map_data['source']}")
        for point in sorted(map_data["points"], key=lambda item: int(item["id"])):
            print("  " + describe_point(current_map, point))


def build_parser():
    parser = argparse.ArgumentParser(description="按名称或编号查询并派发导航任务")
    parser.add_argument("--catalog", default=str(DEFAULT_CATALOG))
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="列出航点")
    list_parser.add_argument("--map")

    show_parser = subparsers.add_parser("show", help="查看单个航点")
    show_parser.add_argument("point")
    show_parser.add_argument("--map")

    plan_parser = subparsers.add_parser("plan", help="只计算目录内路线")
    plan_parser.add_argument("goal")
    plan_parser.add_argument("--start", required=True)
    plan_parser.add_argument("--map")

    dispatch_parser = subparsers.add_parser("dispatch", help="派发ROS导航任务")
    dispatch_parser.add_argument("goal")
    dispatch_parser.add_argument("--start")
    dispatch_parser.add_argument("--map")
    dispatch_parser.add_argument("--gps-timeout", type=float, default=5.0)
    dispatch_parser.add_argument("--max-start-distance", type=float, default=150.0)
    dispatch_parser.add_argument("--subscriber-timeout", type=float, default=30.0)
    dispatch_parser.add_argument("--accept-timeout", type=float, default=20.0)
    dispatch_parser.add_argument("--force", action="store_true")
    dispatch_parser.add_argument("--dry-run", action="store_true")
    return parser


def main():
    args = build_parser().parse_args()
    try:
        catalog = load_catalog(args.catalog)
        if args.command == "list":
            print_catalog(catalog, args.map)
            return 0
        if args.command == "show":
            map_name, point = resolve_point(catalog, args.point, args.map)
            print(describe_point(map_name, point))
            return 0
        if args.command == "plan":
            goal_map, goal = resolve_point(catalog, args.goal, args.map)
            start_map, start = resolve_point(catalog, args.start, goal_map)
            if start_map != goal_map:
                raise ValueError("当前工具禁止跨地图自动派单")
            map_data = catalog["maps"][goal_map]
            path, weight, _ = safe_shortest_path(map_data, start["id"], goal["id"])
            points = {int(point["id"]): point for point in map_data["points"]}
            print(" -> ".join(f"{point_id}:{points[point_id]['name']}" for point_id in path))
            print(f"图权重: {weight:.1f}")
            return 0
        return dispatch(args, catalog)
    except (RuntimeError, ValueError, OSError, yaml.YAMLError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
