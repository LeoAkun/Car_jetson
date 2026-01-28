#!/usr/bin/env python3
"""
测试脚本 - 验证新API格式的JSON解析
"""

import json
import math

def test_json_parsing():
    """测试JSON解析功能"""

    # 测试数据
    test_data = [
        {
            "name": "起点",
            "lng": 116.397428,
            "lat": 39.909230,
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "id": 1,
            "map_name": "floor1",
            "type": 0
        },
        {
            "name": "地图切换点",
            "lng": 116.397500,
            "lat": 39.909300,
            "x": 10.0,
            "y": 5.0,
            "yaw": 1.57,
            "id": 2,
            "map_name": "floor1",
            "next_map_name": "floor2",
            "next_x": "0.5",  # 测试字符串类型
            "next_y": "0.5",
            "next_yaw": "0.0",
            "type": 1
        },
        {
            "name": "终点",
            "lng": 116.397600,
            "lat": 39.909400,
            "x": 15.0,
            "y": 10.0,
            "yaw": 0.0,
            "id": 3,
            "map_name": "floor2",
            "type": 0
        }
    ]

    print("=" * 60)
    print("测试新API格式的JSON解析")
    print("=" * 60)

    for i, waypoint in enumerate(test_data, 1):
        print(f"\n航点 {i}: {waypoint['name']}")
        print(f"  ID: {waypoint['id']}")
        print(f"  GPS: ({waypoint['lng']}, {waypoint['lat']})")
        print(f"  坐标: ({waypoint['x']}, {waypoint['y']})")
        print(f"  偏航角: {waypoint['yaw']} rad = {math.degrees(waypoint['yaw']):.2f} deg")
        print(f"  地图: {waypoint['map_name']}")
        print(f"  类型: {'地图切换点' if waypoint['type'] == 1 else '普通导航点'}")

        if waypoint['type'] == 1:
            print(f"  下一张地图: {waypoint['next_map_name']}")
            print(f"  下一张地图坐标: ({float(waypoint['next_x'])}, {float(waypoint['next_y'])})")
            print(f"  下一张地图偏航角: {float(waypoint['next_yaw'])} rad")

    print("\n" + "=" * 60)
    print("✅ 所有字段解析成功!")
    print("=" * 60)

    # 测试JSON序列化
    json_str = json.dumps(test_data, indent=2, ensure_ascii=False)
    print("\nJSON序列化结果:")
    print(json_str)

    # 测试带task_id的格式
    task_with_id = {
        "task_id": "test_task_001",
        "waypoints": test_data
    }

    print("\n" + "=" * 60)
    print("测试带task_id的格式")
    print("=" * 60)
    print(json.dumps(task_with_id, indent=2, ensure_ascii=False))

def test_angle_conversion():
    """测试角度转换"""
    print("\n" + "=" * 60)
    print("测试角度转换")
    print("=" * 60)

    test_angles = [0, 45, 90, 135, 180, -90, -45]

    for deg in test_angles:
        rad = math.radians(deg)
        back_deg = math.degrees(rad)
        print(f"{deg:6.1f}° = {rad:7.4f} rad = {back_deg:6.1f}°")

    print("\n常用方向:")
    directions = {
        "正东": 0,
        "东北": 45,
        "正北": 90,
        "西北": 135,
        "正西": 180,
        "西南": -135,
        "正南": -90,
        "东南": -45
    }

    for direction, deg in directions.items():
        rad = math.radians(deg)
        print(f"{direction:4s}: {deg:4d}° = {rad:7.4f} rad")

def test_quaternion_conversion():
    """测试四元数转换"""
    try:
        from tf_transformations import quaternion_from_euler, euler_from_quaternion

        print("\n" + "=" * 60)
        print("测试四元数转换")
        print("=" * 60)

        test_yaws = [0, math.pi/4, math.pi/2, math.pi, -math.pi/2]

        for yaw in test_yaws:
            q = quaternion_from_euler(0, 0, yaw)
            back_euler = euler_from_quaternion(q)
            back_yaw = back_euler[2]

            print(f"\nYaw: {yaw:7.4f} rad = {math.degrees(yaw):6.1f}°")
            print(f"  Quaternion: x={q[0]:7.4f}, y={q[1]:7.4f}, z={q[2]:7.4f}, w={q[3]:7.4f}")
            print(f"  Back to Yaw: {back_yaw:7.4f} rad = {math.degrees(back_yaw):6.1f}°")
            print(f"  误差: {abs(yaw - back_yaw):.10f} rad")

        print("\n✅ 四元数转换测试通过!")

    except ImportError:
        print("\n⚠️  警告: tf_transformations 未安装")
        print("请运行: pip3 install transforms3d")

def validate_waypoint(waypoint):
    """验证航点数据的完整性"""
    required_fields = ['name', 'lng', 'lat', 'x', 'y', 'yaw', 'id', 'map_name', 'type']

    errors = []

    # 检查必填字段
    for field in required_fields:
        if field not in waypoint:
            errors.append(f"缺少必填字段: {field}")

    # 检查type字段
    if 'type' in waypoint:
        if not isinstance(waypoint['type'], int):
            errors.append(f"type字段必须是整数，当前类型: {type(waypoint['type'])}")
        elif waypoint['type'] not in [0, 1]:
            errors.append(f"type字段必须是0或1，当前值: {waypoint['type']}")

    # 如果是地图切换点，检查next_*字段
    if waypoint.get('type') == 1:
        next_fields = ['next_map_name', 'next_x', 'next_y', 'next_yaw']
        for field in next_fields:
            if field not in waypoint:
                errors.append(f"地图切换点缺少字段: {field}")

    return errors

def test_validation():
    """测试数据验证"""
    print("\n" + "=" * 60)
    print("测试数据验证")
    print("=" * 60)

    # 测试正确的数据
    valid_waypoint = {
        "name": "test",
        "lng": 116.397428,
        "lat": 39.909230,
        "x": 1.0,
        "y": 2.0,
        "yaw": 0.0,
        "id": 1,
        "map_name": "map1",
        "type": 0
    }

    errors = validate_waypoint(valid_waypoint)
    if errors:
        print("❌ 验证失败:")
        for error in errors:
            print(f"  - {error}")
    else:
        print("✅ 普通航点验证通过")

    # 测试地图切换点
    switch_waypoint = {
        "name": "switch",
        "lng": 116.397500,
        "lat": 39.909300,
        "x": 5.0,
        "y": 3.0,
        "yaw": 1.57,
        "id": 2,
        "map_name": "map1",
        "next_map_name": "map2",
        "next_x": 0.5,
        "next_y": 0.5,
        "next_yaw": 0.0,
        "type": 1
    }

    errors = validate_waypoint(switch_waypoint)
    if errors:
        print("❌ 验证失败:")
        for error in errors:
            print(f"  - {error}")
    else:
        print("✅ 地图切换点验证通过")

    # 测试错误的数据
    invalid_waypoint = {
        "name": "invalid",
        "x": 1.0,
        "y": 2.0,
        "type": 1  # 地图切换点但缺少next_*字段
    }

    errors = validate_waypoint(invalid_waypoint)
    if errors:
        print("\n❌ 检测到错误数据:")
        for error in errors:
            print(f"  - {error}")
    else:
        print("✅ 验证通过")

if __name__ == "__main__":
    try:
        test_json_parsing()
        test_angle_conversion()
        test_quaternion_conversion()
        test_validation()

        print("\n" + "=" * 60)
        print("🎉 所有测试完成!")
        print("=" * 60)

    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
