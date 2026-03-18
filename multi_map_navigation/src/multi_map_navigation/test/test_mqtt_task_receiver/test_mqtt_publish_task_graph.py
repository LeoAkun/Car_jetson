#!/usr/bin/env python3
"""
MQTT TaskGraph 复杂图测试发布器（含地图切换点 type=4）

用于向 MQTT 代理发布包含地图切换点的复杂 TaskGraph 数据。
"""

import paho.mqtt.client as mqtt
import json
import argparse
import time
import sys
from datetime import datetime

def now():
    """返回当前时间ISO格式字符串"""
    return datetime.now().strftime("%Y-%m-%dT%H:%M:%S")

def create_complex_task_graph_with_map_switch():
    """
    创建复杂 TaskGraph，包含地图切换点
    图结构如下：
        西门A(起点) --2.7-- 红绿灯B --3.1-- 主路D --2.0-- 电梯口F(type=4,切换到二楼)
            |                                  ^            |
            +--4.8-- 岔路C ----5.2(不可用)----+            |
            |                                  |            |
            +--6.0-- 充电桩E --------7.5-------+            |
                                                        二楼主路G(切换后终点)
    """
    task_graph = {
        "points": [
            {
                "id": 101,
                "map_id": 10,
                "map_name": "map1",
                "name": "A",
                "gps_lat": 30.761900,
                "gps_lng": 103.981600,
                "gps_heading": 90,
                "slam_x": 0.0,
                "slam_y": 0.0,
                "slam_yaw": 0.0,
                "type": 1,    # 普通导航点
                "created_at": now(),
                "updated_at": now()
            },
            {
                "id": 102,
                "map_id": 10,
                "map_name": "map1",
                "name": "B_traffic",
                "gps_lat": 30.762020,
                "gps_lng": 103.981720,
                "gps_heading": 90,
                "slam_x": 2.5,
                "slam_y": 1.2,
                "slam_yaw": 0.3,
                "type": 2,    # 红绿灯
                "created_at": now(),
                "updated_at": now()
            },
            {
                "id": 103,
                "map_id": 10,
                "map_name": "map1",
                "name": "C",
                "gps_lat": 30.761990,
                "gps_lng": 103.981760,
                "gps_heading": 180,
                "slam_x": 1.8,
                "slam_y": -3.2,
                "slam_yaw": -0.5,
                "type": 1,    # 普通导航点
                "created_at": now(),
                "updated_at": now()
            },
            {
                "id": 104,
                "map_id": 10,
                "map_name": "map1",
                "name": "D",
                "gps_lat": 30.762100,
                "gps_lng": 103.981850,
                "gps_heading": 90,
                "slam_x": 8.2,
                "slam_y": 2.1,
                "slam_yaw": 0.8,
                "type": 1,    # 普通导航点
                "created_at": now(),
                "updated_at": now()
            },
            {
                "id": 105,
                "map_id": 10,
                "map_name": "map1",
                "name": "E_charge",
                "gps_lat": 30.761950,
                "gps_lng": 103.981800,
                "gps_heading": 270,
                "slam_x": 4.3,
                "slam_y": -5.0,
                "slam_yaw": -1.0,
                "type": 3,    # 充电桩
                "created_at": now(),
                "updated_at": now()
            },
            {
                "id": 106,
                "map_id": 10,
                "map_name": "map1",
                "name": "M",
                "gps_lat": 30.762150,
                "gps_lng": 103.981900,
                "gps_heading": 90,
                "slam_x": 10.0,
                "slam_y": 3.0,
                "slam_yaw": 1.57,
                "type": 4,    # 地图切换点
                "next_map_name": "map2",
                "next_slam_x": 2.0,
                "next_slam_y": 1.0,
                "next_slam_yaw": 0.0,
                "created_at": now(),
                "updated_at": now()
            },
            {
                "id": 201,
                "map_id": 20,
                "map_name": "map2",
                "name": "F",
                "gps_lat": 30.762250,
                "gps_lng": 103.982000,
                "gps_heading": 90,
                "slam_x": 2.0,
                "slam_y": 1.0,
                "slam_yaw": 0.0,
                "type": 1,    # 普通导航点（切换后终点）
                "created_at": now(),
                "updated_at": now()
            }
        ],
        "edges": [
            {
                "id": 201,
                "node_a": 101,
                "node_b": 102,
                "weight": 2.7,       # 距离（米）
                "is_active": True
            },
            {
                "id": 202,
                "node_a": 102,
                "node_b": 104,
                "weight": 3.1,       # 距离（米）
                "is_active": True
            },
            {
                "id": 203,
                "node_a": 101,
                "node_b": 103,
                "weight": 4.8,
                "is_active": True
            },
            {
                "id": 204,
                "node_a": 103,
                "node_b": 104,
                "weight": 5.2,
                "is_active": False   # 不可用边（模拟障碍）
            },
            {
                "id": 205,
                "node_a": 101,
                "node_b": 105,
                "weight": 6.0,
                "is_active": True
            },
            {
                "id": 206,
                "node_a": 105,
                "node_b": 104,
                "weight": 7.5,
                "is_active": True
            },
            {
                "id": 207,
                "node_a": 104,
                "node_b": 106,
                "weight": 2.0,
                "is_active": True
            },
            {
                "id": 208,
                "node_a": 106,
                "node_b": 201,
                "weight": 5.0,
                "is_active": True
            }
        ]
    }
    return task_graph

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ 已连接到 MQTT 代理")
    else:
        print(f"❌ 连接失败，返回码: {rc}")
        sys.exit(1)

def on_publish(client, userdata, mid):
    print(f"📤 消息已发布 (mid: {mid})")

def main():
    parser = argparse.ArgumentParser(description='MQTT TaskGraph 复杂图测试发布器（含地图切换点）')
    parser.add_argument('--broker', type=str, default='localhost', help='MQTT 代理地址')
    parser.add_argument('--port', type=int, default=1883, help='MQTT 代理端口')
    parser.add_argument('--topic', type=str, default='prod/ctrl/vehicle/LS1234567890/task/start', help='MQTT 主题')
    parser.add_argument('--username', type=str, default='', help='MQTT 用户名')
    parser.add_argument('--password', type=str, default='', help='MQTT 密码')
    args = parser.parse_args()

    # 创建 MQTT 客户端
    client = mqtt.Client(client_id='test_complex_with_map_switch')
    client.on_connect = on_connect
    client.on_publish = on_publish

    if args.username and args.password:
        client.username_pw_set(args.username, args.password)

    print(f"🔌 正在连接到 {args.broker}:{args.port}...")
    try:
        client.connect(args.broker, args.port, keepalive=60)
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        sys.exit(1)

    client.loop_start()
    time.sleep(1)  # 等待连接建立

    task_graph = create_complex_task_graph_with_map_switch()
    payload = json.dumps(task_graph, ensure_ascii=False, indent=2)

    print(f"\n{'='*60}")
    print(f"📋 测试用例: 复杂园区导航图（含地图切换点）")
    print(f"   节点数: {len(task_graph['points'])}")
    print(f"   边数:   {len(task_graph['edges'])}")
    print(f"   起点:   {task_graph['points'][0]['name']}")
    print(f"   切换点: {task_graph['points'][5]['name']}")
    print(f"   终点:   {task_graph['points'][-1]['name']}")
    print(f"   主题:   {args.topic}")
    print(f"{'='*60}")
    print(f"📦 JSON 数据:\n{payload}")
    print(f"{'='*60}")

    result = client.publish(args.topic, payload, qos=1)
    result.wait_for_publish()
    print(f"✅ 复杂图（含地图切换点）发布成功!\n")

    time.sleep(1)
    client.loop_stop()
    client.disconnect()
    print("🏁 测试完成，已断开连接")

if __name__ == '__main__':
    main()