# 快速参考指南 - 新API格式

## 快速对比

### 航点类型判断

```python
# 旧方式
if waypoint.waypoint_type == 'map_switch':
    # 地图切换点
elif waypoint.waypoint_type == 'normal':
    # 普通点
elif waypoint.waypoint_type == 'final':
    # 最终点

# 新方式
if waypoint.type == 1:
    # 地图切换点
else:  # waypoint.type == 0
    # 普通点（最后一个点自动识别为终点）
```

### 获取航点坐标

```python
# 旧方式
x = waypoint.pose.pose.position.x
y = waypoint.pose.pose.position.y
# 需要从四元数转换yaw
from tf_transformations import euler_from_quaternion
orientation = waypoint.pose.pose.orientation
yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]

# 新方式
x = waypoint.x
y = waypoint.y
yaw = waypoint.yaw  # 直接获取
```

### 地图切换点信息

```python
# 旧方式
next_map = waypoint.next_map_name
# 没有下一张地图的坐标信息

# 新方式
next_map = waypoint.next_map_name
next_x = waypoint.next_x
next_y = waypoint.next_y
next_yaw = waypoint.next_yaw
```

## JSON示例模板

### 单地图导航任务

```json
[
  {
    "name": "起点",
    "lng": 116.397428,
    "lat": 39.909230,
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0,
    "id": 1,
    "map_name": "warehouse_floor1",
    "type": 0
  },
  {
    "name": "货架A",
    "lng": 116.397500,
    "lat": 39.909300,
    "x": 5.0,
    "y": 3.0,
    "yaw": 1.57,
    "id": 2,
    "map_name": "warehouse_floor1",
    "type": 0
  },
  {
    "name": "终点",
    "lng": 116.397600,
    "lat": 39.909400,
    "x": 10.0,
    "y": 8.0,
    "yaw": 0.0,
    "id": 3,
    "map_name": "warehouse_floor1",
    "type": 0
  }
]
```

### 多地图切换任务

```json
[
  {
    "name": "1楼起点",
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
    "name": "电梯口",
    "lng": 116.397500,
    "lat": 39.909300,
    "x": 10.0,
    "y": 5.0,
    "yaw": 1.57,
    "id": 2,
    "map_name": "floor1",
    "next_map_name": "floor2",
    "next_x": 0.5,
    "next_y": 0.5,
    "next_yaw": 0.0,
    "type": 1
  },
  {
    "name": "2楼目标点",
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
```

## 常见问题

### Q1: type字段必须是整数吗？
**A:** 是的，必须是整数。0表示普通导航点，1表示地图切换点。

### Q2: next_x等字段在JSON中是字符串可以吗？
**A:** 可以，代码会自动转换。但建议直接使用数字类型。

### Q3: yaw角度的单位是什么？
**A:** 弧度制。0表示正东，π/2表示正北，π表示正西，-π/2表示正南。

### Q4: 如何计算yaw角？
```python
import math

# 从角度转弧度
yaw_rad = math.radians(90)  # 90度 = π/2弧度

# 从弧度转角度
yaw_deg = math.degrees(1.57)  # 1.57弧度 ≈ 90度
```

### Q5: GPS坐标是必填的吗？
**A:** 是的，但如果没有GPS可以填0。主要用于导航的是x, y, yaw字段。

### Q6: 如何确定next_x, next_y, next_yaw的值？
**A:** 这些值应该是当前点在下一张地图坐标系下的对应坐标。通常需要通过地图对齐或标定获得。

### Q7: 可以不带task_id吗？
**A:** 可以，支持直接发送数组格式。系统会自动生成task_id。

## 测试命令

### 使用mosquitto测试

```bash
# 测试单点导航
mosquitto_pub -h localhost -t robot/task -m '[
  {
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
]'

# 测试地图切换
mosquitto_pub -h localhost -t robot/task -m '[
  {
    "name": "point1",
    "lng": 116.397428,
    "lat": 39.909230,
    "x": 1.0,
    "y": 2.0,
    "yaw": 0.0,
    "id": 1,
    "map_name": "map1",
    "type": 0
  },
  {
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
]'
```

### 监控状态

```bash
# 监控航点列表
ros2 topic echo /waypoint_list

# 监控机器人状态
ros2 topic echo /robot_state

# 监控地图切换
ros2 topic echo /trigger_map_switch
ros2 topic echo /map_switch_complete
```

## Python客户端示例

```python
#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import math

def create_waypoint(name, lng, lat, x, y, yaw_deg, id, map_name,
                   is_switch=False, next_map="", next_x=0, next_y=0, next_yaw_deg=0):
    """创建航点"""
    waypoint = {
        "name": name,
        "lng": lng,
        "lat": lat,
        "x": x,
        "y": y,
        "yaw": math.radians(yaw_deg),  # 角度转弧度
        "id": id,
        "map_name": map_name,
        "type": 1 if is_switch else 0
    }

    if is_switch:
        waypoint["next_map_name"] = next_map
        waypoint["next_x"] = next_x
        waypoint["next_y"] = next_y
        waypoint["next_yaw"] = math.radians(next_yaw_deg)

    return waypoint

def send_task(broker, port, topic, waypoints):
    """发送任务到MQTT"""
    client = mqtt.Client()
    client.connect(broker, port, 60)

    payload = json.dumps(waypoints, indent=2)
    print(f"发送任务:\n{payload}")

    client.publish(topic, payload)
    client.disconnect()
    print("任务发送成功!")

# 使用示例
if __name__ == "__main__":
    waypoints = [
        create_waypoint("起点", 116.397428, 39.909230, 0, 0, 0, 1, "floor1"),
        create_waypoint("中间点", 116.397500, 39.909300, 5, 3, 90, 2, "floor1"),
        create_waypoint("切换点", 116.397550, 39.909350, 10, 5, 90, 3, "floor1",
                       is_switch=True, next_map="floor2", next_x=0.5, next_y=0.5, next_yaw_deg=0),
        create_waypoint("终点", 116.397600, 39.909400, 15, 10, 0, 4, "floor2")
    ]

    send_task("localhost", 1883, "robot/task", waypoints)
```

## 字段验证清单

发送任务前请检查：

- [ ] 所有必填字段都已填写（name, lng, lat, x, y, yaw, id, map_name, type）
- [ ] type字段是整数（0或1）
- [ ] yaw角度使用弧度制
- [ ] 地图切换点（type=1）包含所有next_*字段
- [ ] id字段在任务中唯一
- [ ] map_name与实际地图文件名匹配
- [ ] 坐标值在地图范围内

## 调试技巧

### 1. 查看解析后的消息

```bash
ros2 topic echo /waypoint_list --once
```

### 2. 检查日志

```bash
ros2 run multi_map_navigation mqtt_task_receiver.py
# 查看解析日志
```

### 3. 验证坐标转换

```python
from tf_transformations import quaternion_from_euler
import math

yaw = math.radians(90)  # 90度
q = quaternion_from_euler(0, 0, yaw)
print(f"Yaw: {yaw} rad = {math.degrees(yaw)} deg")
print(f"Quaternion: x={q[0]}, y={q[1]}, z={q[2]}, w={q[3]}")
```

## 性能优化建议

1. **批量发送**: 一次发送完整的航点列表，而不是逐个发送
2. **合理设置容差**: tolerance字段根据实际需求设置，过小会导致频繁调整
3. **GPS坐标**: 如果不需要GPS记录，可以填0以减少数据量
4. **地图切换**: 确保next_*坐标准确，避免切换后重定位失败

## 升级检查清单

从旧版本升级时：

- [ ] 重新编译ROS2包
- [ ] 更新MQTT客户端代码
- [ ] 修改JSON格式
- [ ] 测试单地图导航
- [ ] 测试多地图切换
- [ ] 验证GPS坐标记录
- [ ] 检查日志无错误
- [ ] 性能测试通过
