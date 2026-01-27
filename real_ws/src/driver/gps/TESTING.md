# GNSS Converter 测试指南

## 方式一：自动测试（推荐）

### 1. 安装依赖
```bash
# 安装 socat（需要 sudo）
sudo apt-get install socat

# pyserial 已经安装
```

### 2. 运行快速测试
```bash
cd /home/cplus/Desktop/yunle_sensors
./src/gnss_converter/scripts/quick_test.sh
```

这个脚本会：
- ✅ 创建虚拟串口对 (/tmp/vgps0, /tmp/vgps1)
- ✅ 启动 GNSS Converter 节点
- ✅ 启动数据模拟器发送 NMEA 数据
- ✅ 验证 ROS2 话题输出
- ✅ 自动清理

---

## 方式二：手动测试

### 步骤 1: 安装 socat
```bash
sudo apt-get install socat
```

### 步骤 2: 创建虚拟串口对

打开**终端 1**：
```bash
socat -d -d pty,raw,echo=0,link=/tmp/vgps0 pty,raw,echo=0,link=/tmp/vgps1
```

保持这个终端运行！

### 步骤 3: 启动 GNSS Converter 节点

打开**终端 2**：
```bash
cd /home/cplus/Desktop/yunle_sensors
source install/setup.bash
ros2 run gnss_converter gnss_converter_node --ros-args -p serial_port:=/tmp/vgps1
```

你应该看到：
```
[INFO] [gnss_converter_node]: Serial port /tmp/vgps1 opened successfully
[INFO] [gnss_converter_node]: GNSS Converter Node started
```

### 步骤 4: 发送模拟数据

打开**终端 3**：
```bash
cd /home/cplus/Desktop/yunle_sensors
python3 src/gnss_converter/scripts/simulate_gnss.py /tmp/vgps0
```

你应该看到：
```
✓ 已打开串口: /tmp/vgps0 @ 115200 baud
✓ 发送频率: 1.0 Hz
✓ 开始发送 NMEA 数据...
[0001] 已发送: $GNGGA,063622.00,3046.05670,N,10359.01462,E,2,12,0.52,514.1,M,-30.0,M,,*63
```

在**终端 2**中，你应该看到：
```
[INFO] [gnss_converter_node]: Origin set to: lat=30.76760952, lon=103.98357717, alt=514.10
[INFO] [gnss_converter_node]: Position - Lat: 30.76761183, Lon: 103.98357617, Alt: 514.20 | ENU: x=-0.009, y=0.026, z=0.100 | Sats: 12, HDOP: 0.52
```

### 步骤 5: 查看 ROS2 话题

打开**终端 4**：
```bash
cd /home/cplus/Desktop/yunle_sensors
source install/setup.bash

# 查看话题列表
ros2 topic list

# 查看话题信息
ros2 topic info /sensing/gnss/pose_with_covariance

# 查看话题频率
ros2 topic hz /sensing/gnss/pose_with_covariance

# 查看话题数据
ros2 topic echo /sensing/gnss/pose_with_covariance
```

预期输出：
```yaml
header:
  stamp:
    sec: 1732704123
    nanosec: 456789000
  frame_id: gnss_link
pose:
  pose:
    position:
      x: 0.0  # 第一个位置是原点 (0, 0, 0)
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance:
  - 1.0816  # 基于 HDOP=0.52 计算的协方差
  - 0.0
  - 0.0
  ...
```

---

## 方式三：使用真实 GPS 设备

如果你有真实的 GPS 接收器连接到 /dev/ttyACM0：

```bash
cd /home/cplus/Desktop/yunle_sensors
source install/setup.bash

# 给串口添加权限
sudo chmod 666 /dev/ttyACM0

# 启动节点
ros2 launch gnss_converter gnss_converter.launch.py

# 在另一个终端查看数据
ros2 topic echo /sensing/gnss/pose_with_covariance
```

---

## 验证清单

测试成功的标志：

- ✅ 串口打开成功（无权限错误）
- ✅ 节点输出 "Origin set to..." 消息
- ✅ 节点每秒输出位置信息
- ✅ `/sensing/gnss/pose_with_covariance` 话题存在
- ✅ 话题频率约为 1 Hz
- ✅ 话题数据包含正确的 position (x, y, z)
- ✅ 第一个位置是 (0, 0, 0)（设为原点）
- ✅ 后续位置相对原点有小的偏移

---

## 故障排查

### 问题 1: "Permission denied" 访问串口
```bash
sudo chmod 666 /tmp/vgps1
# 或者对于真实设备
sudo chmod 666 /dev/ttyACM0
```

### 问题 2: 虚拟串口不存在
确保 socat 进程在运行：
```bash
ps aux | grep socat
```

### 问题 3: 没有数据输出
检查模拟器是否正常运行：
```bash
# 直接查看串口数据
cat /tmp/vgps1
```

应该看到持续的 NMEA 消息流。

### 问题 4: 话题不存在
检查节点是否运行：
```bash
ros2 node list
```

应该看到 `/gnss_converter_node`。

---

## 清理

停止所有测试：
```bash
# 停止所有相关进程
pkill -f gnss_converter
pkill -f simulate_gnss
pkill -f socat

# 删除虚拟串口
rm -f /tmp/vgps0 /tmp/vgps1
```

---

## 预期结果示例

当一切正常时，你应该看到类似这样的输出：

**Converter 节点输出：**
```
[INFO] [gnss_converter_node]: Serial port /tmp/vgps1 opened successfully
[INFO] [gnss_converter_node]: GNSS Converter Node started
[INFO] [gnss_converter_node]: Origin set to: lat=30.76760952, lon=103.98357717, alt=514.10
[INFO] [gnss_converter_node]: Position - Lat: 30.76761183, Lon: 103.98357617, Alt: 514.10 | ENU: x=0.000, y=0.000, z=0.00 | Sats: 12, HDOP: 0.52
[INFO] [gnss_converter_node]: Position - Lat: 30.76761183, Lon: 103.98357450, Alt: 514.10 | ENU: x=-0.015, y=0.000, z=0.00 | Sats: 12, HDOP: 0.52
```

**ROS2 话题输出：**
```yaml
header:
  stamp: {sec: 1732704123, nanosec: 456789000}
  frame_id: gnss_link
pose:
  pose:
    position: {x: -0.009, y: 0.026, z: 0.1}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [1.0816, 0.0, 0.0, 0.0, 0.0, 0.0, ...]
```

这表示 GPS 位置相对原点向西移动了 0.9 cm，向北移动了 2.6 cm，高度增加了 10 cm。
