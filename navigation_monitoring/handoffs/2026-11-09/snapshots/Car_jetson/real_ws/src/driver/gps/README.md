# GNSS Converter

ROS2 包，用于将串口 NMEA GPS 数据转换为 ROS2 标准的 `sensor_msgs/NavSatFix` 消息。

## 功能特性

- 从串口读取 NMEA GPS 数据（支持 GNGGA、GPGGA 格式）
- 自动解析经纬度、海拔高度、定位质量等信息
- 发布标准 ROS2 `sensor_msgs/NavSatFix` 消息
- 基于 HDOP 自动计算位置协方差
- 支持多种定位模式（GPS、DGPS、RTK 固定解、RTK 浮点解）

## 依赖项

- ROS2（Humble/Foxy/Galactic）
- serial（串口通信库，已包含在工作空间）
- sensor_msgs

## 编译

```bash
cd /home/cplus/Desktop/yunle_sensors
colcon build --packages-select gnss_converter
source install/setup.bash
```

## 使用方法

### 1. 基本启动

```bash
ros2 launch gnss_converter gnss_converter.launch.py
```

### 2. 自定义串口和波特率

```bash
ros2 launch gnss_converter gnss_converter.launch.py serial_port:=/dev/ttyUSB0 baudrate:=9600
```

### 3. 查看发布的消息

```bash
ros2 topic echo /sensing/gnss/navsat
```

### 4. 查看话题信息

```bash
ros2 topic info /sensing/gnss/navsat
ros2 topic hz /sensing/gnss/navsat
```

## 发布的话题

| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/sensing/gnss/navsat` | `sensor_msgs/NavSatFix` | GPS 位置信息（标准格式） |

## 消息格式

发布到 `/sensing/gnss/navsat` 的消息类型为 `sensor_msgs/NavSatFix`：

```yaml
header:
  stamp: {sec: 1732704123, nanosec: 456789000}
  frame_id: gnss_link
status:
  status: 0              # -1=无定位, 0=未增强GPS, 1=SBAS, 2=GBAS(RTK)
  service: 15            # GPS(1) + GLONASS(2) + Galileo(4) + Compass(8)
latitude: 30.76761167    # 纬度（十进制度数，北纬为正）
longitude: 103.98357700  # 经度（十进制度数，东经为正）
altitude: 514.1          # 海拔高度（米，相对于 WGS84 椭球）
position_covariance:     # 3x3 协方差矩阵
  - 1.0816              # East variance
  - 0.0
  - 0.0
  - 0.0
  - 1.0816              # North variance
  - 0.0
  - 0.0
  - 0.0
  - 2.1632              # Up variance (垂直精度通常较差)
position_covariance_type: 2  # COVARIANCE_TYPE_APPROXIMATED
```

### 状态码说明

**status.status**:
- `-1` - STATUS_NO_FIX（无定位）
- `0` - STATUS_FIX（标准 GPS 定位）
- `1` - STATUS_SBAS_FIX（DGPS 差分定位）
- `2` - STATUS_GBAS_FIX（RTK 定位）

**NMEA 质量码映射**:
- 0 → 无效 → STATUS_NO_FIX
- 1 → GPS → STATUS_FIX
- 2 → DGPS → STATUS_SBAS_FIX
- 4 → RTK 固定解 → STATUS_GBAS_FIX
- 5 → RTK 浮点解 → STATUS_GBAS_FIX

## 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| serial_port | string | /dev/ttyACM0 | GPS 接收器串口设备 |
| baudrate | int | 115200 | 串口波特率 |
| frame_id | string | gnss_link | TF 坐标系 ID |

## NMEA 数据支持

当前支持的 NMEA 语句：
- **$GNGGA**：全球导航卫星系统定位数据（推荐）
- **$GPGGA**：GPS 定位数据

提取的信息包括：
- 经纬度坐标（转换为十进制度数）
- 海拔高度（WGS84）
- 定位质量（0=无效，1=GPS，2=DGPS，4=RTK 固定解，5=RTK 浮点解）
- 使用的卫星数量
- HDOP（水平精度因子）

## 坐标系说明

- **纬度（latitude）**：十进制度数，范围 -90 到 +90
  - 正值：北纬
  - 负值：南纬
- **经度（longitude）**：十进制度数，范围 -180 到 +180
  - 正值：东经
  - 负值：西经
- **海拔（altitude）**：米，相对于 WGS84 椭球面
  - 正值：在椭球面之上
  - 负值：在椭球面之下

## 故障排查

### 串口权限问题

如果遇到 "Permission denied" 错误：

```bash
sudo chmod 666 /dev/ttyACM0
# 或者将用户添加到 dialout 组
sudo usermod -a -G dialout $USER
# 然后重新登录
```

### 检查串口数据

```bash
# 查看原始 NMEA 数据
cat /dev/ttyACM0

# 或使用 minicom
sudo apt install minicom
minicom -D /dev/ttyACM0 -b 115200
```

### 没有数据输出

1. 检查 GPS 接收器是否正常工作
2. 确认串口设备路径正确
3. 确认波特率设置正确
4. 检查 GPS 信号（需要在室外或窗边）
5. 查看节点日志：`ros2 node list` 和相关日志

## 示例输出

### 节点日志

```
[INFO] [gnss_converter_node]: Serial port /dev/ttyACM0 opened successfully
[INFO] [gnss_converter_node]: GNSS Converter Node started
[INFO] [gnss_converter_node]: GPS - Lat: 30.76761167°, Lon: 103.98357700°, Alt: 514.10m | Status: RTK Fixed | Sats: 12, HDOP: 0.52
```

### ROS2 话题输出

```bash
$ ros2 topic echo /sensing/gnss/navsat --once
---
header:
  stamp:
    sec: 1732704123
    nanosec: 456789000
  frame_id: gnss_link
status:
  status: 2
  service: 15
latitude: 30.76761167
longitude: 103.98357700
altitude: 514.1
position_covariance:
- 1.0816
- 0.0
- 0.0
- 0.0
- 1.0816
- 0.0
- 0.0
- 0.0
- 2.1632
position_covariance_type: 2
---
```

## 测试

详细测试指南请参考 [TESTING.md](TESTING.md)

快速测试：
```bash
cd /home/cplus/Desktop/yunle_sensors
./test_gnss.sh
```

## License

MIT
