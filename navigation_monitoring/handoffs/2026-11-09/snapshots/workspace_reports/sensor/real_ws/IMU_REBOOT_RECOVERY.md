# IMU 重启后无数据问题修复记录

## 现象

电脑重启后，IMU 驱动可以启动，`/dev/wheeltec_FDI_IMU_GNSS` 也存在，驱动日志显示串口已经成功打开，但是 `/imu` 没有数据。手动拔插 IMU 的 USB 线，再启动驱动后，IMU 数据恢复。

## 原因分析

### 串口已经打开，但 IMU 没有输出字节

现场确认 IMU 使用的是 CP2102 USB 串口桥，设备属性为：

| 项目 | 值 |
| --- | --- |
| USB Vendor ID | `10c4` |
| USB Product ID | `ea60` |
| USB Serial | `0001` |
| 稳定设备名 | `/dev/wheeltec_FDI_IMU_GNSS` |
| 实际串口 | `/dev/ttyUSB0` |
| 波特率 | `921600` |

这说明 udev 对 IMU 的识别是正确的，不是 `/dev/ttyUSB*` 编号变化或设备别名错误。问题发生在更后面：CP2102 对主机来说已经可用，驱动也可以执行 `open()`，但 IMU 没有从 UART 发出任何数据帧。

原驱动只处理了“串口不存在”或“串口打开失败”的情况。一旦出现“串口能打开，但持续没有数据”，驱动会一直在读取超时中循环，不会关闭串口、更不会尝试重置硬件。因此 ROS 节点存在、发布者也已创建，但 `/imu` 不会有任何消息。

### 为什么拔插会恢复

拔插 USB 会使 CP2102 被注销并重新枚举，同时会重新建立串口控制线状态。对于由 USB 供电，或通过 DTR/RTS 与复位电路有关联的 IMU，这个过程等价于一次硬件侧重新初始化，所以 IMU 开始恢复输出数据。

系统原来的开机脚本已经先等待 30 秒，`run.launch.py` 中 IMU 又延迟 15 秒启动。因此 IMU 实际在开机约 45 秒后才启动，问题不是简单增加几秒启动等待就能解决，而是需要处理“端口存在但无数据”的异常状态，并在开机时重新枚举 USB 设备。

### 开机脚本中的 sudo 问题

`robot-service.service` 使用 `User=akun` 运行 `/home/akun/start_robot_service.sh`。原脚本中的 `sudo chmod` 在 systemd 启动环境没有交互终端，日志会出现 `sudo: a terminal is required to read the password`。

这不是 IMU 无数据的直接原因，因为 udev 规则已提供串口访问权限；但这些命令不可靠，也会把所有 USB 串口设置为全局可写。已将其移除，串口权限统一交给 udev 管理。

## 修改内容

### IMU 驱动

修改目录：`src/driver/imu/fdilink_ahrs_ROS2`

- `include/ahrs_driver.h`
  - 初始化 `serial_timeout_` 为 `1000 ms`。
  - 增加无数据恢复参数和串口恢复函数声明。

- `src/ahrs_driver.cpp`
  - 串口不存在或打开失败时每秒重试。
  - 成功打开串口后发送一次 DTR/RTS 低-高脉冲，并清空串口输入缓冲。
  - 连续 5 秒读取不到任何字节时，关闭串口、等待 500 ms、重新打开并再次执行复位脉冲。

- `launch/ahrs_driver.launch.py`
  - 启用 `respawn=True`，节点异常退出后等待 2 秒自动重启。
  - 增加 `no_data_reconnect_ms_: 5000` 与 `reset_on_open_: true` 参数。

### 整车启动

修改文件：`src/launch_real/launch/run.launch.py`

- IMU 启动延迟由 5 秒调整为 15 秒。

### 开机脚本

修改文件：`/home/akun/start_robot_service.sh`

- 删除三个 `sudo chmod` 命令。
- 保留 30 秒启动等待、ROS 环境加载和整车 launch 启动。

### USB 重新枚举服务

新增文件：

- `src/driver/imu/fdilink_ahrs_ROS2/scripts/reset_wheeltec_imu_usb.sh`
- `src/driver/imu/fdilink_ahrs_ROS2/systemd/reset-wheeltec-imu.service`

服务在 `robot-service.service` 之前执行，最多等待 20 秒查找 IMU。找到序列号为 `0001` 的 CP2102 后，将其 USB 授权状态依次改为 `0` 和 `1`，强制重新枚举，再启动机器人服务。

## 脚本文件内容

### `/home/akun/start_robot_service.sh`

```bash
#!/bin/bash
sleep 30
# Serial-device permissions are configured by udev rules.
# Source ROS2
source /opt/ros/humble/setup.bash
# Source real_ws
source /home/akun/workspace/sensor/real_ws/install/setup.bash
# 启动 launch 文件
ros2 launch launch_real run.launch.py
```

### `reset_wheeltec_imu_usb.sh`

文件路径：`src/driver/imu/fdilink_ahrs_ROS2/scripts/reset_wheeltec_imu_usb.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail

# The IMU uses a CP2102 with this immutable USB serial number. Toggling
# authorization forces a fresh USB enumeration after the host has rebooted.
# USB hubs can enumerate after systemd starts, so wait up to 20 seconds.
for ((attempt = 1; attempt <= 20; ++attempt)); do
  for device in /sys/bus/usb/devices/*; do
    vendor_file="$device/idVendor"
    product_file="$device/idProduct"
    serial_file="$device/serial"
    authorized_file="$device/authorized"

    if [[ ! -r "$vendor_file" || ! -r "$product_file" || ! -r "$serial_file" || ! -w "$authorized_file" ]]; then
      continue
    fi

    if [[ "$(<"$vendor_file")" == "10c4" && "$(<"$product_file")" == "ea60" && "$(<"$serial_file")" == "0001" ]]; then
      echo 0 > "$authorized_file"
      sleep 1
      echo 1 > "$authorized_file"
      exit 0
    fi
  done
  sleep 1
done

echo "Wheeltec IMU CP2102 (10c4:ea60, serial 0001) was not found" >&2
exit 1
```

## systemd 服务配置

文件路径：`src/driver/imu/fdilink_ahrs_ROS2/systemd/reset-wheeltec-imu.service`

```ini
[Unit]
Description=Reset the Wheeltec IMU USB bridge after boot
After=systemd-udev-settle.service
Wants=systemd-udev-settle.service
Before=robot-service.service

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/reset-wheeltec-imu-usb

[Install]
WantedBy=robot-service.service
```

## 用户需要执行的命令

### 1. 编译更新

```bash
cd ~/workspace/sensor/real_ws
colcon build --packages-select fdilink_ahrs launch_real
source install/setup.bash
```

### 2. 安装并启用 USB 复位服务

以下命令仅需执行一次，需要输入 sudo 密码：

```bash
sudo install -m 755 ~/workspace/sensor/real_ws/src/driver/imu/fdilink_ahrs_ROS2/scripts/reset_wheeltec_imu_usb.sh /usr/local/sbin/reset-wheeltec-imu-usb
sudo install -m 644 ~/workspace/sensor/real_ws/src/driver/imu/fdilink_ahrs_ROS2/systemd/reset-wheeltec-imu.service /etc/systemd/system/reset-wheeltec-imu.service
sudo systemctl daemon-reload
sudo systemctl enable reset-wheeltec-imu.service
```

### 3. 重启并验证

```bash
sudo reboot
```

重启完成后：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/sensor/real_ws/install/setup.bash
systemctl status reset-wheeltec-imu.service --no-pager
ros2 topic hz /imu
```

`ros2 topic hz /imu` 持续显示频率，说明 IMU 已在不拔插 USB 的情况下正常恢复。

## 注意事项

- 正常开机只由 `robot-service.service` 启动 `run.launch.py`。
- 不要额外手工运行 `ros2 launch fdilink_ahrs ahrs_driver.launch.py`，否则会与整车启动的 IMU 驱动抢占同一串口。
- 如果更换了 IMU 或 CP2102，需要重新读取 USB 的 Vendor ID、Product ID 和 Serial，并修改 USB 复位脚本中的匹配条件。
