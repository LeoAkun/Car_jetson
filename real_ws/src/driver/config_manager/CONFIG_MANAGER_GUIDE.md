# 在线配置页面使用说明

## 1. 功能概述

`config_web.py` 是一个独立运行的 Python Web 工具，用于管理
`real_ws/src/driver` 下部分驱动的配置，并通过浏览器执行部分 ROS 运行操作。

页面代码、HTTP 服务、HTML、CSS 和 JavaScript 都集中在
`config_web.py` 中，不需要单独安装前端工程。

页面主要提供以下功能：

- 读取和修改驱动源码区配置；
- 编译 ROS 2 工作空间；
- 启动、停止和查看部分驱动进程；
- 调用门锁服务；
- 查看运行状态和操作日志。

页面不等同于整车主启动文件。整车启动仍然使用：

```bash
ros2 launch launch_real run.launch.py
```

## 2. 文件结构

```text
config_manager/
├── config_web.py                 # 在线配置页面主体
├── README.md                     # 简要说明
└── CONFIG_MANAGER_GUIDE.md       # 本说明文档
```

`config_web.py` 会根据自身位置自动计算工作空间根目录：

```text
/home/akun/workspace/sensor/real_ws
```

因此页面保存配置时，修改的是源码目录中的文件，而不是直接修改
`install` 目录中的编译产物。

## 3. 启动页面

### 3.1 在小车本机启动

在小车上打开终端执行：

```bash
cd /home/akun/workspace/sensor/real_ws
python3 src/driver/config_manager/config_web.py
```

默认监听地址和端口为：

```text
127.0.0.1:8765
```

在小车本机浏览器访问：

```text
http://127.0.0.1:8765
```

### 3.2 从局域网内其他电脑访问

如果要使用自己的电脑访问小车页面，必须让服务监听所有网卡：

```bash
cd /home/akun/workspace/sensor/real_ws
CONFIG_MANAGER_HOST=0.0.0.0 \
CONFIG_MANAGER_PORT=8765 \
python3 src/driver/config_manager/config_web.py
```

在小车上查询局域网 IP：

```bash
ip -4 addr
```

假设小车 IP 是 `192.168.1.140`，则在电脑浏览器访问：

```text
http://192.168.1.140:8765
```

小车和电脑必须位于可以互相连通的同一局域网。页面默认没有用户认证，
不要把端口直接暴露到不可信网络。

### 3.3 检查页面是否正在监听

在小车上执行：

```bash
ss -ltnp | grep 8765
```

远程访问时应看到类似：

```text
0.0.0.0:8765
```

如果显示 `127.0.0.1:8765`，只能在小车本机访问；如果没有输出，说明页面进程没有运行。

### 3.4 停止页面

回到运行 Python 服务的终端，按：

```text
Ctrl+C
```

关闭该终端也会结束页面服务。当前项目没有为配置页面提供独立的 systemd
自启动服务，除非另外配置后台服务，否则每次需要手动启动。

## 4. 可编辑的配置文件

页面当前编辑以下源码配置文件：

```text
src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py
src/driver/car_control/cmdvel_2_ecu/config/config.yaml
src/driver/imu/fdilink_ahrs_ROS2/launch/ahrs_driver.launch.py
src/driver/rslidar/rslidar_sdk/config/config.yaml
src/driver/scan_laser/Lakibeam_ROS2_Driver-main/launch/lakibeam1_scan_dual_lidar.launch.py
src/driver/scan_laser/ros2_laser_scan_merger/config/params.yaml
src/driver/gps/config/gnss_converter.yaml
src/driver/lock_controller/config/lock_controller.yaml
src/driver/alarm_controller/alarm_controller/config/alarm_controller.yaml
```

对应页面模块为：

- 底盘控制；
- IMU；
- 速腾雷达；
- Livox 相关运行控制；
- 单线雷达和雷达融合；
- GNSS；
- 门锁；
- 报警灯。

页面不提供相机参数的完整编辑功能，也不提供 Livox 3D 雷达全部参数的编辑功能。
Livox 可以通过运行控制启动，但详细参数仍需直接修改其驱动配置文件。

## 5. 配置修改后的生效流程

推荐流程如下：

1. 启动配置页面；
2. 在页面修改参数并保存；
3. 在页面执行工作空间编译，或者手动编译；
4. 停止并重新启动受影响的 ROS 节点。

手动编译命令：

```bash
cd /home/akun/workspace/sensor/real_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

仅修改某个包时，可以缩小编译范围，例如：

```bash
colcon build --packages-select lock_controller
```

已经运行的节点不会自动读取新参数。保存配置或重新编译后，必须重启对应节点。

## 6. 页面中的运行控制

页面可以单独控制部分节点，包括底盘、IMU、速腾雷达、Livox、单线雷达、GNSS、
门锁和报警灯。

常用的底层命令与页面控制大致对应：

```bash
ros2 launch fdilink_ahrs ahrs_driver.launch.py
ros2 launch rslidar_sdk start.py
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch lakibeam1 lakibeam1_scan_dual_lidar.launch.py
ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py
ros2 launch gnss_converter gnss_converter.launch.py
ros2 launch lock_controller lock_controller.launch.py
```

报警灯节点对应：

```bash
ros2 run alarm_controller alarm_controller \
  --ros-args \
  --params-file src/driver/alarm_controller/alarm_controller/config/alarm_controller.yaml
```

门锁服务对应：

```bash
ros2 service call /unlock lock_controller/srv/UnlockCommand \
  "{board_address: 1, lock_address: 1}"
```

## 7. 避免重复启动和串口抢占

主启动文件会启动多个驱动。如果已经执行：

```bash
ros2 launch launch_real run.launch.py
```

不要再通过页面重复启动同一个 IMU、门锁或报警灯节点，否则可能出现：

- 两个进程同时打开同一个串口；
- 门锁命令发到错误进程；
- IMU 节点无法读取数据；
- 节点名称或 ROS 服务重复。

进行单独测试时，应先停止整车主启动，再通过页面或命令启动单个节点。

特别是 IMU，不要同时运行手工启动的 `fdilink_ahrs` 和整车启动中的 IMU。

## 8. 页面无法访问时的检查顺序

在小车上依次执行：

```bash
pgrep -af config_web.py
ss -ltnp | grep 8765
ip -4 addr
```

在本地电脑上执行：

```bash
ping 小车IP
curl -v http://小车IP:8765/
```

常见结果：

- 没有 `config_web.py` 进程：页面没有启动；
- 没有 `8765` 监听：Python 服务已经退出或启动失败；
- 监听在 `127.0.0.1`：使用 `CONFIG_MANAGER_HOST=0.0.0.0` 重新启动；
- `Connection refused`：端口没有服务监听；
- `Connection timed out`：检查 IP、网络连通性和防火墙；
- 能返回 HTML 但浏览器打不开：检查浏览器地址、代理和缓存。

如果启用了防火墙，还需要允许 TCP 8765 端口，例如：

```bash
sudo ufw status
```

## 9. 与整车启动的关系

在线配置页面只是管理工具，不会自动替代整车启动。正常运行整车时：

终端 1：

```bash
cd /home/akun/workspace/sensor/real_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /home/akun/.local/realsense_ros/setup.bash
ros2 launch launch_real run.launch.py
```

终端 2：

```bash
cd /home/akun/workspace/sensor/real_ws
CONFIG_MANAGER_HOST=0.0.0.0 \
CONFIG_MANAGER_PORT=8765 \
python3 src/driver/config_manager/config_web.py
```

这样可以用主启动文件运行整车，同时用浏览器查看配置或执行不冲突的管理操作。

