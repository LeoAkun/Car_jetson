# 导航车换机后的驱动与配置检查

检查日期：2026-10-26（北京时间）。硬件状态最后复查时间：16:36:59。

检查对象：当前连接的主机 `admin1-Default-string`，Ubuntu 22.04.5，x86_64，内核 `6.8.0-138-generic`，ROS 2 Humble。检查结果仅适用于这台主机和当前可见项目。

## 结论

主要传感器的驱动源码和编译产物已具备，没有发现需要重新下载全部传感器驱动的情况。但当前环境还不能据此判定为可运行导航：存在确定缺少的软件依赖、有线网络配置缺失、GPS 串口不匹配，以及一个 USB 串口设备未绑定驱动。

本次进行了文件、依赖、动态库、USB、网卡、服务和 ROS 发现检查；没有启动导航或底盘控制，没有修改系统配置，没有安装软件。仅新增本报告。检测期间系统的 brltty 和 IMU 状态发生了外部变化，以下硬件结论以最后复查为准。

## 已具备的驱动与环境

| 项目 | 检查结果 | 验证边界 |
|---|---|---|
| ROS 2 Humble、colcon、rosdep | 已安装 | 默认 shell 只加载 `/opt/ros/humble`，项目工作空间仍需 source |
| RoboSense 雷达 | `rslidar_sdk`、`rslidar_msg`、转换程序存在 | 配置型号为 `RSHELIOS`；UDP 6699/7788；尚未验证数据 |
| 前后 Lakibeam 单线雷达 | `lakibeam1`、扫描融合程序存在 | 设备配置为 `192.168.1.12` / `.13`，主机地址 `.140` |
| Livox MID360 | `livox_ros_driver2`、转换程序、SDK2 库及头文件存在 | 总启动文件中 MID360 驱动启动动作被注释 |
| IMU | `fdilink_ahrs`、`serial` 存在，`cp210x` 已加载 | 最新复查 CP2102 已绑定驱动，配置的 by-id 路径存在，尚未验证 IMU 数据 |
| GPS/GNSS | `gnss_converter`、`serial` 存在 | 配置的 `/dev/ttyACM0` 不存在 |
| 底盘 | `chassis_driver`、`yunle_msgs`、速度转换程序存在 | 当前配置使用以太网 CAN，主机 IP 和网口状态不匹配 |
| RealSense D435i | ROS 相机驱动、描述包、librealsense 已安装 | SDK 枚举到序列号 `117322074703`，与配置一致；USB 3、uvcvideo 正常绑定；未测试深度流 |
| 门锁、报警器 | 驱动存在 | 报警器配置路径存在；门锁配置路径不存在，详见下文 |
| Nav2、LIO-SAM、重定位 | 对应工作空间及主要程序存在 | 编译产物的动态库检查通过；不等同于实际导航验证 |

检查方式与结果：

- `rosdep check --from-paths .../Car_jetson/real_ws/src --ignore-src --rosdistro humble`：通过。
- 加载 `Car_jetson/real_ws/install/setup.bash` 后，检查 86 个 ELF 文件：无 `not found` 动态库。
- 分别加载 `utils`、`slam`、`nav2`、`multi_map_navigation` 环境后，检查另外 216 个 ELF 文件：无 `not found` 动态库。
- 另一份 `sensor/real_ws` 加载环境后检查 73 个 ELF 文件：无 `not found` 动态库。
- `ros2 launch launch_real run.launch.py --show-args`：解析通过，此命令没有启动节点。
- Python 的 serial、numpy、scipy、OpenCV、cv_bridge、tf_transformations、rclpy 及主要自定义消息可导入。

## 确定缺少的软件依赖

| Ubuntu 软件包 | 实际失败 | 影响 |
|---|---|---|
| `python3-paho-mqtt` | `No module named 'paho'` | `mqtt_task_receiver`、`status_reporter` 无法导入 |
| `python3-networkx` | `No module named 'networkx'` | `route_planner` 无法导入 |

对应源码证据：

- [mqtt_task_receiver.py](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/mqtt_task_receiver.py:305)
- [status_reporter.py](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/status_reporter.py:11)
- [route_planner.py](/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/route_planner.py:7)

这两个依赖没有在该包的 `package.xml` 或 `setup.py` 中声明，因此 rosdep 不能完整发现它们。

补齐命令（本次未执行）：

```bash
sudo apt install python3-paho-mqtt python3-networkx
```

已用 `apt-get -s install python3-paho-mqtt python3-networkx` 模拟：只需新增这两个包，不要求卸载或升级其他包。

## 当前阻碍运行的配置问题

### 1. 有线网络未恢复

最后复查：`eno1`、`enp2s0` 为 UP，但没有 IP；`enp3s0`、`enp4s0` 为 DOWN。只有 Wi-Fi 地址 `192.168.3.23/24`，没有项目要求的 `192.168.1.140`，也没有通向传感器网段的专用路由。

底盘配置指定 `enp3s0`、主机 `192.168.1.140:8002`、CAN 盒 `192.168.1.10:4001/4002`。`enp3s0` 当前未连通，现有配置无法按原设计通信。

需要确认哪个物理网口接到车载交换机，在对应接口恢复原来的地址与掩码，并使底盘配置的网口名与之匹配。不能仅把 Wi-Fi 地址改成传感器地址，也不能凭接口名猜测接线。

证据：[底盘启动配置](/home/akun/workspace/Car_jetson/real_ws/src/driver/car_control/chassis_driver/chassis_driver/launch/chassis_driver.launch.py:23)。

### 2. GPS 指定串口不存在

总启动文件显式传入 `/dev/ttyACM0`，波特率默认 115200；当前没有任何 `/dev/ttyACM*`。

这说明 GPS 节点无法打开当前指定端口。现有证据不能区分 GPS 未连接、供电/线缆问题、设备枚举异常，或 GPS 实际使用了另一个串口。不要直接把 GPS 改成任意 `ttyUSB` 端口：需要先确认哪根线是 GPS，核对型号、实际设备路径、输出协议和波特率。

证据：[总启动文件](/home/akun/workspace/Car_jetson/real_ws/src/launch_real/launch/run.launch.py:132)、[GPS 参数](/home/akun/workspace/Car_jetson/real_ws/src/driver/gps/launch/gnss_converter.launch.py:8)。

### 3. 门锁对应 CH340 已枚举，但没有绑定串口驱动

系统识别到三个 CH340 USB 串口转换器，`ch341` 模块已经加载：

| USB 位置 | 最新状态 | 项目用途 |
|---|---|---|
| `1-1.2` | `ch341` → `/dev/ttyUSB0`；by-path 存在 | 与报警器配置路径对应，尚未验证协议 |
| `1-1.3` | `ch341` → `/dev/ttyUSB1`；by-path 存在 | 未确认实际连接的设备 |
| `1-1.4` | 接口未绑定驱动，无 tty | 与门锁配置的 USB 位置对应 |

门锁要求 `/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0`，当前不存在。由于同型号另外两个设备能绑定 `ch341`，这不是缺少整个 CH340 驱动包的表现。

发现系统 `85-brltty.rules` 匹配 `1a86:7523`；该 USB 设备的 udev 属性中存在 `BRLTTY_BRAILLE_DRIVER=bm`、`SYSTEMD_WANTS=brltty-udev.service`。最初 brltty 服务处于 active，最新复查已被外部操作改为 masked/inactive，但该 CH340 仍未绑定。brltty 冲突是有证据支持的排查方向，尚不能确认是唯一原因。内核日志需要管理员权限，本次未能读取。

下一步应核对 brltty 停用状态后，对这一只转换器重新插拔并检查绑定及日志；若仍失败，再检查线缆、Hub、供电与设备本身。

证据：[门锁配置](/home/akun/workspace/Car_jetson/real_ws/src/driver/lock_controller/config/lock_controller.yaml:6)、[报警器配置](/home/akun/workspace/Car_jetson/real_ws/src/driver/alarm_controller/alarm_controller/config/alarm_controller.yaml:3)。

### 4. IMU 最新已恢复设备识别，尚未验证数据

初次检查时 CP2102 没有绑定串口驱动；最后复查已绑定 `cp210x` 并生成 `/dev/ttyUSB2`，by-id 为：

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

这与 `Car_jetson` 中 IMU 的配置一致，波特率 921600。当前用户属于 dialout 组；USB 串口访问具备基础条件，仍需单独启动 IMU 并确认 `/imu` 数据。

证据：[IMU 启动配置](/home/akun/workspace/Car_jetson/real_ws/src/driver/imu/fdilink_ahrs_ROS2/launch/ahrs_driver.launch.py:21)。

### 5. 自动启动服务没有迁移

项目 README 提到的 `robot-service.service` 在当前系统为 `not-found`，`reset-wheeltec-imu.service` 同样不存在。当前没有发现相关传感器/底盘节点进程；默认 ROS 发现检查没有导航节点和传感器数据话题，只见 `/parameter_events` 与 `/rosout`。

如果希望上电自动启动，需要恢复原服务及其工作空间加载顺序。自启动服务缺失不代表驱动缺失。

## 另外需要保留的项目差异

### 两份同名驱动工作空间配置不同

`Car_jetson/real_ws` 与 `sensor/real_ws` 都有驱动和 install，但内容并不一致：

- `Car_jetson` 的 IMU 使用已恢复的 CP2102 by-id；`sensor` 使用当前不存在的 `/dev/wheeltec_FDI_IMU_GNSS` 别名。
- `Car_jetson` 的报警器使用 USB 物理路径；`sensor` 配置写的是 `/dev/ttyUSB2`。最新硬件复查中 `ttyUSB2` 是 IMU，不能直接套用该报警器配置。
- 总启动文件在相机参数作用域、报警器参数、雷达坐标系等方面也有差异。

后续测试宜明确加载主工程，避免叠加两份同名包：

```bash
source /opt/ros/humble/setup.bash
source /home/akun/workspace/Car_jetson/real_ws/install/setup.bash
ros2 pkg prefix launch_real
```

最后一条应指向 `Car_jetson/real_ws/install/launch_real`。本次核对的主工程关键 launch/config 源文件与其 install 副本一致。

### MID360 启动动作被注释

MID360 的源码、SDK 和编译产物存在，但总启动文件第 167 行注释了 `start_livox`。因此运行总启动文件不会发布 MID360 原始数据。

当前 LIO-SAM 默认配置读取 `/rslidar_Pointcloud2` 和 `/imu`，重定位也读取 RoboSense 点云，所以 MID360 未启动不一定阻断当前选用的导航方案。是否启用取决于实际使用哪台雷达。

MID360 配置的主机为 `192.168.1.140`，雷达为 `192.168.1.187`；项目 README 中的通用默认地址不能替代实际 JSON 配置。

证据：[总启动动作](/home/akun/workspace/Car_jetson/real_ws/src/launch_real/launch/run.launch.py:167)、[MID360 网络配置](/home/akun/workspace/Car_jetson/real_ws/src/driver/lidar/livox_ros_driver2/src/livox_ros_driver2/config/MID360_config.json:14)、[LIO-SAM 输入](/home/akun/workspace/Car_jetson/slam/src/LIO-SAM/config/params_rslidar.yaml:5)。

### 全工程 rosdep 报告的其他包需按用途判断

全工程检查报告缺少 `ros-humble-slam-toolbox`、`ros-humble-gtsam`、`ros-humble-gazebo-ros-pkgs`、`ros-humble-ompl`。它们不能全部等同于当前实车缺少运行驱动：

- GTSAM 已通过 Ubuntu 的 `libgtsam4` / `libgtsam-dev` 安装，LIO-SAM 实际解析到 `libgtsam.so.4`。
- OMPL 已通过 `libompl16` / `libompl-dev` 安装，Smac 实际解析到 `libompl.so.16`。
- `slam_toolbox` 是通用 Nav2 bringup 声明的依赖，当前工程另有 LIO-SAM；是否补装取决于是否使用该建图路径。
- `gazebo_ros_pkgs` 来自 Nav2 系统测试等内容，主要用于仿真/测试。

本次没有因为上述包名差异而安装或替换已有库。

地图文件方面，默认 Nav2 使用的 `/home/akun/Desktop/K8.yaml`、默认重定位使用的 `GlobalMap3.pcd` 均存在；多地图目录也有 PCD、PGM、YAML、CSV 文件。本次仅验证文件存在，没有验证地图与现场及标定的一致性。

## 建议处理顺序

1. 补齐 `python3-paho-mqtt`、`python3-networkx`，并在项目依赖清单中声明。
2. 确认交换机实际连接的网口，恢复 `192.168.1.140` 与底盘网口配置。
3. 确认 GPS 实际设备和串口；处理门锁 CH340 未绑定问题。
4. 在明确加载 `Car_jetson` 环境后，分别验证 IMU、相机、雷达和 GPS 数据。
5. 核对 TF、定位和底盘反馈后，再恢复自动启动与导航运行。

本报告不能替代实际传感器数据和车辆运行验证。
