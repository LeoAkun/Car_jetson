# 一、项目说明

本项目是基于 ROS 2 的自动驾驶小车驱动与控制系统工作空间。项目集成了多种硬件传感器驱动，实现了一个完整的自动驾驶小车底层驱动系统。

## 硬件设备

- **激光雷达**
  - Livox Mid360（内置 IMU）
  - 速腾 32 线激光雷达（RoboSense）
  - 两个单线激光雷达（雷科感知）

- **定位传感器**
  - IMU（WG-ACES ACCELEROMETER）
  - GPS/GNSS

- **执行机构**
  - 底盘驱动
  - 门锁控制器
  - 警报灯控制器

## 软件功能

- 传感器数据采集与处理
- 点云数据转换（Livox 自定义格式转标准 PointCloud2）
- 激光雷达数据融合
- 底盘运动控制（cmd_vel 转 ECU 信号）
- TF 坐标变换管理

# 二、构建驱动包

为了处理包与包之间的依赖关系，需要依次进行构建：

```shell
cd real_ws
colcon build --packages-select livox_ros2_driver serial
colcon build
```

**注意事项**：
- 首先构建 `livox_ros2_driver` 和 `serial` 包，因为其他包依赖它们
- 第二次 `colcon build` 会构建工作空间中的所有包
- 构建完成后需要 source 环境变量：`source install/setup.bash`

# 三、目录结构

```
real_ws/
├── src/                          # 源代码目录
│   ├── driver/                   # 硬件驱动包
│   │   ├── imu/                  # IMU 驱动（fdilink_ahrs）
│   │   ├── lidar/                # 激光雷达驱动
│   │   │   ├── livox_ros_driver2/    # Livox Mid360 驱动
│   │   │   ├── convert_laser/        # 点云格式转换节点
│   │   │   └── rslidar/              # 速腾雷达驱动
│   │   ├── scan_laser/           # 单线雷达驱动
│   │   │   ├── Lakibeam_ROS2_Driver-main/  # 雷科感知雷达驱动
│   │   │   └── ros2_laser_scan_merger/      # 激光雷达数据融合
│   │   ├── gps/                  # GPS/GNSS 驱动
│   │   ├── car_control/          # 底盘控制
│   │   │   ├── chassis_driver/        # 底盘通信驱动
│   │   │   └── cmdvel_2_ecu/          # 速度指令转换节点
│   │   ├── lock_controller/      # 门锁控制器
│   │   ├── alarm_controller/     # 警报灯控制器
│   │   └── camera/               # 相机驱动
│   └── launch_real/              # 启动文件包
│       └── launch/
│           └── run.launch.py     # 一键启动所有传感器
├── build/                        # 编译输出目录
├── install/                      # 安装目录
├── log/                          # 日志目录
└── README.md                     # 本文件
```

# 四、运行驱动

## 1. 识别环境

在运行之前，需要先 source 工作空间的环境变量：

```bash
source install/setup.bash
```

为了方便，可以将此命令添加到 `~/.bashrc` 文件中：

```bash
echo "source /home/akun/workspace/Car_jetson/real_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. 一键启动传感器

使用主启动文件启动所有传感器和驱动节点：

```bash
ros2 launch launch_real run.launch.py
```

此命令会按顺序启动以下节点：
- IMU 驱动
- Livox Mid360 激光雷达
- 点云转换节点
- 速腾 32 线激光雷达
- 底盘驱动
- cmd_vel 转 ECU 节点
- GPS 驱动
- 两个单线激光雷达
- 激光雷达融合节点

## 3. 单独启动各驱动

如果需要单独测试某个传感器，可以使用对应的 launch 文件：

### Livox Mid360 激光雷达
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 速腾激光雷达
```bash
ros2 launch rslidar_sdk start.py
```

### IMU
```bash
ros2 launch fdilink_ahrs ahrs_driver.launch.py
```

### 单线雷达（双雷达）
```bash
ros2 launch lakibeam1 lakibeam1_scan_dual_lidar.launch.py
```

### 雷达融合
```bash
ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py
```

### 底盘驱动
```bash
ros2 launch chassis_driver chassis_driver.launch.py
```

### GPS
```bash
ros2 launch gnss_converter gnss_converter.launch.py
```

### 门锁控制器
```bash
ros2 launch lock_controller lock_controller.launch.py
```

## 4. 查看传感器数据

### 查看 TF 变换树
```bash
ros2 run tf2_tools view_frames
```

### 查看话题列表
```bash
ros2 topic list
```

### 查看激光雷达数据（RViz）
```bash
rviz2
```

在 RViz2 中添加相应的显示项（PointCloud2、LaserScan 等）来可视化传感器数据。

# 五、坐标系说明

项目定义了以下坐标系及其变换关系：

| 坐标系 | 父坐标系 | 位置偏移 (x,y,z,米) | 说明 |
|--------|----------|---------------------|------|
| base_link | - | (0, 0, 0) | 小车中心坐标系 |
| livox_frame | base_link | (0.35, 0.0, 0.51) | Livox 雷达坐标系 |
| rslidar | base_link | (0.35, 0.0, 0.51) | 速腾雷达坐标系 |
| imu_link | livox_frame | (0.0, 0.0, -0.11) | IMU 坐标系 |
| gnss_link | livox_frame | (-0.7, 0.03, 0.0) | GPS 天线坐标系 |
| laser | - | - | 单线雷达坐标系（由驱动指定） |

**注意**：实际安装位置可能需要根据具体车辆调整 TF 变换参数。

# 六、注意事项

1. **设备连接**
   - 确保所有传感器已正确连接并上电
   - 检查串口权限（如 `/dev/ttyUSB*`、`/dev/ttyACM*`）
   - 如需永久权限，将用户添加到 `dialout` 组：`sudo usermod -aG dialout $USER`

2. **网络配置**
   - 某些传感器（如速腾雷达）可能需要配置网络接口
   - Livox Mid360 默认 IP 为 `192.168.1.1`，需配置主机在同一网段

3. **启动顺序**
   - 主 launch 文件已内置定时启动机制，避免资源竞争
   - 如需自定义启动顺序，修改 `run.launch.py` 中的 `TimerAction` 周期

4. **禁用的节点**
   - 警报灯控制器（`alarm_controller`）默认注释，如需使用请取消注释
   - 门锁控制器（`lock_controller`）默认注释，如需使用请取消注释

5. **性能优化**
   - 在 Jetson 等嵌入式平台运行时，建议调整传感器发布频率以降低 CPU 负载
   - 可以通过各驱动的配置文件修改发布频率

6. **故障排查**
   - 使用 `ros2 topic echo <topic_name>` 检查话题数据
   - 使用 `ros2 node list` 检查节点是否正常运行
   - 检查日志文件：`log/latest/` 目录

# 七、维护者

- 维护者：akun
- 邮箱：2370344139@qq.com

# 八、相关链接

- [ROS 2 官方文档](https://docs.ros.org/en/humble/)
- [Livox SDK2 文档](https://github.com/Livox-SDK/Livox-SDK2)
- [速腾雷达驱动](https://github.com/RoboSense-LiDAR/rslidar_sdk)