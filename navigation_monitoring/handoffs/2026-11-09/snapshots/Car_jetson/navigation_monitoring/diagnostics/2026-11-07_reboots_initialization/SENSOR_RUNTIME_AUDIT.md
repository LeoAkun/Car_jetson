# 传感器实际运行路径核实

审计时间：2026-11-07。仅阅读既有日志、历史命令记录、安装文件与构建元数据；未启动 ROS、驱动、导航或回放，未修改运行配置。

## 可确定的结论

第二次异常重启前，传感器由 `/home/akun/workspace/Car_jetson/real_ws/install` 启动。没有发现该次启动使用 `/home/akun/workspace/sensor/real_ws` 或 `/home/akun/car-host-step9.MgehLL/...` 临时 overlay 的证据。第一次故障前的完整驱动启动输出没有保存，因此不能用第二次的进程列表替代第一次的逐节点证据。

`convert_laser/convert` 节点在第二次重启前确实启动，不可因 Livox 驱动关闭就宣称此转换节点没有运行。已有记录不能证明它收到过不一致的 Livox CustomMsg，也不能证明发生过越界。

## 第二次重启前的启动路径

- 历史工具命令记录 2026-11-07 16:16:46.637（北京时间）显示：cwd `/home/akun/workspace/Car_jetson`；依次 `source /opt/ros/humble/setup.bash`、`source real_ws/install/setup.bash`，再用 `subprocess.Popen(['ros2','launch','launch_real','run.launch.py'])` 启动 PID 4079。相关最小证据已抽取到 `sensor_runtime_start_evidence.json`，含来源会话文件及行号。没有执行历史命令。
- `real_ws/install/setup.bash:22-28` 的前缀链为 `/opt/ros/humble` 与自身目录，没有 sensor 或 driver-fix 临时 overlay 前缀。
- 16:17:37.051 保存的进程查询列出 `Car_jetson/real_ws/install/chassis_driver/lib/chassis_driver/chassis_driver_node`、`.../cmdvel_2_ecu/lib/cmdvel_2_ecu/cmdvel_2_ecu`、`.../ros2_laser_scan_merger/lib/ros2_laser_scan_merger/ros2_laser_scan_merger`。
- `navigation_monitoring/runs/2026-11-07_16-10-50/real_drivers.log:104` 中速腾配置绝对路径也是 `/home/akun/workspace/Car_jetson/real_ws/install/rslidar_sdk/share/rslidar_sdk/config/config.yaml`。
- 注意：该目录中的 `real_driver_stack.log` 当前为 **0 字节**；有内容的是 **real_drivers.log**。后者行 1 从 **16:16:47** 开始，属于第一次重启之后，不应按目录名 16-10-50 解释为第一次重启前日志。

## 两次重启前实际启动阶段

### 第一次：boot 271a95a2 → 7a56ea4e，后一次系统开机 16:16:07

- 16:12:03–16:12:05，重定位与姿态初始化进程已出现：`runs/2026-11-07_16-10-50/monitor.raw.jsonl:19-22`。
- 最后导航监测快照 **16:15:04.554**：`monitor.raw.jsonl:55`。重定位和姿态初始化进程存在，LIO-SAM/Nav2 数组为空；IMU/扫描速率仍为 0。该快照早于随后传感器进程数上升，不能据它断言故障前传感器始终没启动。
- `thermal_testing/idle_crash_runs/20261107_124850_nav_field_test/samples.csv:11944-11948`：16:15:02.852 至16:15:06.973，匹配的机器人进程计数依次2、2、3、13、16。最后3秒有明显启动活动。
- `/home/akun/.ros/log/2026-11-07-16-15-04-164100-admin1-Default-string-10128/launch.log` 当前0字节，未保存具体节点、参数或最终就绪状态。第一次传感器 launch 的完整绝对路径与逐节点情况不能从这些文件独立复原；没有证据显示它使用 sensor 旧目录。
- 最后资源样本 **16:15:06.973**：CPU 19.6%、36°C、可用内存 28,983,608 KiB（27.64 GiB）、节流计数0。前一秒 CPU51.7%，持续时间只有单样本，温度37°C，不能称持续过载。

### 第二次：boot 7a56ea4e → ca32f49f，后一次系统开机16:19:30

- `real_drivers.log:23-35`：16:16:48，IMU、底盘、速度转换、报警、GNSS、门锁、前后单线雷达、扫描融合、点云转扫描进程启动。
- `real_drivers.log:36-40`：底盘实际为 `can bus type:1`（EthCAN_UDP）、`br-sensors`，不是源码中的 USB2CAN 初始化分支。
- `real_drivers.log:90-95`：IMU串口初始化；**16:16:49** 启动 `convert`（PID4312）、速腾SDK节点（PID4314）及rslidar转换节点（PID4316）。
- `real_drivers.log:168`：**16:16:50** 启动RealSense节点（PID4351）。`/home/akun/.ros/log/2026-11-07-16-16-47-078853-admin1-Default-string-4079/launch.log:23-44`提供精确 epoch 时间。
- Livox设备驱动的启动在 `real_ws/src/launch_real/launch/run.launch.py:194` 被注释，但转换节点在行195仍无条件排定2秒启动；二者不可混为一谈。
- **16:18:27.026**快照（`runs/2026-11-07_16-17-52/monitor.raw.jsonl:15`）：IMU100Hz、融合扫描29.8Hz、车速0；重定位和初始化进程存在，Nav2/LIO数组尚为空。
- 管理日志 `runs/2026-11-07_16-17-52/management_stack.log:75-83`显示 **16:18:28.324** 初始化成功，**16:18:28.563**请求启动LIO-SAM，**16:18:28.763**创建LIO launch目录。**16:18:29.038**进程快照（monitor.raw.jsonl:16）实际观察到LIO的4个PID；Nav2仍为空。
- 最后温度样本 **16:18:28.859**（`thermal_testing/idle_crash_runs/20261107_161617_nav_field_test/samples.csv:129`）：CPU24.9%、55°C、可用内存28,037,528KiB（26.74GiB）、节流0。第二次在初始化完成、LIO刚启动阶段中断；不能据此认定LIO是原因，更不能归因于尚未观察到运行的Nav2优化器。

## sensor旧副本、当前源文件与实际二进制的边界

已抽样计算SHA-256并保存 `sensor_runtime_file_manifest.json`：run.launch.py、IMU源文件、IMU/速腾/单线雷达/convert安装程序均与sensor副本不同。路径相似不代表同一版本，也不能把sensor里的全部缺陷自动套给本次运行版本。

- Car_jetson速腾安装程序软链接到`real_ws/build/rslidar_sdk/rslidar_sdk_node`；单线雷达软链接到`real_ws/build/lakibeam1/lakibeam1_scan_node`；mtime均10月29日。
- Car_jetson安装的IMU为普通ELF，mtime **10月15日13:40:44**，不是源码软链接。构建元数据 `real_ws/build/fdilink_ahrs/CMakeFiles/ahrs_driver_node.dir/DependInfo.cmake:11-12` 指向Car_jetson源目录；`CMakeCache.txt:365,630` 同样指向该目录，未见临时driver-fix目录。
- IMU安装ELF与build ELF的hash不同，安装步骤可能修改RPATH；不能只凭hash不同推导编译错源。现有构建元数据也不是逐字源码快照，不能证明当前每项源码修改都已编入10月15日ELF。当前`ahrs_driver.cpp` mtime为9月28日；后续launch配置修改不等于重新编译该二进制。
- 当前二进制hash是审计时状态；故障瞬间没有保存二进制hash，仍有历史版本复原限制。

资源计数只提供相关性。整机两次复位瞬间未留存完整内核证据，源码审计应区分：直接主机重启逻辑、用户态进程崩溃、I/O间接触发内核/硬件问题，不能用后者的可能性替代实证。
