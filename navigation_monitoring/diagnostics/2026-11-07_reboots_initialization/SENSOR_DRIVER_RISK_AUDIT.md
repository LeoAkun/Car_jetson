# sensor 传感器驱动与整机重启：只读源码审计

审计日期：2026-11-07。仅阅读源码、已有构建参数与已保存取证记录，没有运行 ROS、传感器、驱动、回放或控制程序，没有修改驱动源码或系统配置。

## 结论与证据边界

- 在重点检查的速腾、Lakibeam、IMU、GPS、相机源码中，没有找到主动重启主机、写 sysrq、操作主机 watchdog 的代码。`rclcpp::shutdown()` 只终止 ROS 上下文；`exit(-1)`、未捕获异常或 `std::terminate` 会结束驱动进程，不是重启 Linux。
- 找到有条件触发的内存积压、异常退出、停机不可靠等软件缺陷。它们值得修复，但不能因“代码有缺陷”就认定为两次整机异常复位原因。
- 两次异常重启前最后保存的 CPU 使用率约 19.6%、24.9%，CPU 温度 36°C、55°C，可用内存 27.64、26.74 GiB，未见 OOM、panic 或 CPU 温度节流记录。这不支持把目前发现的内存积压或 CPU 忙循环认定为已发生的重启原因。最后若干秒未落盘，仍保留不确定性。
- `enp2s0` 的可纠正 PCIe RxErr 属于主板与网卡的链路层错误；源码中的 UDP socket/read/close 不能直接作为 RxErr 的生成原因。驱动网络流量可能使底层硬件问题更易暴露，但目前没有有/无流量对照或内核崩溃证据证明这种因果链。

## 副本与实际运行版本

本次按用户要求检查 `/home/akun/workspace/sensor/real_ws/src/driver`，同时核对 `/home/akun/workspace/Car_jetson/real_ws/src/driver`。

- `sensor` 与 `Car_jetson` 中的 Lakibeam、速腾节点与 stop 实现、GPS 默认串口、扫描融合源码存在差异。
- IMU 的 `ahrs_driver.cpp` 两份主要逻辑相同，差异只是复位控制线注释措辞；不能把它当成两套不同解析实现。
- 下述当前代码行号均指 `Car_jetson/real_ws/src/driver`，旧副本问题另列 `sensor` 路径。
- **源码存在修复不等于实车已运行该修复。** 运行来源还可能是 `install` 下普通二进制或另一个 overlay，需结合实际启动路径和二进制校验；本报告不凭源码修改时间断言部署生效。当前本地 `build/rslidar_sdk/CMakeCache.txt` 的 `ENABLE_EPOLL_RECEIVE=OFF`、`ENABLE_IMU_DATA_PARSE=OFF` 只说明该构建目录配置，不能证明之前每次实车所用二进制来自这一构建。

## 1. Lakibeam：收帧缓冲没有点数或时间上限

位置：

- `Car_jetson/real_ws/src/driver/scan_laser/Lakibeam_ROS2_Driver-main/src/lakibeam1_scan.cpp:196`：仅当角度恰好为 0 且已有点时将 `scan_vec_ready` 设为 1。
- 同文件 `:210`：持续 `scan_vec.push_back(response_ptr)`。
- 同文件 `:271`：只有完成一帧并发布后才 `scan_vec.clear()`。
- 同文件 `:232`：`scan_vec.size()` 被赋给 `uint16_t num_readings`。

触发条件：传感器持续输出带有效 DataFlag 的点，但因丢掉含零角度的数据报、非完整扫描配置或异常数据，一直没有满足“角度等于 0”的收帧条件。单纯网络完全断开不会持续增加该向量。

影响：可积累多圈数据、增大内存与延迟；超过 65535 点时发布点数还会发生截断。向量没有硬上限，理论上长期异常可耗尽进程/系统内存。目前没有监控证据表明两次重启时发生了这种积压。

建议：使用方位角跨越检测与最大帧时长共同收帧，对最大点数设置上限，异常帧清空并限频告警；在窄化到 `uint16_t` 前检查长度。无需启动设备即可补充离线解析测试。

## 2. 速腾：已完成点云队列没有容量上限

位置：

- `Car_jetson/real_ws/src/driver/rslidar/rslidar_sdk/src/source/source_driver.hpp:230`：`putPointCloud()` 无条件 push。
- 同文件 `:204`：空闲池没有对象时分配新点云对象。
- 同文件 `:264`：消费者先 `sendPointCloud()`，后把对象放回空闲池。
- `.../src/rs_driver/src/rs_driver/utility/sync_queue.hpp:48`：底层 `SyncQueue::push` 没有容量限制。
- `.../src/source/source_pointcloud_ros.hpp:487`：发送包含点云转 ROS 消息及 publish。

触发条件：点云生产长期快于转换/发布消费，例如 CPU 调度不足、发布阻塞或消费者变慢。

影响：待发布点云增加、每帧继续分配内存，导致数据延迟和内存压力。正常速度匹配时会循环复用对象，不能仅凭没有上限就称为“必然内存泄漏”。

已有保护：`lidar_driver_impl.hpp:389` 的原始包队列超过 10240 会清空；`decoder.hpp:477` 的单帧点数超过 5000000 会清空。**这些保护不限制已完成点云队列总量**。MSOP/DIFOP 长度也在解码前检查（`decoder.hpp:442`、`:491`）。

建议：已完成点云队列设置小容量和丢旧帧策略，记录队列深度、丢帧数、帧龄与 RSS，验证压力下可控退化。现有资源记录不支持把这一风险定为本次复位原因。

## 3. IMU：串口断连异常未捕获，短读也未完整检查

位置：

- `Car_jetson/real_ws/src/driver/imu/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:163`：`processLoop()` 整段没有异常捕获。
- 同文件 `:181`、`:210`、`:221`、`:382`、`:418`、`:452` 等串口读；除首字节空读外，多数未检查实际读取字节数。
- 同文件末尾 `main()` 直接构造 `FDILink::ahrsBringup`，没有异常捕获。
- `.../imu/serial_ros2/src/impl/unix.cc:585`：设备显示可读但读取无数据时抛 `SerialException`；`:488` 等也可抛 `IOException`。

触发条件：USB 串口设备断连、驱动 I/O 错误或半帧超时。

影响：未捕获异常可结束 IMU 驱动进程，外层服务可能重启驱动/服务；短读可能引起 CRC 告警、失步或使用旧缓冲内容。固定长度校验和 CRC 是现有保护，没有看到将外部可变长度直接读入固定 IMU 数据缓冲造成明确溢出的路径。未知 ground 帧长度最多 255+4 字节，小于其 500 字节缓冲。

另有 `include/ahrs_driver.h:72` 的 `frist_sn_` 未初始化，会影响首次序列号丢帧统计，不是整机复位证据。

建议：捕获串口 I/O 与 SerialException，统一关闭/有限退避重连；每次解析必须先读满目标长度；初始化所有状态变量；CRC/帧头告警使用限频日志。节点退出应明确告警，不应被描述成 Linux 已重启。

## 4. GPS：异常种类捕获不全；输入长度已有限制

位置：

- `Car_jetson/real_ws/src/driver/gps/src/gnss_converter_node.cpp:72`：`readline(65536, "\n")`，单次字符串有 64 KiB 上限。
- 同文件 `:78`：读取只捕获 `serial::IOException`。
- `.../imu/serial_ros2/include/serial/serial.h:672`、`:690`：`SerialException`、`IOException` 是两个不同的 `std::exception` 子类，前者不会被后者捕获。
- 同文件 GPS 源码 `:181`：数字转换错误有 `std::exception` 捕获。

触发条件与影响：串口断连可能抛出漏捕获的 SerialException，导致 GPS 节点退出；连续异常也可能产生定时器频率的日志。未见持续无界增长的 NMEA 缓冲。GGA 未校验 NMEA checksum 与坐标范围属于数据可靠性问题，不是已证明的整机重启机制。

## 5. 旧 sensor 副本停止缺陷，与当前源码区别

### Lakibeam

- `sensor/real_ws/src/driver/scan_laser/Lakibeam_ROS2_Driver-main/src/lakibeam1_scan.cpp:35` 忽略 `create_socket()` 失败。
- 同文件 `:129` 的 `while(1)` 不检查 shutdown；`:134` 忽略 recvfrom 返回值，无接收超时。绑定失败/无效 fd 的异常路径可能忙循环并重复解析旧缓冲；正常断流则可能卡在阻塞接收，停止不及时。
- 当前 Car_jetson 源码 `:37` 检查 bind 失败，`:119` 增加 200 ms 接收超时，`:154` 检查 `rclcpp::ok()`，`:171` 拒绝长度不符数据包，主函数取消退出后再 spin 的路径。

### 速腾

- `sensor/.../rslidar_sdk/src/source/source_driver.hpp:183` 的 `stop()` 无 `joinable()` 检查，析构又调用 stop；重复 stop 可能抛 `std::system_error`，析构阶段可触发 terminate。
- 旧 `node/rslidar_sdk_node.cpp:52` 信号处理器使用日志及条件变量，`:132` 无条件等待，没有当前源码的显式 stop/reset。
- 当前 `Car_jetson/.../source_driver.hpp:184` 增加 driver 空指针与各线程 joinable 检查、原子退出标志，当前 `node/rslidar_sdk_node.cpp:114` 检查 ROS 状态并显式停止。

上述缺陷解释的是进程停止失败、abort 或被管理器强杀的可能性。没有证据表明它们能直接重启主机；也不在此声称当前源码修复已部署到两次异常复位时使用的二进制。

## 6. 其他检查范围

- `camera/camera_test.py:53` 一次读取/显示一个帧，失败后退出，未发现无界帧队列、系统重启或内核设置操作。异常时缺少 finally 清理属于资源善后问题；脚本配置 1080p60 也不能单凭此认定启动过或引起复位。
- 当前扫描融合 `scan_laser/ros2_laser_scan_merger/src/main.cpp:87`、`:168` 循环以 ranges.size 为边界，缓存仅保存前后各一条扫描；未发现持续历史累积。它会保存另一侧最后一帧且未严格做时间同步，影响感知新鲜度，不是主机重启证据。
- Livox 转换器 `lidar/convert_laser/src/convert.cpp:47` 按 point_num 分配，`:104` 按 points 实际长度写入，二者异常不一致时存在越界风险，应验证一致性。第二次复位前 `real_drivers.log:93` 明确保存转换节点启动 PID 4312；常规 launch 即使注释了 Livox 设备驱动，也仍启动此转换节点。因此不能称该节点未运行。用户此前未确认安装 Livox，也没有证据表明转换节点收到异常 Livox 消息或触发越界，不能据此解释整机复位。
- IMU USB reset 脚本 `imu/fdilink_ahrs_ROS2/scripts/reset_wheeltec_imu_usb.sh:18` 只匹配 CP2102 `10c4:ea60`、serial `0001`，`:19`/`:21` 对该 USB 设备 authorization 做一次断开/重新授权。其“reset/reboot”注释描述 IMU USB 重枚举与主机开机后的恢复，不是重启工控机。
- Lakibeam `remote.cpp:23` 等使用全局空 `node_handle`，若调用 REST 配置/遥测函数会有节点空指针风险；主接收构造函数里这些调用目前被注释，不能当作运行中已触发的缺陷。

## 建议修复顺序与重启排查联系

1. 给 Lakibeam 收帧与速腾点云队列加上界，并提供深度/帧龄/丢帧计数。
2. IMU/GPS 读串口统一处理异常与短读，告警限频、受控重连。
3. 核对实际执行路径与普通安装二进制，避免只改源码而继续运行旧版本；对停止路径做纯离线或无硬件的进程生命周期验证。
4. 继续从独立电源、DC 输入瞬态、主板复位来源、PCIe 链路和未落盘内核故障方向取证。没有观测到资源异常时，不把上述代码缺陷作为“重启原因已经找到”。

关联证据：同目录 `RESULT.md`、`REBOOT_AUDIT.md`、`KERNEL_FINDINGS.md`、`reboot_resource_evidence.json`。本报告是静态审计，未实施以上代码修复，未执行压力或硬件测试。
