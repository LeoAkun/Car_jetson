# sensor 代码与两次异常重启的关系

2026-11-07，只读源码、ELF 元数据和已有运行/系统日志。没有启动 ROS、传感器、导航或回放，没有执行恢复脚本，没有修改驱动或运行配置。

## 当前结论

**未找到 sensor 可见代码主动重启工控机的路径；发现了驱动稳定性缺陷，但没有证据把它们定为两次整机复位的原因。** 这不等于排除设备 I/O、内核、固件或供电之间的间接触发关系。两次已持久化内核日志均缺少最后约 26–33 秒，无法重建故障瞬间。

用户最新范围保持不变：只把前两次计作异常重启；16:58 关联的人工断电/关机不计入；回到室内后的 17:04 初始化不计入室外故障分析。

## 检查范围和版本边界

- 检查用户指定的 `/home/akun/workspace/sensor/real_ws/src`，并对照测试启动链使用的 `/home/akun/workspace/Car_jetson/real_ws`。两份源码不能混为同一版本。
- 文本对比覆盖 384 个候选源文件/配置文件，其中 374 个在两边同路径存在、31 个内容不同；排除了 `.git`、pycache 和第三方目录。差异数量不表示业务逻辑修改数量。完整哈希见 [sensor_source_comparison.json](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/sensor_source_comparison.json)。
- 源码与实际安装二进制还需分开：文件修改时间、源码中有修复、库被加载，都不能单独证明测试时执行了某条分支。
- 第一次复位前最后几秒只有机器人进程总数增长证据，不能据此识别所有驱动身份。该轮 `real_driver_stack.log` 为 0 字节；同目录的 `real_drivers.log` 实际从 16:16:47 开始，属于第一次复位之后、第二次复位之前。
- 第二次复位前的历史启动命令和进程记录明确使用 `Car_jetson/real_ws/install`；setup 链没有 `sensor` 或额外 `driver-fix` overlay。当前 IMU 安装文件为普通 ELF，不能仅凭源码或文件时间证明某次改动已编入。详细来源审计见同目录 `SENSOR_RUNTIME_AUDIT.md`。

## 已核实的直接重启与设备复位路径

在审计的驱动与启动脚本中，未找到主机 `reboot/poweroff/halt`、SysRq、主机看门狗或主动加载/卸载内核模块的活动代码路径。

| 命中项 | 实际作用 | 与主机复位的证据关系 |
|---|---|---|
| `rclcpp::shutdown()`、`rclpy.shutdown()` | 关闭 ROS 上下文 | 不能当作 Linux 主机关机命令 |
| IMU USB `authorized` 0/1 | 针对指定 CP2102 设备重新枚举 | 服务模板当前未安装，不能当作两次故障时执行过的操作 |
| 门锁恢复脚本 | 指定 ch341 USB 接口重绑 | 不在常规驱动启动链中 |
| `VCI_ResetCAN` | USB-CAN 通道/适配器复位 | 已保存驱动日志使用 Ethernet CAN 分支，不能把 USB 分支调用归给该轮运行 |
| 门锁 `long_power_control` | 对锁控板指定通道持续通电/断电 | 注册服务不等于自动调用；已保存测试日志无对应请求 |
| Livox reboot API | 向指定雷达设备发送重启协议命令 | 不是主机重启；常规启动中的 Livox 设备驱动已注释 |

`libcontrolcan.so` 是闭源厂商库。静态导入中未见重启类 API，但存在 `ioctl` 和 USB 设备复位能力；符号检索不能彻底排除内部缺陷或直接系统调用。审计详情和行号见 [SENSOR_POWER_AUDIT.md](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/SENSOR_POWER_AUDIT.md)。

监控服务的 `Restart=always` 只用于重启监控进程；已检查的 `idle_crash_monitor.py` 读取系统状态并保存文件，没有调用主机复位命令。

## 确实存在的软件风险

| 风险 | 触发条件及后果 | 是否已证明导致本次重启 |
|---|---|---|
| Lakibeam 收帧缓冲无上限 | 持续收不到零角度收帧条件时可能积累多圈数据、增加延迟和内存 | 否 |
| 速腾待发布点云队列无上限 | 长时间生产快于消费时可能积压点云并继续分配内存 | 否 |
| IMU 串口异常未捕获、短读检查不全 | 断连/I/O 错误时可能退出节点，或使解析失步 | 否；代码直接支持的是节点退出风险 |
| GPS 串口异常捕获不全 | `SerialException` 不会被仅有的 `IOException` 捕获 | 否；同样是节点退出风险 |
| Livox 消息转换未校验点数一致性 | 按 `point_num` 分配、按 `points` 长度写入，两者异常不一致时可能越界 | 否；转换节点启动过，但没有证据表明收到异常 Livox 消息或触发此路径 |
| 旧 sensor 的停止路径缺陷 | 重复 stop、阻塞接收等可能造成 abort 或退出不及时 | 否；当前源码已有差异，部署情况需单独核对 |

具体源码行号、触发条件和修复建议见 [SENSOR_DRIVER_RISK_AUDIT.md](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/SENSOR_DRIVER_RISK_AUDIT.md)。无界队列是条件性积压风险，不能直接称为已经发生的内存泄漏。

## 与既有重启证据对照

| 最后资源样本 | 第一次 | 第二次 |
|---|---:|---:|
| 时间 | 16:15:06.973 | 16:18:28.859 |
| CPU 占用 | 19.6% | 24.9% |
| CPU 温度 | 36°C | 55°C |
| 可用内存 | 27.64 GiB | 26.74 GiB |
| 温度节流计数 | 0 | 0 |

这些数据不支持“传感器程序把 CPU、内存或温度推到极限后重启”的判断。第二次在 16:18:29.038 还记录到 LIO-SAM 进程出现；两次异常前均未观察到 Nav2 已启动。不能用导航优化器报错解释这两个时段的整机复位。

`enp2s0` 的 PCIe RxErr 在机器人驱动启动前就已出现。没有发现 sensor 源码设置该 PCIe 端口、ASPM 或 AER 的活动逻辑。它仍是底层链路异常线索；网络/USB 活动是否会触发更深层故障，静态代码审计无法证明或排除。

下一步应补齐复位瞬间的留存证据，再通过分组启动对照区分程序退出、设备/内核异常及供电/固件复位。上述队列和串口缺陷可以单独加固，但不能把加固完成作为重启原因已解决的验收结论。本轮没有启动任何实机对照测试。

关联记录：[KERNEL_FINDINGS.md](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/KERNEL_FINDINGS.md)、[REBOOT_AUDIT.md](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/REBOOT_AUDIT.md)、[reboot_resource_evidence.json](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/reboot_resource_evidence.json)。
