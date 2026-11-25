# sensor 驱动：主机重启、电源与设备复位路径审计

审计时间：2026-11-07；仅静态读源码、ELF 信息与既有日志。未启动 ROS、驱动、导航或设备测试，未执行被审计脚本，未修改源码及系统配置。

## 结论

在 `/home/akun/workspace/sensor/real_ws/src` 与 `/home/akun/workspace/Car_jetson/real_ws/src/driver` 中，未发现明确调用 Linux 主机 reboot/poweroff/halt、systemctl reboot、SysRq、硬件看门狗、主机电源控制或加载/卸载内核模块的活动驱动代码。命中的 reset/reboot/shutdown 路径应分开理解：USB 串口设备重新枚举、CAN 通道复位、Livox 雷达设备重启和 ROS 进程退出，均不是源码中的主机重启命令。

这只排除了可见的直接重启逻辑；不能凭此排除用户态 I/O 触发内核/硬件缺陷、闭源库内部缺陷、供电异常或固件看门狗。既有日志没有保存两次复位瞬间，仍不能建立单个驱动与整机重启的因果链。

## 可复核路径

以下相对源码路径均以 `/home/akun/workspace/Car_jetson/real_ws/src` 为基准；sensor 副本同名路径亦已对照搜索。

| 路径与行号 | 实际行为 | 启动/执行证据及结论 |
|---|---|---|
| `driver/imu/fdilink_ahrs_ROS2/scripts/reset_wheeltec_imu_usb.sh:18-22` | 仅选择 VID:PID `10c4:ea60`、序列号 `0001` 的 CP2102，向该设备 sysfs `authorized` 写 0，等 1 秒，再写 1。 | IMU USB 设备重新枚举，不是主机 reboot。20 次查找上限见行 7。没有重置 USB 根控制器或 PCIe 网卡的命令。 |
| `driver/imu/fdilink_ahrs_ROS2/systemd/reset-wheeltec-imu.service:7-12` | oneshot，ExecStart `/usr/local/sbin/reset-wheeltec-imu-usb`，WantedBy `robot-service.service`。 | 当前 `systemctl show`：该服务和 `robot-service.service` 均为 `LoadState=not-found`、`ActiveState=inactive`、`FailureAction=none`、`StartLimitAction=none`；`/usr/local/sbin` 无该脚本。源码模板不能视为当前已安装/执行证据。 |
| `driver/lock_controller/scripts/open_door_recover.py:134-162` | 只重绑指定门锁 `ch341` USB 接口，必要时切该 USB 设备 authorized。 | 独立恢复工具；实车 `launch_real/launch/run.launch.py` 启动门锁节点，没有调用此恢复脚本。不是主机 reboot，也没有触碰 `enp2s0` 的 PCIe 路径。 |
| `driver/car_control/chassis_driver/chassis_driver/src/usb2can_core.cpp:174-180,485-486,527` | `VCI_ResetCAN`、`VCI_CloseDevice` 作用于 USB-CAN 通道/适配器；错误分支 `exit(1)` 退出本进程。 | 第一次重启后、第二次异常重启前的 `real_drivers.log:36-40`（16:16:48）明确为 `can bus type:1`、`can eth card is:br-sensors`，对应头文件枚举 `EthCAN_UDP=1`；`run()` 行 930-936 进入网络 CAN 分支。只能确认这一轮未进入 USB2CAN 复位初始化，不能据此替代第一次重启前缺失的完整启动证据。 |
| `driver/lock_controller/src/lock_controller.cpp:274-280,473-497` | `/long_power_control` 服务把 `0x9A/0x9B` 命令发给 RS485 锁控板上的指定锁通道。 | README 行 25 定义为电磁锁持续通电/断电，不能因名称含 power 就当成工控机断电接口。节点构造仅注册服务（行 58-60），不会自动调用此命令；第二次异常重启之前保存的 `real_drivers.log` 未检出 `Long power control request`，首次重启前完整日志未保存。物理接线是否另接电源不能由源码确认。 |
| `driver/config_manager/config_web.py:443-450,1266-1267` | 用户点击门锁长通电开/关按钮时调用上述服务。 | 无周期自动主机断电逻辑。 |
| `driver/lidar/Livox-SDK2/sdk_core/command_handler/command_impl.cpp:830-837` | `LivoxLidarRequestReboot` 给指定设备 handle 发 `kCommandIDLidarRebootDevice` 协议命令。 | 重启的是 Livox 雷达。`launch_real/launch/run.launch.py:194` 的 Livox 启动已注释，运行记录中是速腾节点。 |
| `driver/rslidar/rslidar_sdk/launch/humble_start.py:9-29` | 缺 Cyclone DDS 时交互式 sudo apt 安装，并设置当前进程 RMW 环境。 | 是系统软件修改入口，但没有 reboot。实车 `launch_real/launch/run.launch.py:61-66` 引用的是 `start.py`，不是 `humble_start.py`。 |
| 各驱动的 `rclcpp::shutdown()`、`rclpy.shutdown()`、launch Shutdown 事件 | 停止 ROS 上下文/节点/launch。 | 这些 API 不是 Linux 主机关机；不可用日志中的 shutdown 字样直接推导整机重启。 |

附加恢复工具 `/home/akun/workspace/Car_jetson/restart_imu.sh` 行 79-109 只向匹配的 IMU 进程发送 INT/TERM/KILL，行 124-131 重新启动一个 IMU 节点；没有主机重启或 USB/PCIe 控制命令。

## 厂商 libcontrolcan.so 的边界

只使用 `file`、`readelf -d/-Ws` 和 `strings -a`，没有运行该库、没有执行 `ldd`。

- 路径：`real_ws/src/driver/car_control/chassis_driver/chassis_driver/lib/x86/linux_64/libcontrolcan.so`。x86-64 ELF，共享库，Build ID `9f927c25a8423faac30366a3daeeeb468d15e484`。
- 动态依赖为 libc、pthread；导入包含 open/read/ioctl、内存和同步函数，未见 reboot/system/popen/exec 或看门狗函数导入。
- 字符串与导出符号包含 `usb_reset`、`VCI_UsbDeviceReset`、`usb_detach_kernel_driver_np`、`/dev/bus/usb`、`/proc/bus/usb`，表明存在用户态 USB 操作和设备复位能力。
- 已安装 `chassis_driver_node` 的 NEEDED 包含 `libcontrolcan.so`，RUNPATH 指向上述 Car_jetson 库目录。它会被动态加载；16:16:48 日志确认的 Ethernet CAN 分支没有进入源码中的 USB2CAN 调用路径。此分支证据仅覆盖第二次异常重启前这一轮，首次重启前最后一次完整启动链未保存。
- 导入/字符串检索不能证明闭源库没有直接系统调用或内部缺陷，不能完全审计其所有行为。库加载也不等于执行了 USB 复位函数。

## 与两次异常重启的联系

- `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-07_16-10-50/real_drivers.log:23-49` 保存的是第一次重启后、第二次异常重启前的 16:16:47-48 驱动启动日志，不应把目录名误当作日志内容时间。底盘网络 CAN 分支证据在行 36-40；门锁节点初始化在行 41-49。旁边同目录的 `real_driver_stack.log` 当前为 0 字节，不提供节点启动证据。
- 同一 `real_drivers.log` 行 90-95 显示该轮 IMU 串口初始化与速腾节点启动，未检出主机关机、设备恢复脚本或门锁长通电服务调用。首次重启前最后一次完整 launch 内容没有保存，不能把这份第二轮日志当成首次所有驱动的执行证明。
- 既有 `reboot_resource_evidence.json` 记录两次最后 CPU 温度分别 36°C、55°C，可用内存约 27.6/26.7 GiB，节流计数均 0；没有资源耗尽直接导致复位的证据。
- 首两次 boot 内核日志未检出软件请求关机、已落盘 panic、OOM 或不可纠正/致命 PCIe 错误；持久日志末端有时间缺口，不能排除缺口内发生错误。
- `enp2s0` 的 PCIe RxErr 已在启动机器人代码之前出现，且源码未见设置该 PCIe 链路、电源或 AER 的活动路径。只能说明它不是这些驱动显式打印或主动制造的“软件报错”；不能由静态审计排除启动 I/O、硬件负载或内核路径对潜在故障的触发。

下一步应优先保留复位瞬间的系统证据，并做受控驱动分组对照来区分“程序直接退出”“I/O 触发内核/硬件故障”“供电/固件复位”。本轮没有执行该类实机测试。
