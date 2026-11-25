# 本轮IMU时间戳与AHRS CRC路径只读核对

检查时间：2026-11-11T15:13:40+08:00。仅阅读本轮监测证据、当前源码/头文件、安装launch及二进制符号/字符串；只新建本文件，没有修改驱动/launch/参数，没有启动设备、ROS节点或回放，没有构建安装。

## 可直接确认的实测事实

来源：[sensor_anomaly_audit.json](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-00-53_readiness_check/sensor_anomaly_audit.json)、[sensor_summary.json](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-00-53_readiness_check/sensor_summary.json)、[imu.log](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-00-53_readiness_check/imu.log)。

- 15:04:41.178901接收的首条 `/imu`，四元数模`1.08170448214449e-36`，header=`1794380681.1777167`。首条之前没有同轮已记录消息，JSON中的`prev`重复首条，不能当作独立前帧。
- 15:04:41.1848924接收的第6条 `/imu`，四元数模`1.238299215406449e38`，相对前一header约`+0.320s`；该帧header=`1794380681.5377169`，已领先同机接收约`0.352824450s`。
- 两异常之间，日志准确时间15:04:41.183360012记录`check crc16 failed (ahrs), total_crc_errors=1`。这是AHRS CRC错误，不是IMU CRC错误，也不是USB重新枚举证据。
- 从首条至第二条异常，header累计前进约`0.360000134s`，接收时间仅前进约`0.005991459s`，两者差约`0.354008675s`。到15:05:25.104仍领先约`0.353101969s`；全记录stamp减receive中位数约`0.352951050s`。
- 总计4398条IMU；暖机后的3500条约35秒，接收频率100.013Hz，header dt均值10ms，范围约9.916—10.047ms，无非正dt、无>30ms gap。`sensor_summary.json`的gaps为暖机后统计，不能覆盖启动的+0.320s。稳定频率不证明绝对时间偏置已恢复。
- 停止时15:05:25.133的`Short IMU serial read for frame type`位于SIGINT之后，不能当作此前运行中串口短读故障。

## 时间戳路径：可以解释持续偏置的代码事实

[ahrs_driver.cpp](/home/akun/workspace/Car_jetson/real_ws/src/driver/imu/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:187)的`imuTimestamp(int64_t device_timestamp_us)`：

1. 首次有效设备时间，或`device_timestamp_us <= last_device_time_us_`时，将设备epoch锚到当前ROS时间（同时保证不小于上一stamp+1ns）。
2. 后续用`ros_time_epoch_ns_ + (device_timestamp_us - device_time_epoch_us_) * 1000`映射；每条再取`max(mapped, last_ros_stamp+1)`保证单调。
3. **没有**正向设备时间跳变上限，没有`mapped_stamp-host_now`未来偏差检查，没有运行中漂移/时钟比例校验，也没有根据未来偏差重锚。
4. [ahrs_driver.cpp](/home/akun/workspace/Car_jetson/real_ws/src/driver/imu/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:609)明确从`imu_frame_.frame.data.data_pack.Timestamp`计算 `/imu.header.stamp`。AHRS时间戳不参与这个调用。

因此，若启动读到缓存中的旧设备epoch、随后设备时间快速追上，或设备计时器出现正向跳变，当前算法允许stamp领先host，并可在后续100Hz稳定采样时保留固定偏置。**这是由源码支持的机制候选，不是本轮设备计数器真实发生了何种变化的证明。** 本轮没有原始串口帧、原始IMU `Timestamp`整数及映射epoch状态日志，不能确定+0.320s来自设备计数跳变、缓存追赶、漏帧/拒帧、设备复位后的首帧处理或其他原因。单调保护还可能在重锚到host时保留既有未来stamp，应一并检查。

## AHRS CRC与姿态发布路径：存在独立的状态污染缺口

- [fdilink_data_struct.h](/home/akun/workspace/Car_jetson/real_ws/src/driver/imu/fdilink_ahrs_ROS2/include/fdilink_data_struct.h:124)的`ahrs_frame_read`使用联合体覆盖字节接收缓冲与解析后的AHRS字段。
- [ahrs_driver.cpp](/home/akun/workspace/Car_jetson/real_ws/src/driver/imu/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:486)先把AHRS payload直接写进成员`ahrs_frame_.read_buf.read_msg`，然后计算CRC；CRC失败只累加告警并`continue`，frame_end失败也只`continue`。**并没有恢复之前CRC正确的AHRS姿态，也没有把该成员标为不可用。**
- [ahrs_driver.cpp](/home/akun/workspace/Car_jetson/real_ws/src/driver/imu/fdilink_ahrs_ROS2/src/ahrs_driver.cpp:609)在后续IMU帧通过自身CRC后，照常读取`ahrs_frame_.frame.data.data_pack.Qw/Qx/Qy/Qz`用于姿态；没有验证该AHRS是否成功校验、是否新鲜、是否与当前IMU时间接近，也没有有限值/四元数模门槛。
- [ahrs_driver.h](/home/akun/workspace/Car_jetson/real_ws/src/driver/imu/fdilink_ahrs_ROS2/include/ahrs_driver.h:91)的`imu_frame_`、`ahrs_frame_`没有显式值初始化；构造路径没有“至少收到一条有效AHRS才能发布完整姿态”的标志。因此第一条有效IMU早于有效AHRS时，也可能发布未初始化的姿态。这是首条异常的明确检查方向，尚缺原始帧顺序证明。
- 单次`readExact()`会把已经收到的部分字节写入成员缓冲，返回失败后仅`continue`；AHRS短读同样可能破坏共享的“最近姿态”状态。本轮运行期未出现对应短读告警，不能据此认定这次由短读触发。
- `device_type_=1`对四元数仅左/右乘旋转四元数，不做有效性门槛或归一化。单位旋转乘法不会把正常单位四元数合理变成`1e-36/1e38`，应优先检查输入AHRS缓存内容与有效性。

**直接证实的是代码允许坏/未就绪AHRS缓存被下一条有效IMU复用；实测CRC与第二条异常在毫秒内相邻，与该缺口一致。** 没有原始AHRS payload就不能确认告警对应的具体坏四元数字节，更不能宣称CRC告警是全部异常的唯一根因。

此外，IMU payload CRC错误路径在`imuTimestamp()`前`continue`，AHRS CRC错误路径不调用`imuTimestamp()`。两种时间戳和姿态问题在此处来源于不同帧/字段，不能把AHRS CRC错误直接写成已证明造成IMU计数器+0.320s。

## 现有安装文件与源码范围

- 本轮`sensor_processes.json`记录新启动`ros2 launch fdilink_ahrs ahrs_driver.launch.py`，日志节点PID16629，正常退出；没有依据把这轮等同于此前沿用deleted旧二进制的10:49轮。
- 当前安装二进制SHA256：`7ea6a0ee8e2d7986335b0b4fee18f59fbf3b8cab42fe016a41367806b630b321`；mtime 2026-11-11 00:32:29（上海）。`nm -C`可见`imuTimestamp(long)`、`readExact(...)`、`processLoop()`；字符串包含设备时间重锚、短读、AHRS CRC告警。未执行该二进制。
- 源`ahrs_driver.cpp` SHA256：`661cee6cb32f6ccd88cdcc7014e39cd33414300d37f7ad5b7d8525fcd09244b8`；构建`DependInfo.cmake`指向本`Car_jetson`源码路径。当前安装launch设置`use_device_timestamp_=true`、`reset_on_open_=true`。
- 本次只读资料未提供PID16629运行时`/proc/.../exe`哈希，因此安装文件身份与当前源码风险不能替代完整的故障瞬间逐字构建/运行证明。安装与build ELF hash不同也可能来自安装RPATH处理，不能直接说安装错版。

## 交代码聊天的具体检查项

1. `fdilink_ahrs_ROS2/src/ahrs_driver.cpp`的AHRS读取/CRC块：临时帧完整读取并CRC/frame_end/数值有效后，才提交“最后有效AHRS”；坏帧/短读不能覆盖仍待发布使用的有效状态。
2. 同文件IMU发布块及`include/ahrs_driver.h`：初始AHRS就绪、有效姿态/时间标记、过期策略、IMU与AHRS配对；未就绪时的姿态字段和orientation_covariance语义应明确，不能伪造正常姿态。
3. `imuTimestamp()`：首帧缓存、正向突变、设备复位/重连、映射未来偏差和单调约束交互；补足原始设备Timestamp、host now、epoch、最后有效时间的可追踪证据，保留真实采样间隔。
4. 由代码聊天离线验证：首帧只有IMU、有效AHRS后注入坏CRC/短帧、无效但有限四元数、正向时间跳变、启动缓存快速读空、时间回退/串口重连。以上是建议检查方向，本聊天没有实现或执行这些测试。

本轮是未派发导航任务的传感器短测，未获得车体全程静止的现场确认；未验收LIO就绪、Nav2、导航控制、实车运动精度或硬件时间同步。
