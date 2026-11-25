# 2026-11-11 15:00:53 传感器与导航/FastDDS残留检查

## 分类与时间范围

- 本轮是用户授权的传感器短测、GNSS串口只读核查及导航/FastDDS残留检查，**未派发导航任务，未进入有效导航**，不是113→106→107路线失败。
- 检查开始：2026-11-11T15:00:53.038+08:00；最后一次残留核查：2026-11-11T15:14:37.872035+08:00。以下时间均为北京时间。
- 运行目录绝对路径：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-00-53_readiness_check/`
- 历史记录绝对路径：`/home/akun/workspace/Car_jetson/navigation_monitoring/test_handoff/history/2026-11-11_15-00-53_readiness_check.md`
- 没有现场人员对车体全程静止的确认；本轮没有导航派单，不等于车辆物理位置或运动状态已得到证明，物理位置以现场人员为准。
- 无导航过程人工接管记录。仅停止本轮新启动的传感器短测组；独立温度监控PID1446保持原状。未修改源码、启动文件、配置参数、已安装程序或系统服务，未构建安装、未回放、未运行FastDDS清理。

## 观测窗口与主要结果

| 窗口 | 执行/证据 | 结果与限制 |
|---|---|---|
| 15:00:53起 | `check_context.json`、`process_inventory.json`、`hardware_inventory.json` | 当前boot ID为`d53fafeb-91ac-4ceb-8254-8bb737042ac6`，与上午`3bc95af4-21c3-458d-acda-61a57fc277dd`不同。只证明两次观察间boot已变化，不指定重启原因。硬件枚举记录串口、视频节点及RealSense USB设备；枚举不能代替流数据验证，激光链以随后消息采样为证。 |
| 15:04:40–15:05:25 | 45秒传感器短测，前10秒暖机 | 新启动IMU、Rslidar、双2D激光、scan_merge、GNSS五组；未启动导航栈。数据见`sensor_samples.jsonl`与各统计JSON。 |
| 15:10:41.521648–15:10:49.522589 | `/dev/ttyS8`只读非阻塞读取 | 3217字节、96行NMEA；8条GGA均quality=0、satellites=00，无fix。没有写串口、刷新输入或修改串口配置。 |
| 15:12:58.104910–15:13:24.478012 | 独立RealSense补测，采样约25秒，前8秒暖机 | 深度流和camera_info可见；仅短时流数据验证，未验收导航负载下稳定性、标定或测距精度。 |
| 15:03:16 / 15:09:21 / 15:14:37 | FastDDS及进程残留三次只读扫描 | 原27项→64项→64项，原对象全部同inode保留，无清理；详见下文。 |

## IMU：启动异常与稳定段必须分开

- 共4398条`/imu`。15:04:41.178901收到的首条四元数模为`1.08170448214449e-36`；15:04:41.1848924收到第6条四元数模为`1.238299215406449e38`。均为有限数值异常，不能以无NaN/Inf当作姿态正常。
- 15:04:41.183360012，驱动记录`check crc16 failed (ahrs), total_crc_errors=1`，位于两条异常之间。这是AHRS CRC，不能写成IMU CRC，也不能据此认定设备USB重连或唯一根因。
- 第6条相对前一header前进约0.320秒；从首条到该条header累计前进约0.360秒，接收时间仅前进约0.006秒。header随后持续领先接收约0.353秒；至15:05:25.104仍领先0.353101969秒。
- 暖机后3500条、约35秒：100.013Hz，header dt均值10ms、范围约9.916–10.047ms；无非正dt、无>30ms gap、无异常四元数。**此稳定段不覆盖启动的0.320秒gap，也不证明绝对时间偏置已恢复。** 全记录最大接收间隔约16.265ms，说明启动header大跳不能直接等同接收端断流0.320秒。
- 15:05:25.133的`Short IMU serial read`出现在SIGINT之后，属于停止窗口，不能当作先前运行期短读证据。未取得本轮运行中USB重连、serial number丢帧或设备原始Timestamp整数的完整证据。
- `imu_source_review.md`已只读核对：`imuTimestamp()`对设备时间作epoch映射与单调保护，没有正向跳变/未来偏差门槛；AHRS payload先写成员缓存再CRC校验，坏CRC/短帧不会恢复最后有效姿态，首个有效AHRS前也无明确就绪门槛。代码允许不良/未就绪AHRS缓存进入后续IMU姿态发布；与本次现象一致，但没有原始串口payload，不能证明具体异常帧的唯一成因，AHRS CRC也不能直接证明造成IMU计数器正跳。
- 本轮新启动驱动，安装ELF SHA256为`7ea6a0ee8e2d7986335b0b4fee18f59fbf3b8cab42fe016a41367806b630b321`，mtime 2026-11-11 00:32:29+08:00。未取得运行PID16629的`/proc/exe`哈希；不能以当前安装文件或源码审查代替故障瞬间运行身份的完整证明，也不能沿用旧轮deleted二进制的判断。

证据：`sensor_summary.json`、`sensor_anomaly_audit.json`、`sensor_extended_statistics.json`、`imu_source_review.md`、`imu.log`、`post_sensor_process_verification.json`。

## 激光、GNSS与深度

- 暖机后前2D扫描约29.692Hz、后2D约29.520Hz、合并扫描约29.691Hz；header无非正dt、无>0.2秒gap。合并扫描末帧722束、659个有限range。这只证明采样窗口有数据，未验证横墙伪影、点位几何或导航代价地图。
- Rslidar共429帧，暖机后342帧、约9.771Hz；3次header gap略超过0.2秒，接收时刻15:04:59.718、15:05:06.118、15:05:21.017，对应0.200862、0.200599、0.200123秒。全记录最大接收间隔约0.212658秒。未将约两帧间隔直接指定为某个硬件或网络根因。
- GNSS节点曾成功打开`/dev/ttyS8`，但传感器短测没有收到`/sensing/gnss/pose_with_covariance`。后续8秒原始NMEA的GGA均无fix、卫星00；可确认串口有输出，不能确认当时具备重定位可用的GNSS位置，不能由此认定接收机损坏。
- 深度图678帧，暖机后504帧、约29.582Hz，848×480、16UC1；camera_info共715帧，暖机后510帧、约29.994Hz。header无非正dt；深度图全记录最大header gap约0.100954秒。
- 深度启动15:12:59.393–59.626有7帧无正深度像素；暖机后正深度比例最小0.810854、中位0.835440。不能忽略启动零深度帧，也不能把短测结果外推到全部工况。

证据：`sensor_samples.jsonl`、`sensor_summary.json`、`sensor_extended_statistics.json`、`sensor_anomaly_audit.json`、`gnss.log`、`gnss_serial_raw.txt`、`gnss_serial_read_events.jsonl`、`gnss_serial_read_summary.json`、`depth_samples.jsonl`、`depth_summary.json`、`depth_anomaly_audit.json`。

## FastDDS与进程保留

- 原检查27项：11个port、11个sem mutex、2个segment、3个`_el`。
- 五组传感器短测结束后64项：相对基线新增37项，原27项全部同inode保留，无删除或替换。新增不应被唯一归给某一个传感器节点，窗口内其他DDS参与者也可能创建对象。
- RealSense补测结束后最终仍64项：22个port、22个sem、7个segment、13个`_el`；与中间64项完全同inode，无新增/删除/替换。
- 三次可读进程扫描均未见相关FD/mmap持有者，`/proc/locks`无匹配共享内存设备号与inode的锁。部分系统进程FD/maps无权读取，非交互sudo只读权限探测需密码，因此只能称**疑似孤儿残留**，不能绝对排除不可读系统进程持有，更不能将对象存在当作通信故障已证。
- `/opt/ros/humble/bin/fastdds`存在，仅查看`--help`及`shm --help`；未运行`shm clean`，未删除`/dev/shm`对象。
- 最终15:14:37核对PGID16624–16628、17588无可见成员，未见ROS/nav/sensor残留进程。PID1446独立温度监控仍为相同命令、状态S，未操作。

证据：`fastdds_check.json/.md`、`fastdds_after_sensor_check.json/.md`、`fastdds_final_check.json/.md`。前次证据保留。

## 停止、证据窗口与范围

- `sensor_processes.json`记录本轮五个launcher PGID16624/16625/16626/16627/16628，均由本轮脚本SIGINT停止、launcher返回0；部分子节点在停止后显示退出-2，应区分计划停止与运行中崩溃。
- 深度PGID17588由本轮脚本SIGINT停止，launcher返回0，结束15:13:24.478012。
- 异常窗口文件位于`event_windows/`，索引`event_windows_index.json`。Rslidar前两次gap包含前后各约10秒；IMU启动窗口前侧与Rslidar第3次gap后侧被采样边界截断，索引已标注。没有补造缺失前后文。
- 本轮没有LIO-SAM、IMU预积分ready、Nav2 ACTIVE/action、全局规划或控制执行的验收证据；未评价106停稳/航向、106→107衔接、偏左、2km/h与0.8km/h补偿、横墙/路沿或低高度过滤。下一轮真实导航须单独记录，不能由本目录存在判断车辆正在导航。

## 交代码聊天检查的文件/模块与缺失证据

1. `/home/akun/workspace/Car_jetson/real_ws/src/driver/imu/fdilink_ahrs_ROS2/src/ahrs_driver.cpp`、`include/ahrs_driver.h`、`include/fdilink_data_struct.h`：AHRS临时帧校验后提交、首帧就绪、坏CRC/短读对最近有效姿态的污染、有限但无效四元数门槛、IMU与AHRS配对及过期语义。详见`imu_source_review.md`。
2. 同驱动`imuTimestamp()`：首帧缓存、正向设备时间突变、未来偏差、epoch与单调保护交互；缺失原始设备Timestamp/host now/epoch状态与原始串口payload，不能锁定唯一原因。
3. GNSS converter与接收机输入条件：结合NMEA quality=0、卫星00、无位置topic检查可用fix前提，不预设串口无数据或设备损坏。
4. FastDDS参与者退出/SHM对象生命周期、ROS运行环境及导航启动通信：保留现有对象和元数据供分析；本轮没有复现Nav2启动失败，不能由残留项数量指定上午故障原因。
5. Rslidar接收/转换链约0.2秒间隔与深度启动零值帧，可在代码聊天结合更长负载与原始消息检查。本聊天只记录，不实现修复或修改参数。
