# 3b2mus 实车测试：110→114

**本轮进入有效导航，启动后发生严重右转入路沿，现场人工接管，路线未完成，系统完成航点 0/2。** 110 未获到达成功，114 未下发至 Nav2。用户现场原话：“启动后不久，严重向右转向行驶，进入了路沿，属于全局地图中的障碍区域。然后接管了小车。”本轮按行驶异常及人工接管记录，不按通过或单纯启动失败记录。

接管精确时钟时间未提供。15:59:43 的软件 ABORTED 单独保留，不能由用户消息到达较晚推断接管发生在报错之后。路沿及全局地图障碍区域的实体关系采用现场反馈，本次未独立进行地图几何核验；不能仅凭一条控制、规划或模式日志唯一确定根因。

- 运行目录：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01`。
- 历史记录：`/home/akun/workspace/Car_jetson/navigation_monitoring/test_handoff/history/2026-11-11_15-56-01.md`；本轮报告：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/RESULT.md`。
- 日期/时区：2026-11-11，北京时间 +08:00；地图 3b2mus，任务 `task_1794383765`，路线 110→114。
- 开始15:56:01.922899；110目标接受15:56:51.379535628；软件目标失败15:59:43.322427122；idle15:59:45.529；人工程序收尾16:02:04起；录包关闭16:02:06.616576392；最后残留退出16:04:47.295989；最终清理核对16:06:23.687183。
- 用户要求直接启动、不要多余检查，本轮未重做传感器预检。录包及被动观察先于驱动和派单启动；未启用监测保护、未主动发布制动/零速/模式命令，未 force 或重复派单。
- 未修改生产源码、配置、启动文件、安装程序或系统服务，未构建/安装；写入仅在 navigation_monitoring。

## 任务阶段与现场事件

| 阶段 | 北京时间 | 证据及含义 |
|---|---|---|
| 录包/被动监测启动 | 15:56:01.923起 | bag23561、monitor23562、detail23563 |
| 实车驱动及管理入口启动 | 15:56:02.928起 | 驱动23598、管理23599 |
| 唯一派单/管理器接受 | 15:56:04.934 / 15:56:05.681503790 | dispatch114 --start110 --map3b2mus；路线仅110、114 |
| LIO ready=true / PM启动成功 | 15:56:32.109498712 / .206445238 | Bool为bag接收时间，不把任务接受当有效导航 |
| Nav2成功/生命周期就绪 | 15:56:51.351985474 / .371246409 | 管理器日志 |
| 110目标发送/接受 | 15:56:51.377016421 / .379535628 | x150.9,y−110.3,yaw1.57；action status2 |
| 行驶阶段 | 启动后，现场精确时间未知 | 用户报告严重向右转、进入路沿及其所述地图障碍区，随后人工接管 |
| 反馈首次自动false/模式1→3 | 15:58:07.633617172 / .643520026 | 速度0；仅为模式反馈，不自动等同接管精确时刻 |
| 反馈恢复自动true/模式3→1 | 15:58:17.971884542 / .981807285 | 原因未知，不推断由谁恢复或现场操作经过 |
| 再次自动false/模式1→3 | 15:58:48.844078774 / .853978078 | 速度0；之后持续至收尾 |
| 110目标ABORTED | 15:59:43.320025138 bag / .322427122 manager | 同一UUID由status2→6，无status4；无已完成航点、无备用路线，114未发送 |
| 自动关栈/idle | 15:59:43起 / 15:59:45.529 | 原程序行为；被动monitor无保护中止 |

航点110为5教门口，坐标(150.9,−110.3)，yaw1.57，type1；114为四教返航点十字路口，坐标(146.4,−43.8)，yaw1.57，type0。软件到点数0/2不等于已独立核实车辆与实体航点距离。管理器失败分支的“reason = 1 测试通过///”调试文字不是成功判据。

## 启动地图定位一致性线索

用户派单前称已到110不远处。15:56:51附近录到 map→odom 平移(156.633499,−200.492844)、yaw1.638858286rad；odom→base_link 平移(−0.354169,−0.000764)。按二维近似组合，车辆地图坐标约(156.658348,−200.846141)，与航点110配置坐标相距约 **90.729m**；下一秒样本近似结果90.732m。这与现场“110不远处”的描述存在明显一致性排查线索。

该计算使用不同时间的抽样变换：bag时间差82.687ms，header时间差269.511ms，仅保留yaw而无完整四元数，不能替代同步三维TF查询、真实位置测量或地图匹配质量评估。不能直接把90.7m写为真实车辆偏移，也不能单独证明本次入路沿由定位造成。应优先检查地图3b2mus、定位初始化、map→odom来源及配置坐标的一致性，而不是仅凭LIO启动成功判断全局定位正确。计算与原样本见 [startup_localization_consistency.json](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/startup_localization_consistency.json)。

## 规划、控制与转向证据

首条 `Starting point in lethal space` 为15:57:04.198838523；共16条，末条15:59:35.206463774。另有优化器失败15次、无有效路径7次、15:59:19.707141850无进展1次、恢复行为Collision Ahead及backup failed各5次、清全局代价地图服务超时3次、控制循环未达30Hz告警30次。它们表明行驶阶段规划/控制持续遇到问题；不能把单条lethal或碰撞日志一一对应实体路沿或接管时刻。已知恒零计数告警170条不作为激光断流证据。完整计数和时间见 [日志复核](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/log_review.md)。

从110首目标到首次模式1→3的15:56:51–15:58:07.643窗口中：

- 15:57:28.624142400，底盘转向反馈峰值+23.737503°，速度0.8km/h，模式1/自动true；15:57:25.302806787最小−15.727502°、速度1.8km/h。反馈数值正负与实体左右方向的转换尚未单独核对，不据正号自行定义右转。
- 15:57:27.639086359，cmd_vel角速度−0.341553092rad/s、线速度0.520833333m/s；上游cmd_vel_nav同段15:57:27.601624048为−0.341553092rad/s、0.444216788m/s。15:57:24.839014251，cmd_vel角速度+0.326218605rad/s、线速度0.555555556m/s。需结合底盘转角换算、速度处理、车辆轴距和时延检查，不能仅凭幅值或正负认定控制符号错误。
- 完整77个逐秒窗口、实际odom与TF样本及精确采样时间，已保存在 [离线审计JSON](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/offline_sensor_audit.json) 的 startup_motion_window，未插值伪造全局轨迹。

全量底盘反馈36,016条，速度峰值2.1km/h、P99约2.0km/h；5秒监测快照峰值仅2.0，报告采用全量峰值。cmd_vel 2,930条，正向线速度上限0.555555556m/s（2km/h），角速度−0.433292091至+0.326218605rad/s；cmd_vel_nav347条，上限0.563607037m/s。两话题均无负linear.x消息，不能据恢复行为名称断言发生实体倒车。

模式3/自动false时仍出现非零cmd_vel，例如15:59:23监测快照；这不等于车辆执行了指令。ABORT前后各10秒均有1,000条底盘反馈，速度全0、模式3/自动false；失败前10秒48条cmd_vel，末条15:59:43.082432412，失败后10秒无命令。最后10秒速度全0、无cmd_vel/cmd_vel_nav。不能以最终ABORT代替此前危险行驶的事故描述，也不能由模式变化唯一断定人为导致任务失败。

## IMU、LIO与传感器

IMU35,964条，bag接收约100.067574Hz；非递增header0、非有限payload/covariance0、异常四元数0，范数范围0.999999932–1.000000140。启动15:56:05.496498611有1次 **IMU** CRC16错误，15:56:05.497058767有330ms header跳变，而相邻bag仅隔66.171μs，不写成330ms实体断流。除这次启动跳变外，无>30ms header间隔；最大bag到达间隔46.364ms。

header持续领先bag接收，中位约381.332ms，最大领先402.794ms；bag时间不是精确驱动callback时间。与前轮不同，本轮没有录到坏四元数，但时间映射问题仍值得检查。本轮未采串口原始字节，CRC与时间偏差不作为同一唯一原因。16:02:04.892的IMU短读在16:02:04.884驱动SIGINT之后，属于收尾边界，不记为行驶中失联。

ready仅录1条true（15:56:32.109498712），没有false不能证明全过程始终true。LIO文本未见行驶期IMU大gap、GTSAM fatal或自主崩溃；由管理器在任务失败后关闭。LIO启动成功只说明该阶段通过其启动判定，不验证全局定位正确。扫描前/后/融合分别10,690/10,634/10,674条；GNSS姿态话题359条。GNSS日志有GPS Fix及卫星数，不直接套用上一轮预检无fix结论，也不把Fix当地图坐标正确证明。

monitor最高CPU温度50℃、最低可用内存24,839.4MiB。未见FastDDS锁失败日志。MQTT127.0.0.1:1拒绝属于既有本地派单入口边界；RViz GLSL错误属于渲染记录，两者不作为已证实根因。

## 结束导航与残留处理

16:02:04.883向本轮驱动组23598发SIGINT，.892停止管理组23599，16:02:06.511/.520停止观察器23563及录包23561；monitor已于16:00:01自行退出。仅停止本聊天启动的进程，未发布额外车辆制动、零速或模式命令。

管理器15:59:43.826声称Nav2及子进程已关闭，但16:03:48复核发现本轮PPID1孤儿nav2_costmap_2d_cloud PID24214/PGID24195/start_ticks1249753。16:04:47.195513核实PID、start_ticks、PGID及exe后，仅向该PID发SIGINT，.295989确认退出，无需SIGTERM/SIGKILL。初始残留证据及最终释放证据均保留，没有用最终状态覆盖最初发现。

最终16:06:23.687183，本轮所有目标组23561/23562/23563/23598/23599/23892/24195均无成员，可见活跃ROS/导航/传感器/helper/daemon候选0。FastDDS125个对象文件仍在，未删除；可见FD/mmap持有0/0、匹配锁0。文件存在本身不等于DDS故障。独立thermal PID1446/start_ticks5651保持原身份运行，未操作。boot ID `d53fafeb-91ac-4ceb-8254-8bb737042ac6` 与本轮开始一致。

## 录包及代码交接

录包正常关闭：13分片、152,304消息、4,595,630,080字节；覆盖15:56:03.604578002–16:02:04.883350246，共361.278772244秒。13个SQLite quick_check均通过，逐topic/逐shard计数与metadata一致，反序列化错误0；只在recorder退出后以mode=ro、immutable及query_only审计，无回放、无reindex。核心bag没有完整原始点云/深度图/地图图像，不能据此完整重放全部LIO输入或独立证实全局地图障碍几何。

建议代码聊天按以下证据检查，不在测试聊天直接改生产代码：

1. 全局定位初始化及地图坐标一致性：`/home/akun/workspace/Car_jetson/nav2/src/nav2_init_pose/nav2_init_pose/nav2_init_pose.py`，以及3b2mus地图、初始位姿、map→odom来源。优先核对首目标时约90.7m地图距离与现场在110附近描述。
2. 规划/恢复及转向链：Nav2全局/局部costmap、足迹/膨胀、MPPI可行轨迹、恢复行为，与cmd_vel_nav→cmd_vel→底盘转角及速度转换时序；对应启动右转、lethal起点及23.74°反馈窗口。保留地图定位与控制两类可能，未指定唯一根因。
3. FDILink时间映射及帧校验：`/home/akun/workspace/Car_jetson/real_ws/src/driver/imu/fdilink_ahrs_ROS2/src/ahrs_driver.cpp`；本轮330ms header跳变及持续约381ms未来偏差。前轮AHRS缓存问题的只读复核仍保留，本轮CRC类型是IMU，不混写成AHRS。
4. 进程回收与日志目录：`/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/process_manager.py`；Nav2 cloud孤儿漏清理，及Nav2日志仍写旧run路径。
5. 任务结果表述：`/home/akun/workspace/Car_jetson/multi_map_navigation/src/multi_map_navigation/multi_map_navigation/navigation_manager.py` 失败分支误导调试文字及中断状态；本轮失败不能因“测试通过///”判通过。

Nav2原日志仍写于 `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_16-42-07/nav2_2026-11-11_15-56-32.log`，副本已放本轮目录，未改生产日志指针。关键事件±10秒文本窗口包含任务接受、IMU CRC、目标接受、首次lethal、转向反馈峰值、三次模式变化、ABORTED及人工程序收尾；实体入路沿/接管精确时间未知，未伪造该事件中心。

- [现场及运行状态](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/run_state.json)、[日志复核](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/log_review.md)、[被动监测汇总](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/monitor_summary.json)。
- [离线审计](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/offline_sensor_audit.md)、[完整数据](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/offline_sensor_audit.json)、[地图定位近似计算](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/startup_localization_consistency.json)。
- [事件窗口索引](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/event_windows_index.json)、[录包汇总](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/recording_summary.json)、[bag元数据](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/navigation_core_bag/metadata.yaml)。
- [初始残留记录](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/cleanup_verification.md)、[残留停止操作](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/nav2_cloud_residual_termination.json)、[最终清理核对](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/final_cleanup_verification.md)、[证据清单](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/evidence_manifest.json)。
- 前轮113→106→107成功及到107后转场接管记录：[历史记录](/home/akun/workspace/Car_jetson/navigation_monitoring/test_handoff/history/2026-11-11_15-20-04.md)。两轮现场接管原因不同，保持分别记录。

归档时间：2026-11-11T16:19:24.048239+08:00。
