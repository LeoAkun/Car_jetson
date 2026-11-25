# 110→114 本轮日志复核

复核时间：2026-11-11T16:05:20+08:00。只读取文本日志；未查询任何 SQLite、操作节点/进程/参数或改动生产文件。本文件为本子任务唯一写入。

本轮软件任务 task_1794383765、地图 3b2mus，路线为 110（5教门口，x=150.9,y=-110.3,yaw=1.57）→114（四教返航点十字路口，x=146.4,y=-43.8,yaw=1.57）。结果分类：有效导航中严重右转入路沿（现场称属于全局地图障碍区域）后人工接管，完成 0/2，系统后报首目标 ABORTED（状态6）；114 未发送、未接受。用户现场原话：“启动后不久，严重向右转向行驶，进入了路沿，属于全局地图中的障碍区域。然后接管了小车。”人工接管精确时间未知，不能写成发生在软件 ABORT 之后；模式反馈时间仅为旁证，不等于精确接管时刻。主代理说明 16:02:04 开始停止本轮组并闭包，这是接管后的程序收尾时间。

## 阶段与任务证据

- 15:56:05.677 monitor 收到 waypoint_list；15:56:05.681503790 管理器收到同任务和两航点，15:56:05.682178206 开始序列。这证明任务到达管理器，不能将该时间写成 Nav2 已进入有效导航。
- 15:56:32.206445238 LIO PM 启动成功；15:56:51.351985474 Nav2 PM 启动成功，15:56:51.371246409 关键节点/生命周期就绪。
- 110 于 15:56:51.377016421 发送、15:56:51.379535628 接受；monitor 15:56:51.378 见 action 状态2。首个同时有有效 cmd_vel 与实际运动的监测快照是 15:56:57.203：cmd_vel=[0.5556,-0.0677]，实际速度1.8 km/h、模式1/自动true、IMU100Hz、scan29.67Hz。监测约5–6秒采样，不能将该快照当成车辆开始运动的精确时刻。
- 15:59:43.320014493 Nav2 Goal failed，15:59:43.322427122 管理器状态6失败，接着“尚无已完成航点，无法请求备用路线”并主动关栈。15:59:45.529 monitor 见 idle；不是路线成功。
- 管理器虽有 debug 字符串“reason = 1 测试通过///”，上下文明确为失败/中止，不能把此 debug 文本作为成功判据。

- 15:56:05.681503790，management_stack.log:24：`[navigation_manager-4] [INFO] [1794383765.681503790] [navigation_manager]: 收到航点列表: task_id=task_1794383765, 航点列表: waypoint_list[multi_map_navigation_msgs.msg.Waypoint(name='5教门口', id=110, map_name='3b2mus', lng=103.988758, lat=30.764288, x=150.9, y=-110.3, yaw=1.57, type=1, next_map_name='', next_x=0.0, next_y=0.0, next_yaw=0.0, tolerance=0.5), multi_map_navigation_msgs.msg.Waypoint(name='四教返航点十字路口', id=114, map_name='3b2mus', lng=103.988348, lat=30.764701, x=146.4, y=-43.8, yaw=1.57, type=0, next_map_name='', next_x=0.0, next_y=0.0, next_yaw=0.0, tolerance=0.5)]total_waypoints=2`
- 15:56:05.682178206，management_stack.log:26：`[navigation_manager-4] [INFO] [1794383765.682178206] [navigation_manager]: 开始导航序列，共 2 个航点`
- 15:56:05.682711413，management_stack.log:27：`[navigation_manager-4] [INFO] [1794383765.682711413] [navigation_manager]: 正在导航到航点 1/2: type=1, map=3b2mus`
- 15:56:32.206445238，management_stack.log:78：`[process_manager-1] [INFO] [1794383792.206445238] [process_manager]: LIO-SAM启动成功`
- 15:56:51.351985474，management_stack.log:121：`[process_manager-1] [INFO] [1794383811.351985474] [process_manager]: Navigation2启动成功`
- 15:56:51.371246409，management_stack.log:122：`[navigation_manager-4] [INFO] [1794383811.371246409] [navigation_manager]: Navigation2关键节点和生命周期状态均已就绪`
- 15:56:51.377016421，management_stack.log:124：`[navigation_manager-4] [INFO] [1794383811.377016421] [navigation_manager]: 向Navigation2发送目标: x=150.90, y=-110.30, yaw=1.57`
- 15:56:51.379535628，management_stack.log:125：`[navigation_manager-4] [INFO] [1794383811.379535628] [navigation_manager]: 导航目标已接受`
- 15:59:43.322427122，management_stack.log:473：`[navigation_manager-4] [ERROR] [1794383983.322427122] [navigation_manager]: 导航目标失败，状态码: 6`
- 15:59:43.322766119，management_stack.log:474：`[navigation_manager-4] [WARN] [1794383983.322766119] [navigation_manager]: 导航序列已中止`
- 15:59:43.323270021，management_stack.log:477：`[navigation_manager-4] [ERROR] [1794383983.323270021] [navigation_manager]: 尚无已完成航点，无法请求备用路线`
- 15:59:43.323476818，management_stack.log:478：`[navigation_manager-4] [INFO] [1794383983.323476818] [navigation_manager]: 导航结束，关闭所有进程`
- 15:59:43.826645510，management_stack.log:481：`[process_manager-1] [INFO] [1794383983.826645510] [process_manager]: navigation2 及其子进程已关闭`
- 15:59:44.377362694，management_stack.log:484：`[process_manager-1] [INFO] [1794383984.377362694] [process_manager]: liosam 及其子进程已关闭`

## 规划与控制关键异常

Nav2 独立日志实际仍写到旧 run 目录：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_16-42-07/nav2_2026-11-11_15-56-32.log`，该路径由本轮 management_stack.log:82 明示。本子任务没有改动日志指针或移动文件。

| 警告/错误 | 条数 | 首条（上海） | 末条（上海） |
|---|---:|---|---|
| 控制器无进展 | 1 | 15:59:19.707141850 | 15:59:19.707141850 |
| 优化器失败 | 15 | 15:57:07.107371643 | 15:59:42.109647027 |
| 起点致命区 | 16 | 15:57:04.198838523 | 15:59:35.206463774 |
| 没有有效路径 | 7 | 15:58:07.988489681 | 15:59:43.276988833 |
| 倒退检测前方碰撞 | 5 | 15:58:07.540507337 | 15:59:42.140587695 |
| backup failed | 5 | 15:58:07.540553932 | 15:59:42.140649511 |
| 清图服务超时 | 3 | 15:57:30.277907743 | 15:59:35.269920579 |
| 控制循环未达30Hz | 30 | 15:56:51.898513793 | 15:59:40.802221780 |
| 有效点数0计数警告 | 170 | 15:56:49.923810338 | 15:59:42.861234432 |

这些是日志事件次数，不是全部周期/失败尝试的独立统计。有效点数0沿用已知恒零计数问题，不能据此断言激光断流；Optimizer 错误不能单独证明旧 MPPI/GoalCritic 补丁是否部署。软件日志里的 Collision Ahead 是规划/行为服务基于其环境表征的判定；现场右转进入路沿及随后接管采用上述用户明确描述，不能把某条 Collision Ahead 与实体事件或接管时刻直接一一对应。

- 15:57:30.277907743，Nav2:97：`[component_container_isolated-1] [WARN] [1794383850.277907743] [bt_navigator_navigate_to_pose_rclcpp_node]: Node timed out while executing service call to global_costmap/clear_entirely_global_costmap.`
- 15:58:07.540507337，Nav2:148：`[component_container_isolated-1] [WARN] [1794383887.540507337] [behavior_server]: Collision Ahead - Exiting DriveOnHeading`
- 15:58:34.440403979，Nav2:202：`[component_container_isolated-1] [WARN] [1794383914.440403979] [behavior_server]: Collision Ahead - Exiting DriveOnHeading`
- 15:59:19.707141850，Nav2:278：`[component_container_isolated-1] [ERROR] [1794383959.707141850] [controller_server]: Failed to make progress`
- 15:59:28.904058173，Nav2:303：`[component_container_isolated-1] [WARN] [1794383968.904058173] [behavior_server]: Collision Ahead - Exiting DriveOnHeading`
- 15:59:28.999874297，Nav2:306：`[component_container_isolated-1] [WARN] [1794383968.999874297] [bt_navigator_navigate_to_pose_rclcpp_node]: Node timed out while executing service call to global_costmap/clear_entirely_global_costmap.`
- 15:59:34.800356260，Nav2:319：`[component_container_isolated-1] [WARN] [1794383974.800356260] [behavior_server]: Collision Ahead - Exiting DriveOnHeading`
- 15:59:35.269920579，Nav2:325：`[component_container_isolated-1] [WARN] [1794383975.269920579] [bt_navigator_navigate_to_pose_rclcpp_node]: Node timed out while executing service call to global_costmap/clear_entirely_global_costmap.`
- 15:59:42.140587695，Nav2:340：`[component_container_isolated-1] [WARN] [1794383982.140587695] [behavior_server]: Collision Ahead - Exiting DriveOnHeading`
- 15:59:43.320014493，Nav2:351：`[component_container_isolated-1] [ERROR] [1794383983.320014493] [bt_navigator]: Goal failed`

## LIO、驱动与结束边界

- lio_sam_featureExtraction_24043_1794383788757.log：共 2 行；未见 IMU gap/GTSAM/fatal/FastDDS 锁异常。
- lio_sam_imageProjection_24041_1794383788757.log：共 6 行；未见 IMU gap/GTSAM/fatal/FastDDS 锁异常。
- lio_sam_imuPreintegration_24039_1794383788760.log：共 3 行；未见 IMU gap/GTSAM/fatal/FastDDS 锁异常。
- lio_sam_mapOptimization_24045_1794383788797.log：共 2 行；未见 IMU gap/GTSAM/fatal/FastDDS 锁异常。

- IMU 预积分独立日志 15:56:29.076901224 Started，15:59:43.875797509 SIGINT/SIGTERM；在管理器15:59:43.875447124要求关LIO之后。未见 LIO 自发 fatal 或 GTSAM 崩溃。
- 预积分日志启动期有重复 rosout logger 警告。没有 ready 订阅 QoS 不兼容警告；但本次文本审计无法重建 ready Bool 历史，未知不等于false或持续true。完整 IMU header/无效值/ready 证据须由闭包后的独立审计给出。
- 15:56:05.496498611，real_drivers.log:101：`[ahrs_driver_node-5] [WARN] [1794383765.496498611] [ahrs_bringup]: check crc16 failed (imu), total_crc_errors=1`
- 16:02:04.892122100，real_drivers.log:476：`[ahrs_driver_node-5] [WARN] [1794384124.892122100] [ahrs_bringup]: Short IMU serial read for frame type: received=0 expected=1 total_short_reads=1`
- 15:56:05.496498611 的 IMU CRC 警告是启动期实际记录；15:56:13 之后监测IMU大多约100Hz，只说明回调窗口，不证明所有原始间隔或数值有效。16:02:04.892122100 的串口短读处在人工 SIGINT 清理段，紧邻驱动16:02:04.884135127 signal_handler，不写成行驶期故障。
- 文本所见 MQTT 127.0.0.1:1 拒绝连接属于本轮既有本地任务接收边界；管理器已收到本地派发任务，不能把它当成这次目标终止的已证实原因。
- 管理器15:59:43.826645510声称Nav2关闭、15:59:44.377362694声称LIO关闭；monitor15:59:44.208已无LIO/Nav2目标进程。此为当时日志/快照，最终是否无孤儿与FastDDS残留以主代理清理后实核为准，本子任务不做实时进程检查。
- monitor protection_enabled=false，16:00:01.207 terminal 记录 protective_abort=false，fault=null；因此该监测器没有报告保护中止。现场已经明确异常行驶后接管，软件后报失败；车辆模式变化不能独立补出精确接管时刻或软件故障的唯一根因。

## 模式变化与末次状态

- 监测快照首次见模式3/自动false为15:58:09.198，15:58:20.199又见模式1/自动true，15:58:49.202再见模式3/自动false并持续至末次快照。这些只证明已采样车辆反馈变化，可作现场接管描述的旁证，不能唯一定位精确接管时刻。
- 在模式3时仍有非零 cmd_vel 发布，例如15:59:23.199 cmd_vel=[0.5556,-0.4036]，车速0；指令发布与实际运动要分开解释。
- 末次快照16:00:00.207：idle、状态[6]、实际车速0、模式3/自动false、cmd_vel回调频率0、最后值[0,0]且陈旧17.11秒；IMU100Hz、scan29.8Hz，导航/定位进程列表为空、管理器仍在。它不能反映16:02:04清理完成后的实时情况。
- monitor 16:00:01.207以idle终态退出，16:00:01.213 reason=rclpy_shutdown；早于主代理16:02:04人工收尾。

```json
[
  {
    "time": "2026-11-11T15:56:02.192+08:00",
    "event": "monitor_started",
    "boot_id": "d53fafeb-91ac-4ceb-8254-8bb737042ac6",
    "pid": 23562,
    "protection_enabled": false
  },
  {
    "time": "2026-11-11T15:56:03.554+08:00",
    "event": "robot_state_changed",
    "state": "idle"
  },
  {
    "time": "2026-11-11T15:56:05.677+08:00",
    "event": "waypoint_task",
    "task_id": "task_1794383765",
    "total_waypoints": 2,
    "start_map": "3b2mus",
    "waypoint_ids": [
      110,
      114
    ]
  },
  {
    "time": "2026-11-11T15:56:05.681+08:00",
    "event": "task_started",
    "source": "waypoint_list",
    "task_id": "task_1794383765"
  },
  {
    "time": "2026-11-11T15:56:05.682+08:00",
    "event": "robot_state_changed",
    "state": "running"
  },
  {
    "time": "2026-11-11T15:56:51.378+08:00",
    "event": "goal_status_changed",
    "statuses": [
      2
    ]
  },
  {
    "time": "2026-11-11T15:59:43.320+08:00",
    "event": "goal_status_changed",
    "statuses": [
      6
    ]
  },
  {
    "time": "2026-11-11T15:59:45.529+08:00",
    "event": "robot_state_changed",
    "state": "idle"
  },
  {
    "time": "2026-11-11T15:59:46.216+08:00",
    "event": "terminal_candidate",
    "kind": "idle"
  },
  {
    "time": "2026-11-11T16:00:01.207+08:00",
    "event": "monitor_terminal",
    "kind": "idle",
    "task_id": "task_1794383765",
    "fault": null,
    "protective_abort": false,
    "vehicle": {
      "speed_kmh": 0.0,
      "steer_deg": -0.338,
      "drive_mode": 3,
      "is_autodrive": false
    },
    "goal_statuses": [
      6
    ]
  },
  {
    "time": "2026-11-11T16:00:01.213+08:00",
    "event": "monitor_stopped",
    "reason": "rclpy_shutdown"
  }
]
```

## 其余启动期信息

- imageProjection 仅在15:56:29.386962274–29.684304213打印四次 Waiting for IMU data；后续未继续该提示。不能把启动等待计作持续行驶期 IMU 失联。
- Nav2:19，15:56:35.501883805：Controller period is less then model dt；Nav2:24，15:56:49.974070236：Parameter controller_server.verbose not found。均为实际配置/诊断警告，本审计未查询参数或改配置，也不据此唯一归因。
- Nav2:21，15:56:37.473865153：RViz地图纹理 GLSL link 错误，different sampler types share same texture unit；这是可视化渲染错误记录，不等同于Nav2未就绪或规划器地图无效。
- management_stack.log内nav2_init_pose的ExternalShutdown/重复shutdown异常发生于15:59:44.428开始关该进程之后；16:02:04附近management/drivers的SIGINT、KeyboardInterrupt和exit -2属于主代理人工收尾时段，不能计作先前任务中止的独立崩溃证据。
