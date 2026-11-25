# 本轮关闭 bag 离线审计

- Bag：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/navigation_core_bag`
- 范围：2026-11-11T15:56:03.604578002+08:00 → 2026-11-11T16:02:04.883350246+08:00，361.278772244 秒。
- 13 个 shard；全部 quick_check=True；实际 152304 条，metadata 152304 条；逐 shard/逐 topic 均一致=True。

## IMU

- 35964 条，记录接收频率 100.067574 Hz；首末 header 2026-11-11T15:56:05.336000330+08:00 → 2026-11-11T16:02:05.286013330+08:00。
- Header 非递增 0；>30 ms 1；>200 ms 1；最大 0.330000000 s。
- 四元数范数范围 0.999999932–1.000000140；范数超 [0.9,1.1] 0 条，超 [0.5,1.5] 0 条；非有限 payload/covariance 0/0。
- bag时间减header：最小 -0.402793638、中位 -0.381331874、P99 -0.354994860、最大 0.160203136 s。该量不等于精确 host callback 延迟。

## Readiness 与 action

- Ready `True`：2026-11-11T15:56:32.109498712+08:00（无 header）。
- `/navigate_to_pose/_action/status` UUID `b07567ab20284901afc5b5a2b94303c0`：2026-11-11T15:56:51.377788049+08:00 status=2；2026-11-11T15:59:43.320025138+08:00 status=6。

## 底盘反馈与命令

- VehicleStatus 36016 条；cur_speed km/h：最大 2.100000，P99 2.000000；记录的是底盘报告速度。
- drive_mode 分布 {3: 20662, 1: 15354}；变化 4 次；完整时刻见 JSON。
  - 2026-11-11T15:56:04.635145518+08:00：None → 3，speed=0.000000 km/h，is_autodrive=True。
  - 2026-11-11T15:56:04.872334134+08:00：3 → 1，speed=0.000000 km/h，is_autodrive=True。
  - 2026-11-11T15:58:07.643520026+08:00：1 → 3，speed=0.000000 km/h，is_autodrive=False。
  - 2026-11-11T15:58:17.981807285+08:00：3 → 1，speed=0.000000 km/h，is_autodrive=True。
  - 2026-11-11T15:58:48.853978078+08:00：1 → 3，speed=0.000000 km/h，is_autodrive=False。
- `/cmd_vel` 2930 条；linear.x<0 0 条，负最小 None m/s，首负 None、末负 None。负命令不能独立证明小车倒车。
- `/cmd_vel_nav` 347 条；linear.x<0 0 条，负最小 None m/s，首负 None、末负 None。负命令不能独立证明小车倒车。

## 方法与限制

- 使用 SQLite `mode=ro&immutable=1` 和 query_only；未回放、未创建 ROS 节点，未修改数据库。quick_check 会检查数据库页，但不提取/反序列化 costmap 大 BLOB。
- 不依据 bag 的 ready 消息缺失判断持续 ready；不依据 drive_mode 自动认定人工接管时刻。
- 用户反馈：“启动后不久，严重向右转向行驶，进入了路沿，属于全局地图中的障碍区域。然后接管了小车。”具体事件与接管时刻未提供；不推断唯一根因。
- 反序列化错误 0 条；完整数值、逐 shard 和 topic 计数见 `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_15-56-01/offline_sensor_audit.json`。

## ABORTED 前后 10 秒与停止前 10 秒

- 参考中心为 manager 失败日志 2026-11-11T15:59:43.322427122+08:00；action bag ABORTED 时间 2026-11-11T15:59:43.320025138+08:00。
- 失败前10秒：vehicle 1000条，speed 0.0–0.0 km/h，steer -0.33750003576278687–-0.33750003576278687°，mode分布 {3: 1000}，is_autodrive {False: 1000}，档位 {2: 1000}。
  - /cmd_vel：48条，linear.x 0.000000000–0.187678173 m/s，angular.z 0.000000000–0.022318237 rad/s，末消息 2026-11-11T15:59:43.082432412+08:00。
  - /cmd_vel_nav：4条，linear.x 0.000000000–0.187678173 m/s，angular.z 0.000000000–0.022318237 rad/s，末消息 2026-11-11T15:59:42.109780712+08:00。
- 失败后10秒：vehicle 1000条，speed 0.0–0.0 km/h，steer -0.33750003576278687–-0.33750003576278687°，mode分布 {3: 1000}，is_autodrive {False: 1000}，档位 {2: 1000}。
  - /cmd_vel：0条。
  - /cmd_vel_nav：0条。
- 最后10秒：vehicle 1000条，speed 0.0–0.0 km/h，steer -0.04500000551342964–-0.04500000551342964°，mode分布 {3: 1000}，is_autodrive {False: 1000}，档位 {2: 1000}。
  - /cmd_vel：0条。
  - /cmd_vel_nav：0条。

| 失败 ±10s 定位话题 | 条数 | 首末位姿帧 | 净 XY 位移 m | 最大相邻 XY 步长 m | 最大相邻 yaw 步长 rad |
|---|---|---|---|---|---|
| /lio_sam/mapping/odometry | 24 | odom → odom_mapping | 0.005202316 | 0.003290327 | 0.000116259 |
| /lio_sam/mapping/odometry_incremental | 24 | odom → odom_mapping | 0.005206256 | 0.003289926 | 0.000116229 |
| /odom | 1055 | odom → odom_imu | 0.002596674 | 0.017863810 | -0.001139563 |

- 完整位姿端点、TF摘要、逐秒聚合在 JSON abort_window；仅描述记录变化，不判定定位根因。
- 仅凭 drive_mode/is_autodrive/档位记录，不能判断人为造成 ABORTED；用户已补充严重右转入路沿后接管，具体时刻仍未知。

## 指令幅值与车辆模式补充

- /cmd_vel：linear.x 0.000000000–0.555555556 m/s，angular.z -0.433292091–0.326218605 rad/s。
- /cmd_vel_nav：linear.x 0.000000000–0.563607037 m/s，angular.z -0.433292091–0.326218605 rad/s。
- is_autodrive 初始值与变化：
  - 2026-11-11T15:56:04.635145518+08:00：None → True（drive_mode=3，speed=0.000 km/h）。
  - 2026-11-11T15:58:07.633617172+08:00：True → False（drive_mode=1，speed=0.000 km/h）。
  - 2026-11-11T15:58:17.971884542+08:00：False → True（drive_mode=3，speed=0.000 km/h）。
  - 2026-11-11T15:58:48.844078774+08:00：True → False（drive_mode=1，speed=0.000 km/h）。
- shift_level 初始值与变化：
  - 2026-11-11T15:56:04.635145518+08:00：None → 2（drive_mode=3，speed=0.000 km/h）。
  - 2026-11-11T15:56:51.955204141+08:00：2 → 1（drive_mode=1，speed=0.000 km/h）。
  - 2026-11-11T15:57:07.254615194+08:00：1 → 2（drive_mode=1，speed=0.600 km/h）。
  - 2026-11-11T15:57:07.554885447+08:00：2 → 1（drive_mode=1，speed=0.300 km/h）。
  - 2026-11-11T15:57:11.754434853+08:00：1 → 2（drive_mode=1，speed=0.900 km/h）。
  - 2026-11-11T15:57:12.054915878+08:00：2 → 1（drive_mode=1，speed=0.000 km/h）。
  - 2026-11-11T15:57:30.505207379+08:00：1 → 2（drive_mode=1，speed=0.400 km/h）。
  - 2026-11-11T15:57:35.855190512+08:00：2 → 1（drive_mode=1，speed=0.000 km/h）。
  - 2026-11-11T15:58:07.553578902+08:00：1 → 2（drive_mode=1，speed=0.000 km/h）。
  - 2026-11-11T15:58:07.603694743+08:00：2 → 1（drive_mode=1，speed=0.000 km/h）。
  - 2026-11-11T15:58:07.643520026+08:00：1 → 2（drive_mode=3，speed=0.000 km/h）。
  - 2026-11-11T15:58:09.363136630+08:00：2 → 3（drive_mode=3，speed=0.000 km/h）。
  - 2026-11-11T15:58:15.272299559+08:00：3 → 2（drive_mode=3，speed=0.000 km/h）。
  - 2026-11-11T15:58:17.991687166+08:00：2 → 1（drive_mode=1，speed=0.000 km/h）。
  - 2026-11-11T15:58:20.302613669+08:00：1 → 2（drive_mode=1，speed=0.300 km/h）。
  - 2026-11-11T15:58:24.751851802+08:00：2 → 1（drive_mode=1，speed=0.000 km/h）。
  - 2026-11-11T15:58:34.462261284+08:00：1 → 2（drive_mode=1，speed=1.300 km/h）。
  - 2026-11-11T15:58:34.502244048+08:00：2 → 1（drive_mode=1，speed=1.300 km/h）。
  - 2026-11-11T15:58:35.052010060+08:00：1 → 2（drive_mode=1，speed=0.700 km/h）。
  - 2026-11-11T15:58:35.552014792+08:00：2 → 1（drive_mode=1，speed=0.000 km/h）。
  - 2026-11-11T15:58:37.051813512+08:00：1 → 2（drive_mode=1，speed=0.500 km/h）。
  - 2026-11-11T15:58:37.251861677+08:00：2 → 1（drive_mode=1，speed=0.300 km/h）。
  - 2026-11-11T15:58:48.853978078+08:00：1 → 2（drive_mode=3，speed=0.000 km/h）。
  - 2026-11-11T16:00:19.633830617+08:00：2 → 3（drive_mode=3，speed=0.000 km/h）。
  - 2026-11-11T16:00:20.495454969+08:00：3 → 2（drive_mode=3，speed=0.000 km/h）。
  - 2026-11-11T16:00:20.685464043+08:00：2 → 3（drive_mode=3，speed=0.000 km/h）。
  - 2026-11-11T16:00:48.403650102+08:00：3 → 2（drive_mode=3，speed=0.000 km/h）。

## 启动行驶阶段与现场反馈

- 用户原话：“启动后不久，严重向右转向行驶，进入了路沿，属于全局地图中的障碍区域。然后接管了小车。”事件/接管精确时间未知，不能写接管发生在 ABORTED 之后。
- 采样窗口 2026-11-11T15:56:51.000000000+08:00 → 2026-11-11T15:58:07.643520026+08:00；边界为首次运动目标阶段至首次 drive_mode 1→3 的反馈时刻，并不据此认定物理接管时刻。
- 转向反馈最大：23.737503052°，2026-11-11T15:57:28.624142400+08:00，speed=0.800 km/h，mode=1、is_autodrive=True。
- 转向反馈最小：-15.727501869°，2026-11-11T15:57:25.302806787+08:00，speed=1.800 km/h，mode=1、is_autodrive=True。
- 有速度时转向反馈最大：23.737503052°，2026-11-11T15:57:28.624142400+08:00，speed=0.800 km/h，mode=1、is_autodrive=True。
- /cmd_vel linear最大：2026-11-11T15:56:55.141123107+08:00，linear.x=0.555555556 m/s，angular.z=-0.010823941 rad/s。
- /cmd_vel angular最小：2026-11-11T15:57:27.639086359+08:00，linear.x=0.520833333 m/s，angular.z=-0.341553092 rad/s。
- /cmd_vel angular最大：2026-11-11T15:57:24.839014251+08:00，linear.x=0.555555556 m/s，angular.z=0.326218605 rad/s。
- /cmd_vel_nav linear最大：2026-11-11T15:57:38.599127666+08:00，linear.x=0.563607037 m/s，angular.z=0.137802318 rad/s。
- /cmd_vel_nav angular最小：2026-11-11T15:57:27.601624048+08:00，linear.x=0.444216788 m/s，angular.z=-0.341553092 rad/s。
- /cmd_vel_nav angular最大：2026-11-11T15:57:24.801616357+08:00，linear.x=0.555555582 m/s，angular.z=0.326218605 rad/s。
- 启动阶段每秒速度/转向/命令汇总、odom 与 map→odom/odom→base_link 的首个实际样本及时间保存在 JSON startup_motion_window.one_second_samples。odom 坐标不是 global map 坐标，不据此验证路沿或全局障碍区域；未读取 costmap 大 BLOB。
- 现场反馈、底盘与指令记录必须并列保留；未唯一归因于定位、规划、控制或人工操作。
