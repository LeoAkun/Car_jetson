# 本轮关闭 bag 离线审计

- Bag：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_16-30-04/navigation_core_bag`
- 范围：2026-11-11T16:31:57.476510257+08:00 → 2026-11-11T16:35:16.634691255+08:00，199.158180998 秒。
- 7 个 shard；全部 quick_check=True；实际 91456 条，metadata 91456 条；逐 shard/逐 topic 均一致=True。

## IMU

- 19751 条，记录接收频率 100.021373 Hz；首末 header 2026-11-11T16:31:59.176454190+08:00 → 2026-11-11T16:35:16.996454190+08:00。
- Header 非递增 0；>30 ms 1；>200 ms 1；最大 0.330000000 s。
- 四元数范数范围 0.999999955–1.000000118；范数超 [0.9,1.1] 0 条，超 [0.5,1.5] 0 条；非有限 payload/covariance 0/0。
- bag时间减header：最小 -0.362028489、中位 -0.357405992、P99 -0.352995242、最大 0.000440412 s。该量不等于精确 host callback 延迟。

## Readiness 与 action

- Ready `True`：2026-11-11T16:32:22.009215299+08:00（无 header）。
- `/navigate_to_pose/_action/status` UUID `12e72a11f050452d9a9b2a09708a604c`：2026-11-11T16:32:40.439829345+08:00 status=2；2026-11-11T16:34:40.434740437+08:00 status=6。

## 底盘反馈与命令

- VehicleStatus 19807 条；cur_speed km/h：最大 2.100000，P99 2.000000；记录的是底盘报告速度。
- drive_mode 分布 {3: 4675, 1: 15132}；变化 1 次；完整时刻见 JSON。
  - 2026-11-11T16:31:58.574343676+08:00：None → 3，speed=0.000000 km/h，is_autodrive=False。
  - 2026-11-11T16:32:45.321914575+08:00：3 → 1，speed=0.000000 km/h，is_autodrive=True。
- `/cmd_vel` 1872 条；linear.x<0 755 条，负最小 -0.5555555555555556 m/s，首负 2026-11-11T16:33:35.175460190+08:00、末负 2026-11-11T16:34:40.428098986+08:00。负命令不能独立证明小车倒车。
- `/cmd_vel_nav` 214 条；linear.x<0 78 条，负最小 -0.5613529086112976 m/s，首负 2026-11-11T16:33:34.763596504+08:00、末负 2026-11-11T16:34:40.568396959+08:00。负命令不能独立证明小车倒车。

## 方法与限制

- 使用 SQLite `mode=ro&immutable=1` 和 query_only；未回放、未创建 ROS 节点，未修改数据库。quick_check 会检查数据库页，但不提取/反序列化 costmap 大 BLOB。
- 不依据 bag 的 ready 消息缺失判断持续 ready；不依据 drive_mode 自动认定人工接管时刻。
- 用户已报告人工接管，但具体时刻与原因未提供；本审计不推断唯一根因。
- 反序列化错误 0 条；完整数值、逐 shard 和 topic 计数见 `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_16-30-04/offline_sensor_audit.json`。
