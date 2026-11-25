# 2026-11-14 手动派单测试监测结果

## 结论

本轮任务 `task_1794626035` 由现场手动发布，地图为 `3b2mus`，航点为 `110 → 114`。任务在执行第一个 `NavigateToPose` 目标 110 时失败，action UUID `ff382058068841e68d9d54f32e0add50` 从 `EXECUTING (2)` 变为 `ABORTED (6)`；没有到达 110，也没有下发 114 的第二个导航目标。

Nav2 最终记录 `Goal failed`。失败前反复出现 `Starting point in lethal space`，恢复规划持续失败；local/global costmap 同期频繁报告缓冲区有效点数为 0。上游原始与过滤融合激光仍稳定在约 30 Hz，因而现有证据指向代价地图/变换/过滤链路或起点占用状态，不能仅据本轮日志认定为上游激光话题中断。

## 时间线

- 11:12:06：只观察监测、详细观察器和 rosbag 启动；没有由监测端发布任务或车辆控制命令。
- 11:13:55：捕获手动发布任务，航点 `[110, 114]`；`robot_state=running`，`task_status=1`。
- 11:14:30：第一个 `NavigateToPose` 目标进入 `EXECUTING`。
- 11:17:06：第一个目标变为 `ABORTED`；Nav2 记录 `Goal failed`。
- 11:17:08：机器人回到 `idle`、`task_status=0`，导航、LIO-SAM、初始位姿和重定位子进程均退出。
- 11:17:23：监测器确认 `idle` 终态持续 15 秒。
- 11:17:26：rosbag 正常停止并写出 metadata。

## 运行与安全状态

- 有效导航约 156 秒；最大实车反馈速度 2.1 km/h。
- 全局计划起点距 110 从约 82.6 m 缩短到约 36.1 m；里程计首末位置净位移约 46.6 m。
- IMU 约 100 Hz，原始与过滤融合激光约 30 Hz，导航期间 TF 约 110–114 Hz。
- 最高温度 50°C；最低可用内存约 24.1 GiB；未出现温度或内存异常。
- 最终实车速度 0、drive mode 3；没有 `process_fault`，没有监测器保护动作。

## 证据

- `monitor.raw.jsonl`：任务、进程、传感器、车辆和终态记录。
- `detail_observer.jsonl`：计划、位姿和 action UUID/状态记录。
- `nav2_2026-11-14_11-14-11.log`：代价地图告警、恢复规划失败和最终 `Goal failed`。
- `navigation_core_bag/`：正常关闭的 rosbag，时长约 319.4 秒，共 157,458 条消息，约 7.6 GiB。
