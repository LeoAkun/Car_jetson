# 2026-11-14 手动派单测试监测结果

## 结论

任务 `task_1794627438` 由现场手动发布，地图为 `map2`，航点为 `53 → 54 → 58 → 57`。系统共进行了三次启动/导航尝试，均未到达第一个航点 53，后续航点也未执行。本次测试判定失败。

最后一次自动重试的 NavigateToPose action UUID 为 `89791e869e72480c91a1e57442b677f9`，状态从 `EXECUTING (2)` 变为 `ABORTED (6)`，Nav2 最终记录 `Goal failed`。直接失败证据是反复出现 `Starting point in lethal space`，同时伴随 costmap 有效点数为 0 和全局代价地图清除超时。

## 三次尝试

1. 11:37:18 捕获任务；11:37:25 在导航动作开始前因 `nav2_init_pose: launch process exited with code 0` 进入 fault。车辆未移动。
2. 重定位、LIO-SAM 和 Nav2 成功启动。11:39:41，UUID `1109eed7a4534099812b07463c15b64a` 进入 `EXECUTING`，车辆最高约 2.0 km/h。11:40:29，进程管理器报告多个 lifecycle 节点查询超时，action 进入 `CANCELING (3)`，导航子进程随后退出。
3. 11:42:22，UUID `89791e869e72480c91a1e57442b677f9` 进入 `EXECUTING`；11:44:09 以 `ABORTED (6)` 结束。日志中 `Starting point in lethal space` 出现 20 次，并持续出现 costmap 有效点数为 0、全局 costmap 清除超时和控制周期丢失。

## 诊断

第二次尝试的录包中，两类全局 costmap 主题合计约 4.61 GiB，并与 lifecycle 集体超时处于同一时间窗口。停止录包后的第三次重试没有再次出现该 lifecycle 健康检查 fault，因此大体量 costmap 序列化和录盘负载可能是第二次超时的诱因；现有证据不足以认定为唯一原因。

第三次尝试的终止原因更明确：规划器持续认为起点处于致命占用区域。上游 IMU 约 100 Hz、原始及过滤融合激光约 30 Hz，说明不能仅据本轮现象认定为上游传感器话题中断；需要继续检查 costmap 观测源、TF 时序、机器人足迹和起点附近占用状态。

测试期间最高温度 52°C，最低可用内存约 22.2 GiB，没有温度或内存保护终止。监测端始终为只读观察，未发布任务，也未发送速度、制动或驻车指令。

## 测试结束与程序状态

现场报告已人工接管后，导航程序进程组 `13861` 已通过 SIGINT 关闭。复核结果为：导航、Nav2、LIO-SAM、重定位、任务管理及所有监测/录包进程均已退出；底盘和传感器基础驱动继续运行。

关闭前最后确认车辆速度为 0 km/h、drive mode 3、task status 0。

## 证据

- `monitor.raw.jsonl`：第二次尝试的进程、传感器、车辆和 fault 终态。
- `detail_observer.jsonl`：第二次尝试的计划、里程计和 action 状态。
- `nav2_2026-11-14_11-39-20.log`：第二次尝试的 costmap、TF 和控制周期告警。
- `nav2_2026-11-14_11-42-04.log`：第三次自动重试的规划失败和最终 `Goal failed`。
- `navigation_core_bag/`：第二次尝试的 rosbag，正常关闭，时长约 131.0 秒，共 53,026 条消息，约 4.9 GiB。
- 第三次重试发生在本目录 rosbag 停止后；其 action 与里程计旁证来自持续观察器 `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-14_11-02-27_manual_publish_monitor/detail_observer.jsonl`。
