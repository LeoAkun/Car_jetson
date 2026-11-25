# 3b2mus 实车测试：110→114

本轮进入有效导航，但首目标110在运行约120秒后返回ABORTED，路线完成0/2；110未成功到达，114未下发。系统随后自动关闭Nav2/LIO并回到idle。本轮没有收到人工接管报告，不将其与15:56轮“严重右转入路沿后接管”混写。

- 运行目录：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_16-30-04`。
- 历史记录：`/home/akun/workspace/Car_jetson/navigation_monitoring/test_handoff/history/2026-11-11_16-30-04.md`。
- 用户指令：直接启动110→114，车辆已在110不远处；按要求未做额外传感器或残留预检。
- 地图/任务：3b2mus，task_1794385920；110为5教门口 `(150.9,-110.3,1.57)`，114为四教返航点十字路口 `(146.4,-43.8,1.57)`。
- 本聊天未启用监测保护，未主动发布制动、零速或模式指令，未使用force，任务只发布一次。派单工具漏到短暂的running状态而报确认超时，但管理器、独立monitor和Nav2均记录任务已经执行，因此没有重复派单。

## 任务时序

| 阶段 | 北京时间（2026-11-11 +08:00） |
|---|---|
| 本轮记录建立 | 16:30:04.928558521 |
| bag首消息/驱动运行 | 16:31:57.476510257起 |
| 管理器接收任务、running | 16:32:00.436422325 / 16:32:00.442 |
| LIO ready=true / PM启动成功 | 16:32:22.009215299 / 16:32:22.504048916 |
| Nav2启动成功/生命周期就绪 | 16:32:40.388204965 / 16:32:40.430164416 |
| 110目标发送/接受 | 16:32:40.438964432 / 16:32:40.444464584 |
| action status 2→6 | 16:32:40.439829345 → 16:34:40.434740437 |
| 管理器中止/回到idle | 16:34:40.438093974 / 16:34:42.645 |
| bag正常关闭 | 16:35:16.634691255 |
| 本轮进程清理完成 | 16:37:24前 |

管理器明确记录“尚无已完成航点，无法请求备用路线”，所以114没有进入Nav2。本轮应记为有效导航失败，不是启动失败，也不是路线成功。

## 规划与控制

Nav2记录22次 `Starting point in lethal space`、10次优化器无法计算路径、1次 `Failed to make progress`、1次 `Collision Ahead`、1次清全局代价地图服务超时，以及18次控制循环未达到期望频率。首条lethal-start发生在16:32:44.801736359，紧随110目标接受；这些记录与用户所述“车辆在110不远处”之间存在地图/定位/代价地图一致性检查需求，但单轮日志不能唯一归因于定位、地图、障碍或控制。

底盘反馈19,807条，速度峰值2.1km/h、P99为2.0km/h；16:32:45.321914575由模式3/非自动切换为模式1/自动，之后没有记录人工模式切换。`/cmd_vel` 1,872条，其中负linear命令755条、最小−0.555556m/s；`/cmd_vel_nav` 214条，其中负linear命令78条、最小−0.561353m/s。负命令表示恢复/控制输出，不能单独证明车辆实际倒车；监测快照中后段多次反馈速度为0。任务失败时底盘反馈速度为0，monitor最终记录状态6、idle、fault=null、protective_abort=false。

本轮规划日志仍被生产程序写入旧目录 `/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_16-42-07/nav2_2026-11-11_16-32-22.log`；副本已保存为本轮 `nav2_2026-11-11_16-32-22.log`，未改生产日志指针。

## IMU、LIO与录包

关闭后只读审计得到IMU 19,751条，bag接收频率约100.021Hz；header非递增0，异常/非有限四元数0。启动有1次IMU CRC错误及1次330ms header跳变，随后没有>30ms header间隔；header领先bag中位约357.406ms。ready只录到1条true，不能据无false证明持续恒true。LIO日志未见GTSAM fatal或自主崩溃，任务失败后由管理器关闭。

录包7分片、91,456条、约2.94GiB，覆盖16:31:57.476510257–16:35:16.634691255，共199.158180998秒。7个SQLite `quick_check`全部通过，逐topic、逐shard计数与metadata一致，反序列化错误0；只在recorder关闭后以只读immutable模式审计，无回放、无reindex。核心bag没有完整原始点云/深度/地图图像，不能独立完整重放LIO或验证实体障碍几何。

## 收尾

16:35:16开始停止本轮管理、观察、录包和驱动。实车launch在两次SIGINT后仍存活并重新拉起激光合并/IMU子进程；核对PID27839、PGID27839及start_ticks1462100后，于16:37:19向该本轮进程组发SIGTERM，随后全部退出。最终本轮bag、monitor、detail observer、dispatch、management和real drivers的PID均退出，未发现Nav2/LIO/re_localization/nav2_init_pose残留；独立thermal PID1446保持运行。

本轮没有修改生产源码、配置、启动文件、安装程序或系统服务，没有构建安装。建议代码聊天继续检查：3b2mus下110附近的map→odom与代价地图一致性、起点落入lethal区的足迹/膨胀参数、MPPI恢复与负向命令链、process_manager及实车launch子进程回收、Nav2日志目录、IMU时间映射。

- [run_state](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_16-30-04/run_state.json)
- [monitor原始记录](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_16-30-04/monitor.raw.jsonl)
- [bag离线审计](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_16-30-04/offline_sensor_audit.md)
- [完整审计数据](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_16-30-04/offline_sensor_audit.json)
- [Nav2日志副本](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_16-30-04/nav2_2026-11-11_16-32-22.log)
- [录包metadata](/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-11_16-30-04/navigation_core_bag/metadata.yaml)

归档时间：2026-11-11T16:41:20+08:00。
