# 车辆导航测试与修改交接（截至 2026-11-20 12:05）

> 新聊天先完整阅读本文及文末链接。工作目录固定为 `/home/akun/workspace/Car_jetson`。当前导航栈、传感器栈、录包器和监控器均已关闭，不要把“阅读交接”当作启动授权。

## 1. 操作职责和不可违反的约束

- 操作者负责现场安全、手动启动导航管理栈、手动发布任务点和人工接管。
- 新聊天只在操作者明确说“开始测试”后启动只读监控和 rosbag；操作者说“结束测试”或报告人工接管后，停止监控和录包并核对落盘。
- 禁止发布驻车、刹车、零速、`/cmd_vel`、底盘模式、`/process_manager/fault` 或其他车辆控制命令。现场有人保护。
- 不代替操作者发布导航任务；不调用 `dispatch_navigation.py` 派单。
- 不使用 `nav_task_monitor.py --enable-protection` 或 `--stop-on-terminal`。
- 任务进入 `fault`、`idle` 或 action 终态后继续录制，直到操作者明确结束。
- 当前工作区有大量跨多轮工作的未提交改动。禁止 `git reset --hard`、`git clean`、切换分支覆盖文件或笼统回滚；先读本目录的状态快照和差异文件。

## 2. 当前关机状态

2026-11-20 收尾顺序：

1. 12:03:35 向 segment 3 rosbag 进程组发送 `SIGINT`；正常写缓存并生成 `metadata.yaml`，12:03:35.851 已退出。
2. 导航管理 launch 进程组 `448368` 收到 `SIGINT` 后正常退出；其管理节点和导航子栈均退出。
3. 传感器/整车驱动 launch 进程组 `355146` 收到 `SIGINT` 后20秒仍未完全退出，随后发送 `SIGTERM`，整组退出；未使用 `SIGKILL`。
4. 12:04:48 停止 observation-only 监控器；随后停止遗留 ROS CLI daemon。
5. `stop_navigation.sh --check-only` 最终结果：导航进程0、导航ROS节点0、ROS CLI daemon 0、独立 Fast DDS discovery 0、Fast DDS 活跃 FD/mmap/内核锁0。
6. `/dev/shm` 中仍枚举到454个 Fast DDS 历史对象文件，但没有进程持有、映射或加锁。文件名存在不等于锁故障，不要例行删除。
7. 最终 `ros2 node list --no-daemon` 为空；未发现 `launch_real`、传感器驱动、LIO、Nav2、重定位、管理器、录包器或监控器残留。

本次关闭只发送操作系统退出信号，没有发布驻车、制动、零速、故障或车辆控制消息。

关闭检查记录：

- `navigation_monitoring/shutdown_checks/2026-11-20_12-05-14/`
- 本目录 `runtime_shutdown_state.txt`

## 3. 手动测试的固定流程

新终端已通过 `~/.bashrc` 自动 source：

```bash
/home/akun/workspace/Car_jetson/navigation_tools/ros2_navigation_env.bash
```

该脚本显式加载 ROS Humble 以及 `real_ws`、`utils`、`slam`、`nav2`、`multi_map_navigation` 和根工作区 overlay，避免只 source 根 `install/setup.bash` 时因生成时 underlay 链不同而找不到 `launch_real` 或 `multi_map_navigation_msgs`。可用 `car_ros_env_check` 检查，构建后在旧终端用 `car_ros_env_reload` 刷新。

现场人工流程：

```bash
# 1. 启动整车传感器/底盘驱动
ros2 launch launch_real run.launch.py

# 2. 启动长期运行的导航管理栈
ros2 launch multi_map_navigation multi_map_navigation.launch.py

# 3. 操作者手动发布 MQTT/任务；重定位、初始位姿、LIO 和 Nav2 由管理栈按任务自动拉起
```

测试聊天不执行第3步。关闭导航的工具为：

```bash
bash navigation_tools/stop_navigation.sh
```

但该脚本正常模式会发布 `/process_manager/fault`。在“禁止发布 fault/控制消息”的现场规则下，优先向已确认身份的 launch PID/PGID 发送 `SIGINT`；`--check-only` 可安全用于残留检查。

## 4. 已实现的主要修改

### 4.1 导航会话隔离、ProcessManager 和连续任务

相关文件：

- `multi_map_navigation/.../process_manager.py`
- `multi_map_navigation/.../navigation_manager.py`
- `multi_map_navigation/.../map_switch_controller.py`
- `multi_map_navigation/.../stack_transaction_lock.py`
- `multi_map_navigation/.../map_odom_authority.py`
- `multi_map_navigation_msgs` 新增/扩展消息和服务

已实现：

- 保存进程组并验证整个 PGID 退出；父进程退出但子进程仍存活不再算清理成功。
- 启动前扫描未纳管的重定位、初始位姿、LIO、Nav2 残留；发现残留时拒绝叠加新实例。
- 加管理器所有权锁和跨管理器栈事务锁，避免启动、清理、地图切换同时操作同一导航子栈。
- 清理顺序统一为 `navigation2 → liosam → nav2_init_pose → re_localization`，状态未知或部分停止时 fail closed。
- 长启动/关闭工作从服务回调中移出，使用可重入 callback group、工作线程和事务结果，避免 ProcessManager 被阻塞后清理服务无法执行。
- 每次定位建立 UUID、地图、generation、会话开始时间；旧点云、旧 TF、旧 correction、旧 action 或旧地图结果不能被新任务复用。
- 路线地图与实际栈地图在发目标前复核；普通航点不能直接跨地图，地图变化必须通过 type=4 切换点。
- `map_odom_authority` 成为唯一 `map→odom` 发布者；`nav2_init_pose` 只发布带会话身份的定位结果。
- 目标完成、故障和下一任务之间有清理门禁；此前“导航栈未清理干净/永久 busy”问题的代码修复已经完成，但仍需继续收集连续实车样本。

详细说明：`navigation_monitoring/diagnostics/2026-11-17_navigation_session_isolation_fix.md`。

### 4.2 重定位新鲜点云、自适应双确认和静态足迹门禁

相关文件：

- `utils/src/re_localization/srv/ReLocalization.srv`
- `utils/src/re_localization/src/pcl_localization_component.cpp`
- `nav2/src/nav2_init_pose/nav2_init_pose/nav2_init_pose.py`
- `multi_map_navigation/.../static_footprint_gate.py`
- `multi_map_navigation_msgs/msg/LocalizationSession.msg`
- `multi_map_navigation_msgs/msg/LocalizationResult.msg`

规则：

- 服务请求/结果携带 session、地图、`observation_id`、`refresh_cloud` 和 `min_cloud_stamp`；新 observation 必须使用更新的点云。
- fitness `<0.025` 且先验、点云时间和完整静态 footprint 合法时直接接受。
- fitness `0.025～0.05` 时用新鲜点云做局部二次确认；两次位姿差要求 `≤0.30 m、≤3°`。
- fitness `>0.05` 只允许换新点云重试一次；单轮总时限120秒。
- 地图切换使用 `switch_pose` 做有限范围先验；GPS搜索结果也受先验范围约束。
- 完整矩形 footprint 使用约 `0.86 m × 0.56 m`，再叠加0.02 m padding和0.05 m启动余量；occupied、unknown、地图外均拒绝。
- 发首目标前要求最终 `map→base_link` 连续10个不同时间戳样本均通过静态足迹门禁。

历史58轮中51轮接受、7轮明确失败；大多数成功分数很好，但 fitness 是1.2 m内点比例，不是厘米级绝对误差。详见 `navigation_monitoring/diagnostics/2026-11-18_historical_relocalization_scores.md`。

### 4.3 位置跳变阶段 A～E 和 LIO correction 保护

核心文件：

- `slam/src/LIO-SAM/src/mapOptmization.cpp`
- `slam/src/LIO-SAM/src/imuPreintegration.cpp`
- `slam/src/LIO-SAM/include/lio_sam/utility.hpp`
- `slam/src/LIO-SAM/config/params_rslidar.yaml`
- `multi_map_navigation/.../map_odom_authority.py`
- 新消息：`EpochOdometry`、`LioCorrectionEvent`、`LioMapOdomCorrection`、`LioReadiness`

已实现：

- correction 增加 `instance_id + generation + correction_epoch + source + residual + score + accepted/reason`。
- TransformFusion 只用完全相同 identity/epoch 的 mapping correction 和高频预积分；不再用 `queue.front()` 近似 correction 时刻，也不跨 epoch 组合。
- rejected scan-to-map correction 使用同时间戳预测位姿作为 anchor，不能从 mapping odometry 绕过门禁进入 `/odom`。
- 新 epoch 建立连续性 offset；输出相邻步长超过 `0.05 m/10 ms` 或 `1°` 时拦截并报告 fault。
- 后端优化 frame 改为 `lio_map`；连续控制 frame 保持 `odom`。`map_odom_authority` 受控应用 `lio_map→odom`，并在每个插值步检查静态 footprint。
- scan-to-map 单帧 correction 硬门限保持 `0.15 m / 2°`；连续3帧 rejected 或退化触发定位故障。
- GPS factor 默认 `useGpsFactor: false`；实车完成三帧一致性验收前不得打开。
- 回环第一轮保守参数：1 Hz、历史时间差60秒、fitness 0.30、最大对应距离2 m、overlap≥0.55；增加相邻关键帧复核、重复约束抑制、`0.30 m/5°`硬拒绝和 Huber noise。

隔离域历史回放：

- 第一轮故障窗口最大 `/odom` 步长0.03996 m/10 ms，无样本超过0.05 m/1°，但99.9分位0.03951 m，仍未达到建议的0.02 m。
- 第二轮最大0.00304 m，短段最大0.00165 m。
- 该回放证明硬门限和 epoch 链路工作，不等于实车验收完成。

完整方案与实施记录：

- `navigation_monitoring/diagnostics/2026-11-18_position_jump_root_cause_and_fix_plan.md`
- `navigation_monitoring/diagnostics/2026-11-19_phase_b_d_replay_report.md`

### 4.4 2026-11-20 11:09 累计滞后误报修复

任务 `task_1795144184`（map2，13→17）已完成重定位和LIO稳定检查，但目标下发前被误终止。原逻辑把当前限速 `map→odom` 与最新目标之间累计34.31 cm滞后，当成单帧大 correction；实际 epoch 19 的 scan-to-map correction 仅4.29 cm并已被LIO接受。

已修改：

- 不再用累计目标滞后判单帧跳变。
- LIO已接受的5～15 cm correction 限速连续跟随，不再累计三次触发重定位故障。
- `map→odom` 平移跟随速度调为0.10 m/s；每一步仍检查静态 footprint。
- 保留单帧15 cm/2°硬拒绝和连续三次 rejected/退化故障。
- 移除 NavigationManager 目标和地图切换路径中的 `tf_transformations` 依赖，避免 NumPy 2 兼容问题。

验证：map authority 16项测试通过；联合回归93项通过；正式 install 已构建。实车下一轮已经证明旧累计滞后误报未再次出现。详情见 `navigation_monitoring/diagnostics/2026-11-20_1109_navigation_failure_analysis_and_fix.md`。

### 4.5 2026-11-20 11:37 静止 IMU 漂移和零速更新

任务 `task_1795145873`（map2，13→17）重定位成功，fitness `0.011950`。车辆速度与 `/cmd_vel` 均为0、Nav2目标尚未发送时，scan-to-map 依次需要18.09、16.94、21.54 cm修正，连续超过15 cm门限后触发 `LIO_SCAN_TO_MAP_UNRELIABLE`。特征点充分且无退化；后续 correction 最大1.825 m。

现场静止 IMU 姿态补偿后仍有约 `0.0151 m/s²` 水平加速度残差；原初速度先验 sigma 为 `1e4 m/s`，没有使用底盘静止反馈，Nav2启动负载带来的 correction 间隔放大了积分漂移。

已在 `imuPreintegration.cpp` 增加只读底盘零速更新：

```yaml
useVehicleZeroVelocityUpdate: true
vehicleStationarySpeedThresholdKmh: 0.10
vehicleStatusTimeoutSec: 0.25
vehicleStationaryMinSamples: 5
zeroVelocitySigma: 0.05
```

- 只有新鲜 `/vehicle_status` 连续5帧低于0.10 km/h才加入零速度因子。
- 车辆移动后立即清零静止计数；反馈超过0.25秒不使用约束。
- 只订阅状态，不发送任何车辆命令。
- 15 cm/2°硬门限保持不变。

已构建正式 install；隔离 ROS 域回放中 correction 周期的速度由约0.009～0.061 m/s压至0.004～0.040 m/s。**此修复尚未经过下一轮实车任务验证。** 下一次 LIO 子进程启动会加载新二进制，无需为了该修改重启长期管理器；当前管理器已经按本次收尾关闭。

详情见 `navigation_monitoring/diagnostics/2026-11-20_1137_stationary_imu_drift_failure_and_fix.md`。

### 4.6 雷达近车体回波和过滤

第一组前雷达过滤条件仍存在：

- 来源：前雷达
- 原始角度132°～135°
- 原始距离0.60～0.83 m
- 转换后 `x=-0.40～-0.18 m，y=0.28～0.33 m`

扫描融合生成独立 filtered scan：全局代价地图使用 `/laser/scan_merge_laser_filtered`；局部代价地图和近场保护仍使用原始 `/laser/scan_merge_laser`。

此前操作者怀疑这可能来自车上悬挂线束，因此暂不把它作为默认根因。退役规则要求连续至少3次独立测试、累计至少3600秒有效前雷达数据均零命中，只提醒、不自动删除。

截至本次收尾，tracker 在11月19日和11月20日都检测到大量候选点，最近一次为12:04:48，因此零命中计数为0，**当前不满足删除条件**。后续需要结合现场线束状态和录包确认这些候选是不是同一类自车回波。

规则和状态：

- `navigation_monitoring/FILTER_RETIREMENT_POLICY.md`
- `navigation_monitoring/filter_retirement_tracker.json`

### 4.7 Nav2 第一轮安全余量修改

`nav2_real_params.yaml` 当前关键值：

- controller 频率30 Hz→20 Hz，与 `model_dt=0.05` 对齐。
- 车体 footprint 全局/局部统一为约0.86 m×0.56 m，padding 0.02 m。
- 局部动态障碍 inflation：0.85 m，cost scaling 3.5。
- 全局静态障碍 inflation：0.75 m，cost scaling 3.5。
- MPPI ObstaclesCritic 对齐0.85 m/3.5。
- Smac、MPPI和底盘转换统一保守最小转弯半径1.0 m。
- 普通导航速度上限约0.556 m/s（2 km/h），单位已统一为SI。

这是操作者要求的“第一轮修改”。目的在于更早对人员和临时障碍作出反应，并提高路线远离静态边界的倾向。还没有足够实车A/B样本确认对狭窄道路可通行性的影响。

### 4.8 导航退出后持续制动修复

`cmdvel_2_ecu` 已改为：

- 普通零速、非法输入和输入超时不设置 `Ecu.brake`；JD03中该位同时映射制动模式和电子驻车，因此固定 `brake=false`。
- 输入超时后只发送一次 `motor=0 + N挡 + brake=false` 的中性停止帧，随后停止输出，避免导航退出后持续请求自动模式。
- 人工模式下停止占用底盘输出。

该修改有单元测试和代码审计，仍需在现场明确验证“结束导航后人工可正常接管/解除”。本聊天和后续聊天都不得主动发布驻车命令。

## 5. 已查明的历史现象

- “全局代价地图”是运行时 costmap，不等同于预先建好的静态PGM；它由静态层、动态障碍层和膨胀层组成。
- 11月14日11:13的1.179 m/10 ms位置跳变发生在重定位以后，`map→odom`不变，属于LIO/TransformFusion运行中 correction epoch 问题。
- 11月18日车辆靠边和进入静态障碍区，主要链路为估计位姿/路径迁移后 footprint 覆盖静态占据格；不能把所有历史失败都归因于前雷达回波。
- 红圈位置附近航点不在红圈中心；审计时红圈中心本身落在静态致命格附近，最近目录航点约27.6 m远。相关图和坐标见 `navigation_monitoring/diagnostics/2026-11-18_red_circle_audit/`。
- 11:53任务的第一目标确实是航点15，路径包含Reeds-Shepp反向段，因此开始倒车不等于跳过起点。详情见 `2026-11-18_map2_start_legality/README.md`。
- 11月16日跑一半倒车失败的直接链为控制器无法生成轨迹→起点致命→BT BackUp→倒车碰撞检查失败→原路线和备用路线均ABORTED；当轮录包不足以唯一确定最初致命格来源。
- 11月17日地图切换失败的直接点是map2 Nav2 composable nodes加载服务超时；旧定位服务实例和阻塞式ProcessManager放大后续问题。这些生命周期问题已按4.1修改。

## 6. 当前仍未完成或必须继续验证

以下项目在当前源码中仍存在，新聊天不得写成“已经解决”：

1. **最新零速度更新未做实车闭环验证。** 下一轮首先原地启动、只定位，观察 `LIO_ZERO_VELOCITY_UPDATE`、correction residual和是否仍出现连续 rejected。
2. **普通导航行为树仍包含 BackUp。** `general_nav_to_pose.xml` 和 `general_nav_through_poses.xml` 仍有0.30/0.50 m盲倒恢复。起点非法时应直接失败/重定位的专用分支尚未落地。
3. **`default_server_timeout` 当前仍为30 ms。** 历史清图超时会放大等待与倒车循环；计划值1000 ms尚未写入当前参数。
4. **全局 costmap 当前仍 `always_send_full_costmap: true`。** 7404×8579地图全量发布约63.5 MB/帧，会增加DDS、CPU和录包I/O；计划改为false并在故障时按需保存完整帧，尚未落地。
5. **位置连续性建议指标尚未全部达到。** 历史回放通过0.05 m/10 ms硬门限，但第一轮99.9分位约0.0395 m，未达到建议0.02 m。
6. **GPS factor保持关闭。** 三帧一致性、协方差和innovation门禁尚未实车验收。
7. **膨胀半径需实车A/B。** 观察人员绕行距离、路径是否更居中、狭路是否还能规划，以及是否增加不必要停顿。
8. **RViz曾在Nav2启动时以-6退出。** Nav2随后仍通过健康检查，因此不是11:37失败原因；若启动负载继续影响LIO correction间隔，可考虑实车运行默认不拉RViz。
9. **传感器栈本次SIGINT未在20秒内完全退出。** SIGTERM后全部退出。后续应记录哪个子进程拖延，完善专用传感器关闭脚本，避免宽泛 `pkill`。
10. **第一组雷达过滤不能删除。** tracker本次仍有候选命中；先确认现场线束和候选点是否随车体移动。

## 7. 下一轮实车验证顺序

1. 操作者启动传感器栈和管理栈；新聊天在“开始测试”后启动 observation-only monitor 和完整 rosbag。
2. 第一轮只原地定位，不发移动目标，保持至少2～5分钟：确认零速更新持续工作、scan-to-map correction不超过15 cm、无 correction storm。
3. 空旷低速直线和缓弯：检查 `/odom` 相邻10 ms步长、`map→odom`限速跟随、路线与车体位置是否连续。
4. 短闭环路线重复3次：核对回环约束、generation/epoch、静态footprint和目标状态。
5. 复跑此前靠边路段：对比首条plan和最新plan的横向迁移，确认footprint不进入静态占据格。
6. 连续发布两个任务：确认前一任务子栈、action、TF和session不被复用，ProcessManager不永久busy。
7. 测试结束只停止本轮monitor/rosbag；只有操作者明确要求时才关闭导航和传感器栈。

必须录制：

```text
/imu
/vehicle_status
/rslidar_Pointcloud2
/odom_incremental
/odom
/lio_sam/mapping/odometry
/lio_sam/mapping/odometry_incremental
/lio_sam/mapping/correction_event
/lio_sam/correction_event
/lio_sam/epoch_odometry
/lio_sam/map_odom_correction
/lio_sam/imu_preintegration_readiness
/localization_session
/localization_result
/tf
/tf_static
/laser/scan_front
/laser/scan_back
/laser/scan_merge_laser
/laser/scan_merge_laser_filtered
/global_costmap/costmap_raw
/local_costmap/costmap_raw
/plan
/cmd_vel
/cmd_vel_nav
/process_manager/fault
/navigate_to_pose/_action/status
/navigate_to_pose/_action/feedback
/navigate_through_poses/_action/status
/navigate_through_poses/_action/feedback
/rosout
```

## 8. 本次录包与证据位置

主测试目录：

```text
/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-20_10-25-54_manual_publish_monitor
```

- segment 2：覆盖11:09累计滞后误报，已在11:31:16正常停止。
- segment 3：11:55:41.850～12:03:35.094，473.244秒，155,136条消息，5.7 GiB，16个SQLite分片，metadata完整；16个分片的只读 `PRAGMA quick_check` 均为 `ok`。该段开始于11:37失败之后，主要用于后续基线和下一次任务准备；不能伪称完整覆盖11:37故障。
- 11:37故障本身由 monitor JSON、ROS节点日志和现场IMU采样诊断；上一段bag已停止，这是证据限制。
- observation-only监控：`monitor.raw.jsonl`，已于12:04:48停止。

## 9. 工作区和构建状态

- 代码未提交，工作区还有大量与导航、传感器、充电、交通灯等不同任务有关的历史改动。
- 本交接只声明上文列出的导航测试相关改动；其他dirty文件不能据此归因给本聊天。
- 当前正式 install 中，`lio_sam` 二进制和参数时间为2026-11-20 11:50～11:51，包含零速度更新。
- 11:09修复后的 `multi_map_navigation` 已构建并由随后实车管理栈加载。
- 核心文件哈希见 `relevant_files.sha256`；已跟踪文件差异见 `relevant_tracked_changes.patch`；完整工作区状态见 `git_status_short.txt` 和 `git_diff_stat.txt`。
- segment 3 的 bag 摘要和逐分片检查分别见 `segment_3_bag_info.txt`、`segment_3_sqlite_quick_check.txt`。

### 9.1 Python/NumPy 环境风险补充

2026-11-19 18:16 为另一个工程 `my-driving` 执行了带 `--user` 的依赖安装，曾导致普通用户 Python 优先加载 `~/.local` 中的 NumPy 2.2.6 和 OpenCV 5.1.0.95。该环境变化与10:59～11:01两次 `nav2_init_pose` 启动即退出高度相关。

2026-11-20 已完成环境隔离和恢复：`my-driving/.venv` 独立使用 NumPy 2.2.6/OpenCV 5.1.0.95，`start.sh` 固定调用该虚拟环境；公共用户目录中的两个冲突包已卸载，普通 ROS Python 恢复系统 NumPy 1.21.5/OpenCV 4.5.4。`paho-mqtt 1.6.1` 继续保留在用户目录，没有设置全局 `PYTHONNOUSERSITE=1`。SciPy、Matplotlib、`tf_transformations`、`transforms3d`、`cv_bridge`、OpenCV 和 MQTT 均已验证导入成功，两套环境的 `pip check` 均通过。

完整证据、当日故障归因和近几日修改影响见：

- `navigation_monitoring/diagnostics/2026-11-20_numpy2_environment_change_impact_analysis.md`
- `navigation_monitoring/diagnostics/2026-11-20_python_env_migration/README.md`

## 10. 新聊天的第一条回复

新聊天读完后应回复：

```text
已读取2026-11-20交接。当前导航、传感器、录包和监控均已关闭。
我会遵守：任务由你手动发布；你说“开始测试”时我只启动监控和录包；你说“结束测试”时我停止并检查落盘；不发布驻车、刹车、零速、fault或其他车辆控制命令。
下一轮首先验证静止零速度更新，不能把离线回放当成实车验收。
```

