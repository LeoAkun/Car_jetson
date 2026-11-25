# 车辆导航实测监控与录包交接（2026-11-16）

> 将本文完整交给新的 Codex 聊天。新聊天先阅读和完成离线准备，然后等待操作者明确说“开始测试”。阅读本文不代表允许启动导航、监控、录包或发布任何 ROS 控制消息。

## 1. 本聊天的唯一任务

这是现场车辆导航测试的专用监控聊天。工作目录固定为：

```text
/home/akun/workspace/Car_jetson
```

操作者负责：

- 现场人工安全保护；
- 手动启动导航；
- 手动发布任务点；
- 车辆运动控制和必要时人工接管。

新聊天负责：

- 在收到“开始测试”后立即启动 observation-only 监控；
- 启动覆盖完整诊断链的 ROS 2 录包；
- 在测试期间观察传感器、TF、融合里程计、Nav2、代价地图和任务状态；
- 在收到“结束测试”或明确的人工接管通知后停止监控和录包；
- 确认 rosbag 正常落盘并形成测试记录；
- 只读分析失败原因。

## 2. 不可违反的控制边界

1. **禁止发布驻车、制动、零速度、`/cmd_vel`、fault 或任何车辆控制命令。**现场已有人工保护。
2. 不使用 `nav_task_monitor.py --enable-protection`。
3. 手动测试不使用 `nav_task_monitor.py --stop-on-terminal`。
4. 导航出现 `fault`、`idle`、`ABORTED`、`Starting point in lethal space` 或进程退出后，继续监控和录包，直到操作者明确说“结束测试”。
5. 不启动导航，不发布任务点，不调用派单器。
6. **不要运行 `navigation_tools/start_navigation.sh`**：它是自动派单入口，与本轮“操作者手动启动导航、手动发布任务”的职责约定不符。
7. “结束测试”默认只结束本轮监控和录包，不关闭整车传感器驱动，也不关闭导航栈；只有操作者明确要求时才处理导航栈。
8. 不修改地图、footprint、LIO-SAM、Nav2、雷达过滤或车辆参数。
9. 不例行清理 FastDDS SHM、重启 ROS daemon 或重启传感器。只有发现明确的 discovery/锁异常并取得证据后再处理。
10. 不使用宽泛的 `pkill ros2`、`pkill python3`。停止时只向本轮保存的 PID/PGID 发送信号。

## 3. 操作口令

### 操作者说“开始测试”

立即执行以下动作，不做无关的额外检查：

1. 建立独立测试目录；
2. 保存环境和进程基线；
3. 启动 observation-only `nav_task_monitor.py`；
4. 启动 rosbag；
5. 确认两个进程存活后立即回复“监控和录包已启动，可以手动启动导航/发布任务”；
6. 随后进行只读订阅和频率验证，不因尚未启动的 LIO/Nav2 话题不存在而阻塞操作者。

### 操作者说“结束测试”或“已经人工接管”

1. 记录操作者报告的时间、位置、原因和测试结果；
2. 对本轮 rosbag 和监控进程发送 `SIGINT`；
3. 等待 rosbag 正常退出并生成 `metadata.yaml`；
4. 检查关键话题计数、分片、SQLite 完整性和是否还有本轮进程残留；
5. 写入本轮 `run_state.json` 和 `summary.md`；
6. 回复落盘目录、覆盖时间、关键消息计数和初步结论。

不要因“人工接管”自行向车辆发送零速度或驻车命令。

## 4. 当前需要验证的故障假设

2026-11-14 11:13 的 110→114 失败已经确认以下事实：

- LIO mapping odometry 出现约 `1.508 m` 的大 correction；
- correction 到达约 `4 ms` 后，融合 `/odom` 和 `odom→base_link` 在 `10 ms` 内跳变约 `1.179 m`；
- `map→odom` 全程保持不变，因此不是启动重定位失败；
- 跳变把估计足迹送入预建静态地图障碍，随后持续出现 `Starting point in lethal space`；
- 同时刻 local costmap 车体附近为空，global costmap 致命格对应静态 PGM occupied；
- 已知前雷达近车体回波没有命中本次失败窗口。

当前最高概率机制是：异常大的 mapping correction 到达后，TransformFusion 切换新 anchor，但增量队列跨 correction epoch 组合，导致 `/odom`/TF 不连续。上一包漏录了 TransformFusion 的高频直接输入 `/odom_incremental`，因此还不能逐帧证明队列边界。

必须牢记：

```text
/odom_incremental
```

与：

```text
/lio_sam/mapping/odometry_incremental
```

是两个不同话题，二者都必须录制。

完整旧证据见：

- [三次失败复盘](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-14_costmap_failures/REPORT.md)
- [11:13 深度审计](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-14_costmap_failures/task_1794626035_deep_audit/README.md)
- [位姿跳入静态障碍图](/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-14_costmap_failures/task_1794626035_deep_audit/jump_into_static_obstacle.png)

## 5. 录包主题

### 5.1 融合、原始传感器和基础诊断：必须全程录制

```text
/imu
/rslidar_Pointcloud2
/odom_incremental
/odom
/lio_sam/mapping/odometry
/lio_sam/mapping/odometry_incremental
/lio_sam/imu_preintegration_ready
/lio_sam/feature/cloud_info
/tf
/tf_static
/rosout
/parameter_events
```

关键关系：

```text
IMUPreintegration:
  输入  /imu + /lio_sam/mapping/odometry_incremental
  输出  /odom_incremental

TransformFusion:
  输入  /lio_sam/mapping/odometry + /odom_incremental
  输出  /odom + odom→base_link TF
```

`/lio_sam/feature/cloud_info` 已包含去畸变点云、角点、面点、IMU/odom 可用标志和初始位姿猜测。不要再同时录制下面这些重复大数据话题：

```text
/lio_sam/deskew/cloud_deskewed
/lio_sam/deskew/cloud_info
/lio_sam/feature/cloud_corner
/lio_sam/feature/cloud_surface
```

### 5.2 GNSS 链：必须全程录制

```text
/sensing/gnss/pose_with_covariance
/odometry/filtered_local
/odometry/gpsz
/gps/filtered
/odometry/navsat
```

LIO-SAM 真正使用的 GPS factor 输入是 `/odometry/gpsz`，原始 GNSS 不能替代它。若派生 GNSS 话题没有 publisher，要记录“系统未发布”，不要误判为录包器故障。

### 5.3 导航后果链：必须全程录制

```text
/laser/scan_front
/laser/scan_back
/laser/scan_merge_laser
/laser/scan_merge_laser_filtered
/global_costmap/costmap_raw
/local_costmap/costmap_raw
/plan
/cmd_vel
/cmd_vel_nav
/vehicle_status
/robot_status
/robot_state
/waypoint_list
/start_end_graph
/process_manager/fault
/initialpose
/navigate_to_pose/_action/status
/navigate_to_pose/_action/feedback
/navigate_through_poses/_action/status
/navigate_through_poses/_action/feedback
```

不要同时录 `/global_costmap/costmap` 和 `/global_costmap/costmap_raw`，二者历史数据基本重复，每帧约 25 MB。local costmap 同样只保留 raw。

### 5.4 体积允许时的 LIO 深度诊断

```text
/lio_sam/mapping/cloud_registered
/lio_sam/mapping/map_local
/lio_sam/mapping/trajectory
/lio_sam/mapping/path
/lio_sam/mapping/loop_closure_constraints
/lio_loop/loop_closure_detection
```

优先保留原始 `/rslidar_Pointcloud2` 和 `/lio_sam/feature/cloud_info`。`cloud_registered_raw` 体积很大且可通过原始云和位姿离线重建，默认不录。

## 6. 推荐 rosbag 启动模板

阅读本文时不要执行。只有收到“开始测试”后才使用。

```bash
cd /home/akun/workspace/Car_jetson

set +u
source /opt/ros/humble/setup.bash
for setup_file in \
  real_ws/install/setup.bash \
  utils/install/setup.bash \
  slam/install/setup.bash \
  nav2/install/setup.bash \
  multi_map_navigation/install/setup.bash; do
  if [[ -f "$setup_file" ]]; then
    source "$setup_file"
  fi
done
set -u

RUN_ID="$(date '+%Y-%m-%d_%H-%M-%S')_manual_publish_monitor"
RUN_DIR="/home/akun/workspace/Car_jetson/navigation_monitoring/runs/${RUN_ID}"
BAG_DIR="${RUN_DIR}/navigation_core_bag"
REC_NODE="field_test_recorder_$(date '+%Y_%m_%d_%H_%M_%S')"
mkdir -p "$RUN_DIR"

TOPICS=(
  /imu
  /rslidar_Pointcloud2
  /odom_incremental
  /odom
  /lio_sam/mapping/odometry
  /lio_sam/mapping/odometry_incremental
  /lio_sam/correction_event
  /lio_sam/epoch_odometry
  /lio_sam/map_odom_correction
  /lio_sam/localization_fault
  /lio_sam/imu_preintegration_ready
  /lio_sam/feature/cloud_info
  /tf
  /tf_static
  /rosout
  /parameter_events
  /sensing/gnss/pose_with_covariance
  /odometry/filtered_local
  /odometry/gpsz
  /gps/filtered
  /odometry/navsat
  /laser/scan_front
  /laser/scan_back
  /laser/scan_merge_laser
  /laser/scan_merge_laser_filtered
  /global_costmap/costmap_raw
  /local_costmap/costmap_raw
  /plan
  /cmd_vel
  /cmd_vel_nav
  /vehicle_status
  /robot_status
  /robot_state
  /waypoint_list
  /start_end_graph
  /process_manager/fault
  /initialpose
  /navigate_to_pose/_action/status
  /navigate_to_pose/_action/feedback
  /navigate_through_poses/_action/status
  /navigate_through_poses/_action/feedback
  /lio_sam/mapping/cloud_registered
  /lio_sam/mapping/map_local
  /lio_sam/mapping/trajectory
  /lio_sam/mapping/path
  /lio_sam/mapping/loop_closure_constraints
  /lio_loop/loop_closure_detection
)

setsid ros2 bag record \
  --storage sqlite3 \
  --compression-mode none \
  --include-hidden-topics \
  --polling-interval 100 \
  --max-bag-size 1073741824 \
  --max-bag-duration 30 \
  --max-cache-size 268435456 \
  --node-name "$REC_NODE" \
  --output "$BAG_DIR" \
  "${TOPICS[@]}" \
  >"${RUN_DIR}/bag_record.log" 2>&1 < /dev/null &

BAG_PID=$!
BAG_PGID="$(ps -o pgid= -p "$BAG_PID" | tr -d ' ')"
printf '%s\n' "$BAG_PID" >"${RUN_DIR}/bag_record.pid"
printf '%s\n' "$BAG_PGID" >"${RUN_DIR}/bag_record.pgid"
```

`--max-bag-duration 30` 和 `--max-bag-size 1073741824` 都只是分片条件，不会在 30 秒或 1 GiB 后停止整轮录制。不要使用 `--no-discovery`，因为 LIO/Nav2 话题可能在录包启动后才出现。

历史数据量约 25 MB/s；原始点云约 640 MiB/min，global costmap raw 历史约 660 MiB/min。开始前只需确认磁盘空间明显足够，不要为节省空间删除历史证据。

## 7. 监控启动模板

与 rosbag 同时启动，但不带任何保护或自动终止参数：

```bash
setsid python3 \
  /home/akun/workspace/Car_jetson/navigation_monitoring/nav_task_monitor.py \
  --output "${RUN_DIR}/monitor.raw.jsonl" \
  >"${RUN_DIR}/monitor.log" 2>&1 < /dev/null &

MONITOR_PID=$!
MONITOR_PGID="$(ps -o pgid= -p "$MONITOR_PID" | tr -d ' ')"
printf '%s\n' "$MONITOR_PID" >"${RUN_DIR}/monitor.pid"
printf '%s\n' "$MONITOR_PGID" >"${RUN_DIR}/monitor.pgid"
```

正确命令中不得出现：

```text
--enable-protection
--stop-on-terminal
```

在 `run_state.json` 中至少保存：

- `started_at`；
- `run_dir`；
- rosbag 和 monitor 的 PID、PGID、`/proc/<pid>/stat` start ticks；
- 完整 topics；
- `observation_only: true`；
- `vehicle_control_commands_sent: false`；
- 当前 `boot_id`；
- `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、FastDDS 相关环境；
- 操作者负责发布任务的说明。

## 8. 启动后的验证

先确认监控和 recorder 进程存活，然后立即回复操作者。后续验证不能阻塞其手动启动导航。

### 8.1 recorder 订阅日志

LIO 启动后，`bag_record.log` 必须出现：

```text
Subscribed to topic '/odom_incremental'
Subscribed to topic '/rslidar_Pointcloud2'
Subscribed to topic '/lio_sam/mapping/odometry'
Subscribed to topic '/lio_sam/mapping/odometry_incremental'
```

其中 `/odom_incremental` 是本轮最关键检查项。

### 8.2 话题频率与 publisher

分批、短时检查，避免同时运行大量 `ros2 topic hz`：

```bash
timeout 12 ros2 topic hz /imu
timeout 12 ros2 topic hz /rslidar_Pointcloud2
timeout 12 ros2 topic hz /odom_incremental
timeout 12 ros2 topic hz /odom
timeout 12 ros2 topic hz /lio_sam/mapping/odometry
ros2 topic info /odom_incremental -v
```

大致预期：

| 话题 | 预期频率 |
|---|---:|
| `/imu` | 约 100 Hz |
| `/rslidar_Pointcloud2` | 约 10 Hz |
| `/odom_incremental` | 约 100 Hz |
| `/odom` | 约 100 Hz |
| `/lio_sam/mapping/odometry` | 约 2–10 Hz |

录包活动期间 `metadata.yaml` 尚未最终生成，SQLite 也可能有缓存未提交。此时以 recorder 订阅日志、topic publisher 和短时频率为准，不要因为读活动数据库而造成锁竞争。

## 9. 现场重点观察

监控只报警和记录，不控制车辆。

重点关联以下事件：

1. `/lio_sam/mapping/odometry` 单帧平移 correction，特别是 `>0.5 m`；
2. `/odom_incremental` 在几十毫秒内的异常步长；
3. `/odom` 和 `odom→base_link` 是否紧随 mapping correction 跳变；
4. `map→odom` 是否变化；
5. IMU header 与接收时间、丢帧、非单调或数值尖峰；
6. `/odometry/gpsz` 是否在大 correction 同期到达；
7. loop constraint 是否与 correction 同期；
8. `Starting point in lethal space`、`exceeded maximum iterations`、costmap clear timeout；
9. global costmap 中心/footprint 是否变为 254，同时 local costmap 是否仍为空；
10. raw/filtered scan 是否存在近车体回波；
11. 实际车速、`cmd_vel_nav`、人工接管时间和原因。

当前 obstacle layer 中“缓冲区内有效点数为0/雷达丢失”的中文告警存在统计 bug，不能单独作为雷达掉线证据。应以 scan 话题频率、消息计数和实际 ranges 为准。

## 10. 结束与落盘

只使用本轮保存的 PID/PGID。推荐先对进程组发送 `SIGINT`：

```bash
kill -INT -- "-${BAG_PGID}"
kill -INT -- "-${MONITOR_PGID}"
```

等待退出，不要立刻 `SIGKILL`。确认：

```bash
kill -0 "$BAG_PID" 2>/dev/null
kill -0 "$MONITOR_PID" 2>/dev/null
```

正常情况下两者都应返回不存在。随后：

1. 确认 `${BAG_DIR}/metadata.yaml` 存在；
2. 运行 `ros2 bag info "$BAG_DIR"`；
3. 核对 `/odom_incremental`、`/odom`、两路 mapping odometry、IMU、点云、TF 和 `/rosout` 的计数大于 0；
4. 对每个 `.db3` 只读执行 `PRAGMA quick_check`；
5. 检查 metadata 分片数、总消息数和文件列表一致；
6. 检查没有本轮 recorder/monitor PID 残留；
7. 写 `summary.md`，记录成功/失败、航点、异常、接管原因和证据限制。

不得因为本轮结束而停止整车传感器驱动。

## 11. 录包后的判定矩阵

| 观察结果 | 优先判断 |
|---|---|
| `/odom_incremental` 平滑，mapping correction 到达后 `/odom` 突跳 | TransformFusion 重锚或增量队列跨 correction epoch 的概率最高 |
| `/odom_incremental` 自己先跳，再带动 `/odom` | IMUPreintegration 重置/重传播或其输入首先异常 |
| mapping 大跳同期 `feature/cloud_info` 特征骤减、初始猜测异常或退化标志置位 | scan-to-map 匹配退化 |
| mapping 大跳与 `/odometry/gpsz` 同期 | GPS factor 需要重点审计 |
| mapping 大跳与 loop constraint 同期 | 回环因子需要重点审计 |
| `map→odom` 跳变 | 重定位/地图坐标变换问题；与 11:13 机制不同 |
| 位姿跳变后 global costmap 才变 lethal，local costmap 和 scan 正常 | 错误位姿进入静态地图障碍 |
| 位姿平滑，但 raw scan 近点进入 global/local obstacle layer | 雷达近回波或实际障碍问题 |

现有源码没有发布 scan-to-map 残差、LM 迭代结果、GPS/回环因子实际采用状态和 TransformFusion queue epoch。若要严格证明跨 epoch，还需后续增加只读诊断输出：correction 序号、header/arrival 时间、`lidarOdomTime`、队列首尾 header、队列长度、清理数量以及输入/输出位姿增量。未经离线验证，不要直接修改车上融合行为。

## 12. 新聊天读完后的回复格式

新聊天完成只读准备后，只回复类似：

```text
已阅读交接并准备好。
职责：只启动 observation-only 监控和 rosbag，不启动导航、不发布任务、不发布任何车辆控制命令。
关键补录项：/odom_incremental。
收到“开始测试”后立即启动；收到“结束测试”或人工接管通知后正常落盘并检查完整性。
```

然后等待操作者口令，不提前运行任何现场程序。
