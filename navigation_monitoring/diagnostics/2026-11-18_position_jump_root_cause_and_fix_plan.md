# 位置跳变复盘与修改方案（2026-11-18）

## 1. 结论

行驶中的主要位置跳变已经能够定位到 LIO-SAM 的融合链，而不是车辆真实运动，也不是本轮 `map -> odom` 重定位 TF 被重复发布：

1. `mapOptimization` 发布会被 scan-to-map、回环或 GPS 因子修正的全局 `/lio_sam/mapping/odometry`。
2. `TransformFusion` 收到新消息后，直接把 `lidarOdomAffine` 替换为新锚点。
3. 它随后用 `新锚点 × queue.front^-1 × queue.back` 生成 100 Hz `/odom`，没有修正 epoch、同 epoch 校验、时间插值、连续性约束或异常修正门禁。
4. IMU 预积分收到同一轮 mapping correction 后会重置并重新传播，队列里可能同时存在修正前和修正后的增量状态。新锚点和跨修正边界的增量组合后，后端修正会在一个 10 ms 周期内进入 `/odom`。
5. Nav2 根据跳变后的 `map -> base_link` 重新规划。车辆仍能贴着“最新路径”行驶，但最新路径相对最初路径已经迁移，现场表现为突然靠边、进入静态障碍区、`Starting point in lethal space` 和恢复倒车。

今天还发现第二个放大源：实际启动默认加载的是 `params_rslidar.yaml`，其中回环检测为 10 Hz、fitness 上限为 0.5；今天三次 LIO 会话分别累计接受了 99、73、1 条回环约束。前两轮两三分钟内持续增加回环约束，明显过于频繁。回环不是全部跳变的唯一来源，因为第二轮在 38～57 秒没有新增回环的区间仍有跳变；scan-to-map 修正和预积分重传播也必须一起治理。

第三轮属于另一条故障链：重定位约 322 秒后以 fitness `0.032711` 被接受，Nav2 启动时完整足迹已经压入静态占据格。本轮 `/odom` 没有超过 0.10 m 的跳变，不能把这个启动非法归因于行驶中 TransformFusion 跳变。

## 2. 历史证据对照

| 时间/样本 | 跳变证据 | 与后端修正的关系 | 判断 |
|---|---|---|---|
| 2026-11-10 10:37:57 | `/odom_incremental` 在 10 ms 跳 0.665 m，融合 `/odom` 跳 1.232 m | mapping correction 后约 8.7 ms 出现；按源码公式重放，选取的大跳残差约 `10^-14 m` | 直接证明当前融合公式能把 correction epoch 边界放大成 `/odom` 瞬移 |
| 2026-11-14 11:16:38 | `/odom` 在 10 ms 跳 1.179 m，表观速度 118.7 m/s | 1.508 m mapping anchor 修正到达约 4 ms 后发生；`map -> odom` 全程常量 | 已闭合的运行中位置跳变，不是重定位失败 |
| 2026-11-14 11:22 成功对照 | 最大仍有约 0.414 m/10 ms 小跳和瞬时 lethal | 跳变量没有把完整足迹持续推入静态障碍 | 同一缺陷一直存在，只是量级不足以终止任务 |
| 2026-11-18 第一轮 | 导航移动段 18 次 `/odom > 0.10 m/10 ms`，最大 0.190 m；若包含发目标前启动段为 21 次，最大 0.496 m | 导航移动段 `/odom_incremental` 同等级跳变仅 1 次；会话累计 99 条回环约束 | 输出跳变远多于高频输入，TransformFusion 重锚仍是直接传播点 |
| 2026-11-18 第二轮 | 33 次 `/odom > 0.10 m/10 ms`，最大 0.237 m | `/odom_incremental` 无同等级跳变；会话累计 73 条回环约束；无回环新增的长区间仍有跳变 | 回环是放大源之一，但不是唯一来源 |
| 2026-11-18 第三轮 | `/odom` 最大 0.079 m，零次超过 0.10 m | 只有 1 条回环；启动足迹已非法 | 主要是弱重定位结果缺少几何复核，与行驶中跳变分开处理 |

11 月 16 日倒车失败和 11 月 17 日部分 lethal-space 任务没有覆盖关键时段的完整 TF、两路增量里程计和 costmap，不能反向断言每一次都发生了位置跳变。已有证据只支持将位置跳变列为高概率来源之一。

## 3. 已排除或已降级的方向

- **真实车辆瞬移**：车辆速度约 0～1.9 km/h，不可能出现 0.2～1.2 m/10 ms 位移。
- **11 月 14 日的 `map -> odom` 变化**：1,795 个样本保持常量，跳的是 `odom -> base_link`。
- **今天的 GPS 因子**：`/odometry/gpsz` 为 0 条，今天的修正不是 GPS 注入；以后 GPS 恢复后仍要加 innovation gate。
- **固定方向转向零偏**：今天两轮转向反馈符号不一致，不支持一个固定转向偏置解释全部靠边。
- **旧导航栈或旧 TF 是今天的主因**：会话 UUID、进程 generation、清理屏障和唯一 `map -> odom` 发布者已经生效；今天的跳变仍发生在新会话内部。
- **11 月 11 日 IMU 修复已经覆盖此问题**：11 月 11 日修改解决时间戳间隙、回调阻塞、预积分重建和 readiness；没有修改 TransformFusion 的锚点替换或连续性。

## 4. 修改总原则

最终必须满足下面的坐标系职责：

```text
map --(唯一 authority，可受控修正)--> odom --(连续、不可瞬移)--> base_link
                         \
                          +--> LIO 后端优化结果只作为修正观测和诊断
```

- `odom -> base_link` 只表达连续局部运动，回环、GPS和全局优化不能让它瞬移。
- 全局修正形成 `map -> odom` 的目标值，由现有唯一 `map_odom_authority` 审核和限速应用。
- 大修正先结束当前 Nav2 action，再验证定位和静态足迹；不发布驻车或刹车命令。
- 每一个 correction、epoch、LIO generation 和定位 session 都必须可追踪，旧进程消息不能进入新会话。

## 5. 分阶段修改

### 阶段 A：先补齐可观测性，不改变车辆位姿

#### A1. 新增消息

在 `multi_map_navigation_msgs` 增加：

```text
LioCorrectionEvent.msg
  std_msgs/Header header
  string instance_id
  uint64 generation
  uint64 correction_epoch
  uint8 source                 # SCAN_TO_MAP / LOOP / GPS / RESET
  geometry_msgs/Pose predicted_pose_at_stamp
  geometry_msgs/Pose optimized_pose_at_stamp
  geometry_msgs/Pose incremental_pose_at_stamp
  float64 translation_residual
  float64 yaw_residual
  float64 source_score
  float64 arrival_delay_sec
  bool degenerate
  bool accepted
  string reason

EpochOdometry.msg
  std_msgs/Header header
  string instance_id
  uint64 generation
  uint64 correction_epoch
  nav_msgs/Odometry odometry
```

不要把 epoch 塞进 covariance、frame_id 或 child_frame_id；这些字段已有语义，后续很难可靠检查。

#### A2. `mapOptmization.cpp` 记录真实修正来源

修改位置：

- `scan2MapOptimization()`：记录 initial guess 到优化结果的平移/航向残差、特征数、退化标志和迭代数。
- `performLoopClosure()`：记录 key id、ICP fitness、重叠率、最终变换量和接受/拒绝原因。
- `addGPSFactor()`：记录 GPS innovation、协方差和接受/拒绝原因。
- `saveKeyFramesAndFactor()`：不要继续用一个 `aLoopIsClosed` 同时表示 GPS 和回环；拆成明确 source bitmask。

计算 correction 时不能比较相邻 mapping pose，因为其中包含车辆真实运动。应比较同一时间戳下的预测位姿和优化位姿：

```text
delta_correction = inverse(predicted_pose_at_stamp) * optimized_pose_at_stamp
```

#### A3. `imuPreintegration.cpp` 给 correction 建 epoch

- 只有一条 mapping incremental correction 成功完成优化和 repropagation 后，`correction_epoch` 才加 1。
- 每条高频增量里程计带上 `instance_id + generation + correction_epoch`。
- reset、IMU gap 恢复、LIO 重启都必须开启新 epoch；不得复用旧编号。
- TransformFusion 记录 anchor stamp、arrival time、队列首尾 stamp、队列中各 epoch 数量和最终输出步长。

#### A4. 阶段 A 实施记录（2026-11-18）

阶段 A 已完成，保持现有定位和 TF 输出算法不变：

- 新增 `LioCorrectionEvent` 和 `EpochOdometry` 正式消息；没有复用 covariance、frame id 或 child frame id 承载 epoch。
- `mapOptimization` 分别产生 `SCAN_TO_MAP`、`LOOP`、`GPS` 诊断。scan-to-map 记录 initial guess、最终位姿、平均绝对特征残差、角/面/入选特征数、迭代数和退化状态；回环记录 key id、ICP fitness、2 m overlap、点数和接受/拒绝原因；GPS 记录 XY innovation、协方差和接受/拒绝原因。
- GPS 与回环使用独立 source bitmask，已经移除原先用同一个 `aLoopIsClosed` 表示两种来源的歧义。
- IMU 预积分只在 mapping correction 完成优化和 realtime IMU repropagation 后增加 `correction_epoch`；初始化、reset 和 IMU gap 恢复均产生可追踪 epoch/reset event。
- `/lio_sam/epoch_odometry` 为每条高频增量里程计附带 `instance_id + generation + correction_epoch`；`/odom_incremental` 和 `/odom` 的原消息与计算保持不变。
- TransformFusion 增加 `TFUSION_ANCHOR`、`TFUSION_CORRECTION`、`TFUSION_OUTPUT` 结构化日志，记录 anchor 延迟、队列首尾时间、epoch 分布、source、残差和输出步长；本阶段只诊断跨 epoch，不拦截或改变输出。
- `nav_task_monitor.py` 已订阅正式 correction/epoch 话题；测试录包清单已加入 `/lio_sam/correction_event` 和 `/lio_sam/epoch_odometry`。

### 阶段 B：立即修复 TransformFusion 的跨 epoch 与瞬移

#### B1. 禁止跨 epoch 融合

TransformFusion 只允许以下三项完全一致后计算输出：

1. mapping anchor 的 `instance_id/generation` 与当前 LIO 实例一致；
2. anchor 的 `correction_epoch` 与增量样本一致；
3. 作为 base 和 current 的两条增量样本属于同一 epoch。

缺少匹配样本时保持最后一条合法输出并发布 `WAITING_FOR_EPOCH` 诊断，不能把旧 front 与新 back 相乘。

#### B2. 用明确的 epoch 基准替代 `queue.front`

每个 correction event 必须携带 correction stamp 上的 `corrected_incremental_pose`。新 epoch 当前时刻的原始结果为：

```text
raw_current = optimized_anchor
            * inverse(corrected_incremental_pose_at_anchor)
            * incremental_pose_current
```

这样不再依赖“删除 `stamp <= lidarOdomTime` 后剩下的第一帧”近似 correction 时刻，也不会受 0.4～0.7 秒 callback/arrival 延迟影响。

#### B3. 连续性偏置与修正状态机

新 epoch 第一次得到 `raw_current` 时计算：

```text
continuity_offset = last_published_pose * inverse(raw_current)
published_pose = continuity_offset * raw_current
```

第一帧因此与上一帧连续。后续如何释放 offset 由修正状态机决定，初始实车阈值如下：

| correction residual | 导航中处理 | 应用方式 |
|---|---|---|
| 平移 `<= 0.05 m` 且航向 `<= 1°` | 接受 | 至少 1～2 秒渐进释放，修正速度不超过 0.05 m/s、0.5°/s |
| 平移 `0.05～0.30 m` 或航向 `1～5°` | 判为可疑 | 取消当前 Nav2 action，冻结新目标；通过新点云和静态足迹复核后在 2～5 秒内应用 |
| 平移 `> 0.30 m` 或航向 `> 5°` | 拒绝 | 保持连续里程计，发布 localization fault，要求重新定位 |

阈值作用于“预测与优化的修正残差”，不能作用于相邻 pose 的绝对位移。车辆最高约 0.556 m/s，mapping 消息间隔 0.2 秒时正常物理位移本来就可能超过 0.10 m。

额外输出硬保护：100 Hz `/odom` 相邻约 10 ms 平移超过 0.05 m或航向超过 1°时，无条件发布连续性故障。这个门限是输出质量门限，不是 correction 接受门限。

#### B4. 中短期行为

在坐标系拆分完成前，小修正可以在 `/odom` 上限速释放；中、大修正期间必须取消 action 后处理。该补丁用于先消除瞬移，不应成为最终 frame 架构。

### 阶段 C：拆分连续 odom 与 LIO 全局优化坐标系

这是最终方案。

#### C1. Frame 结构

- `mapOptimization` 的优化结果改在内部 `lio_map` frame 表达，不再把受回环/GPS修正的结果标为 `odom`。
- `odom -> base_link` 只有一个发布者，输出经过连续化的局部 IMU/LiDAR运动。
- 删除或改名当前 `mapOptmization.cpp` 发布的 `odom -> lidar_link` 诊断 TF，避免把优化坐标混入 Nav2 TF 树。
- 同一时间戳计算：

```text
T_lio_map_odom_target = T_lio_map_base_optimized
                      * inverse(T_odom_base_continuous)

T_map_lio_map = T_map_odom_initial
              * inverse(T_lio_map_odom_at_initial)

T_map_odom_target = T_map_lio_map
                  * T_lio_map_odom_target
```

#### C2. 扩展 `map_odom_authority.py`

现有节点已经持有文件锁、定位 session 和唯一 `map -> odom` 广播权，应在这里增加动态 correction 状态机，不得再创建第二个 TF 发布者。

- 只接受当前 localization session、当前 LIO instance/generation 的 correction。
- 新 session 或地图切换时立即清除旧 correction offset、旧 epoch 和等待队列。
- 导航未开始时，稳定且通过足迹检查的结果可以一次设置。
- 导航中小修正限速应用；中修正先通知 NavigationManager 取消 action；大修正拒绝并要求重定位。
- 每次应用前后都检查完整矩形足迹；任何中间姿态触碰 static occupied/unknown，均不得自动恢复目标。

这一步完成后，TransformFusion 的 continuity offset 不再逐渐释放到 `/odom`，而是永久保持局部连续性；全局修正只通过唯一 `map -> odom` authority 进入 Nav2。

### 阶段 D：限制产生异常 correction 的上游

#### D1. 回环配置的第一轮降风险值

实际生效文件是 `slam/src/LIO-SAM/config/params_rslidar.yaml`，不是 `params_real.yaml`。第一轮建议：

```yaml
loopClosureFrequency: 1.0          # 10.0 -> 1.0
historyKeyframeFitnessScore: 0.30  # 0.50 -> 0.30
historyKeyframeSearchTimeDiff: 60.0
loopClosureMaxCorrespondenceDistance: 2.0
loopClosureMinOverlap: 0.55
```

前两项恢复到更保守的常见量级；后三项需要增加参数和实现。短路线 A/B 诊断可以临时关闭回环，用于确认频繁小跳是否显著下降，但不能把永久关闭回环当成最终修复。

当前距离检测把“时间差大于 30 秒、当前位置 15 m 内的旧关键帧”当作候选。在车辆低速经过长直路或相似走廊时，30 秒前的关键帧仍可能就在 15 m 内，但这不等于车辆真正回到了同一地点。随后 ICP 又允许 30 m 最大对应距离（`historyKeyframeSearchRadius * 2`）、fitness 0.5、零次 RANSAC；一旦接受，每个新 keyframe 还可以继续匹配另一个旧 keyframe。这条链能够解释为什么前两轮出现 99 和 73 条连续回环边。

#### D2. 回环代码门禁

一次 ICP fitness 小于阈值不足以加入因子。至少同时满足：

- source/target 有足够点数；
- 最大对应距离固定为 2 m 左右，不能继续用 `historyKeyframeSearchRadius * 2 = 30 m`；
- overlap/inlier ratio `>= 0.55`；
- 非退化，平移和航向修正有限；
- 候选需二次验证，两个独立点云结果相差不超过 0.20 m 和 2°；
- 同一走廊/同一历史 key 不得对连续每个新 keyframe反复加入等价约束；
- 因子使用 robust noise model，不能只把一个 fitness 值同时作为六维旋转和平移方差。

导航中若回环预计造成超过 0.30 m 或 5° 的当前位姿修正，因子先进入 pending，交由 correction 状态机处理，不能直接更新在线控制 TF。

#### D3. scan-to-map 门禁

第二轮在没有新增回环的长区间仍出现跳变，因此还要限制 scan-to-map：

- 记录 initial guess 到 scan-to-map 结果的总修正，不比较相邻绝对 pose；
- 记录特征数、残差、退化矩阵和每次 LM update；
- 单帧总修正初始上限设为 0.15 m / 2°；超过时保留预测位姿、标记该 scan rejected；
- 连续 3 帧 rejected 或连续 3 帧退化时发布 localization fault；
- 先用历史包统计正常分布后再固化最终阈值。

#### D4. GPS 门禁

今天 GPS odometry 为 0 条，所以 GPS 不是今天跳变来源。为以后恢复 GPS 做以下修改：

- 增加 `useGpsFactor` 参数，未完成验证前默认 false；
- 用 GPS 与同 timestamp 局部预测的 innovation 和协方差做 Mahalanobis gate；
- 至少 3 个连续新鲜样本一致后才能加入因子；
- 单次绝对偏差过大时只报错，不设置 `aLoopIsClosed` 或更新在线位姿；
- correction event 明确标记 source=GPS。

### 阶段 E：重定位和发目标门禁

用于处理今天第三轮以及地图切换后的全局位姿错误：

1. 采用分级确认，不能要求所有结果都重复两次：`fitness < 0.025` 且先验一致、静态足迹合法时可直接接受；`0.025～0.05` 才必须换一帧新点云做第二次局部确认。第二次不再执行完整 ±10 m PSO，而是以第一次结果为中心做小范围匹配；两次分数都应小于 0.05，位姿相差不超过 0.30 m 和 3°。
2. `map_odom_authority` 自身也校验 hard threshold `<= 0.05`，不能只信上游的 `success=true`。
3. 结果必须来自本 session 开始后的新点云，并限制结果点云年龄。
4. 组合最终 `map -> base_link` 后，用全局静态地图栅格化完整 footprint 加 0.05 m 启动余量；occupied 和 unknown 都判非法。
5. 连续 10 个样本、约 2 秒都合法后才允许发送首个目标。
6. 弱结果或足迹非法只允许用新点云重试 1～2 次，单轮总时间上限 120 秒；仍失败则结束本轮。
7. 地图切换优先使用已知 `switch_pose` 作为先验，并校验新定位结果与先验的偏差。

当前 `pcl_localization` 会在一个 session 内永久复用第一次缓存的点云。实施双确认前必须给 `ReLocalization` 增加 `observation_id`、`refresh_cloud` 和 `min_cloud_stamp`（或等价接口）：同一个 observation 内的所有候选复用同一帧以便公平比较；第二次确认必须使用更大的 observation id，并等待时间戳更新的新点云。否则所谓“两次确认”只是对同一数据重复计算，不提供独立证据。

12:12 的 fitness `0.032711` 会落入“双帧确认”区间，且最终会被静态足迹门禁拒绝，因此不会再进入无 plan 的倒车恢复。

## 6. 故障处置行为

检测到中、大 correction 或 `/odom` 连续性故障后：

1. NavigationManager 取消当前 Nav2 action并拒绝下一个目标；
2. 保存 correction event、TF、两路 mapping odometry、epoch odometry、静态足迹采样和最近 plan；
3. 小修正按状态机完成；中修正重新验证；大修正进入重新定位；
4. 只有定位、连续性和静态足迹重新通过后才允许新目标；
5. 不调用驻车服务，不发布驻车或刹车命令，现场人工保护规则保持不变。

## 7. 测试与验收

### 7.1 单元测试

- mapping anchor 先到、epoch odom 后到；不得跨 epoch 输出。
- epoch odom 先到、anchor 后到；等待配对，不得误用旧 anchor。
- correction arrival 延迟 0.7 秒、消息乱序、丢一帧、重复一帧；输出仍连续。
- generation/session 更新后，旧实例延迟消息全部拒绝。
- 小/中/大 correction 分别进入正确状态；取消 action 路径不发布驻车命令。
- SE(3) 插值使用平移插值和四元数 slerp，跨 `+-pi` 航向不产生反向大旋转。

### 7.2 离线回放

| 数据 | 必须通过的条件 |
|---|---|
| 11 月 10 日历史包 | 原 1.232 m/10 ms 跳变不再进入新 `/odom`；跨 epoch 事件被准确标记 |
| 11 月 14 日 11:13 包 | 1.508 m correction 被判为大修正并拒绝；在首个 lethal 前结束 action；连续 odom 不越入静态障碍 |
| 11 月 18 日第一、二轮 | 100 Hz `/odom` 无大于 0.05 m/10 ms 输出；每个 correction 都能追溯到 source/epoch |
| 11 月 18 日第三轮 | 弱重定位进入二次确认；非法足迹阻止发目标和 BackUp |

### 7.3 实车分段验证

1. 原地 10 分钟，只定位不发目标：无 correction storm，无 odom 连续性故障。
2. 空旷直线和缓弯：`/odom` 相邻 10 ms 最大平移 `<= 0.05 m`，99.9 分位建议 `<= 0.02 m`。
3. 短闭环路线重复 3 次：确认真实闭环能被接受，且不是每个关键帧都产生新回环边。
4. 复跑原故障路段：相对首条路径的迁移不超过 0.30 m；完整 footprint 不进入静态占据格。
5. 连续发布两次任务：新 session/generation 正确，旧 correction、旧 TF 和旧 action 都不被复用。

录包必须包含：

```text
/odom
/odom_incremental
/lio_sam/mapping/odometry
/lio_sam/mapping/odometry_incremental
/lio_sam/correction_event
/lio_sam/epoch_odometry
/lio_sam/map_odom_correction
/lio_sam/localization_fault
/lio_sam/mapping/loop_closure_constraints
/tf
/tf_static
/imu
/odometry/gpsz
/plan
/global_costmap/costmap_raw
/local_costmap/costmap_raw
/navigate_to_pose/_action/status
/navigate_to_pose/_action/feedback
/vehicle_status
/rosout
```

## 8. 实施顺序

1. 阶段 A：诊断、source、epoch 和 generation 全部可见。
2. 阶段 D1：先把实际生效的回环参数降到保守值，做短路线 A/B。
3. 阶段 B：完成 TransformFusion 同 epoch 计算和连续性状态机，用历史包回归。
4. 阶段 E：完成重定位双确认和发目标前静态足迹门禁。
5. 阶段 D2～D4：分别治理回环、scan-to-map、GPS 的异常 correction。
6. 阶段 C：把连续 odom 和优化 frame 完整拆分，由唯一 map authority 受控应用全局修正。
7. 通过离线回放后再按 7.3 分段实车，不在第一次实车同时放开所有变量。

## 9. 证据边界

- TransformFusion 把后端修正传播成不连续 `/odom` 已有源码、时间相关和公式重放证据，可信度高。
- 今天回环约束数量异常高已由 MarkerArray 累计边数确认；但旧 MarkerArray 没有 fitness、修正量和 source，不能逐条证明每个大跳由哪条回环边产生。
- 第二轮无新增回环期间仍有跳变，证明不能只调回环参数；scan-to-map 与预积分 correction epoch 必须同时修改。
- 11 月 16 日和部分 11 月 17 日任务缺少完整关键录包，不能把所有历史 lethal 或倒车都归为位置跳变。

## 10. 重定位精度与双确认风险

历史生产及真实传感器初始化日志已进一步汇总为 58 轮：51 轮被初始化器接受，7 轮明确失败。成功分数中位数为 `0.0007`，46/51 低于 `0.025`，5/51 位于 `0.025～0.05` 的建议二次确认区间。7 次失败中，5 次为生命周期/服务未就绪产生的 `inf`，只有 `0.3018` 和 `0.3823` 是完成点云计算后的有限分数失败，而且两次均受到旧服务实例和过期缓存点云污染。完整清单见 [历史重定位分数汇总](2026-11-18_historical_relocalization_scores.md)。

当前只能说“有些点云匹配得分很好”，不能据此说绝对定位精度已经稳定达到厘米级。`pcl_localization_component.cpp` 的分数定义是：

```text
inlier_ratio = 最近地图点距离 < 1.2 m 的点数 / 点云总点数
fitness      = 1 - inlier_ratio
```

所以 fitness `0.02` 表示约 98% 点落在 1.2 m 半径内，不表示 98% 点误差小于 2 cm。NDT 分辨率当前为 1.5 m、点云体素为 0.3 m；没有输出协方差、重复观测方差或绝对真值误差。当前结果应分为：

| 结果 | 能说明什么 | 不能说明什么 |
|---|---|---|
| `0.011995`、`0.019327` | 当前缓存点云与地图有很高的几何重叠 | 不能保证完整足迹安全，也不能证明姿态没有 0.1～0.3 m 偏差 |
| `0.032711` | 匹配达到现有 `<0.05` 门槛，但处于边界 | 本轮启动足迹已经非法，说明单次 score 不是安全和绝对精度门槛 |
| `0.3018`、`0.3823` | 明显没有得到可用匹配 | 失败主要还可能由旧服务实例使用过期缓存点云造成 |

双确认如果被实现成“完整 PSO 再跑一次、两次都必须 `<0.025`”，确实会增加失败和等待时间。原因有三点：

1. 全局 PSO 本身需要数千次服务调用，重复一次会把已经很长的等待再翻倍。
2. 不同点云会受到遮挡、人员、反射和扫描时序影响，第二次 fitness 可能略差，即使位姿仍可用。
3. 当前服务在一个 session 内缓存第一帧点云；若不刷新缓存，第二次实际上还是同一帧，既没有独立确认，也会掩盖缓存实例问题。

因此双确认应采用自适应规则，而不是所有任务无条件加一道失败门：

- fitness `<0.025`、与地图切换先验/GPS先验一致、完整静态足迹有余量：直接接受，不重新跑全局 PSO。
- fitness `0.025～0.05`：必须用新鲜点云做第二次局部确认；两次位姿初始要求差不超过 `0.30 m、3°`，足迹必须连续合法。收集至少10轮数据后，再根据实际重复方差收紧到 `0.15～0.20 m`。
- fitness `>0.05`或第二次不收敛：只允许换新点云重试1次；仍失败才报告重定位失败。
- 任意匹配分数通过但静态足迹非法，都报告“定位结果几何不安全”，不能统计成“匹配失败”，也不能发目标。

### 10.1 阶段 E 实施记录（2026-11-18）

本节方案已经落地：

- `ReLocalization` 已增加 `observation_id`、`refresh_cloud` 和 `min_cloud_stamp`；同一 observation 复用同一帧，后续 observation 必须使用时间戳严格更新的点云。
- 初始化器按 `<0.025` 强结果、`0.025～0.05` 弱结果和 `>0.05` 不合格结果分级。强结果满足先验和静态足迹门禁后直通；弱结果在第一帧位姿附近用新点云确认；不合格结果只换新点云重试一次。
- 二次确认要求两次位姿差 `<=0.30 m、<=3°`，不重复执行完整 `±10 m` PSO；单轮定位总时限为 120 秒。
- 地图切换使用 `switch_pose`，最终结果限制在先验 1.5 m、35° 内；GPS 初始搜索结果限制在 GPS 先验 15 m 内。
- 静态门禁使用完整 `0.86 m × 0.56 m` footprint，并叠加 Nav2 的 0.02 m padding 和 0.05 m 启动余量；occupied、unknown 和地图外均非法。
- `map_odom_authority` 独立校验 0.05 硬阈值、confirmation 元数据、两帧时间顺序和点云年龄；首目标发送前还要求 `map -> base_link` 连续 10 个不同时间戳样本的完整静态足迹合法。

阶段 E 只保护启动/地图切换时的 `map -> odom` 结果，不能替代前面阶段 B/C 对行驶中 TransformFusion 跳变的修复。

### 10.2 阶段 B～D 实施记录（2026-11-19）

阶段 B、C、D1～D4 已完成代码修改和离线编译，尚未进行实车放行：

- TransformFusion 已停止订阅不带 epoch 的 `/odom_incremental` 来参与融合。mapping odometry 必须先与相同时间戳的 `SCAN_TO_MAP` correction 配对，再与完全相同 `instance_id + generation + correction_epoch` 的 `EpochOdometry` 计算输出。
- correction event 中的 `incremental_pose_at_stamp` 现由 IMU 预积分在优化和重传播成功后写入，公式改为 `optimized_anchor * inverse(corrected_incremental_pose_at_anchor) * incremental_pose_current`，不再用 `queue.front()` 近似 correction 时刻。
- 每个新 epoch 首帧建立 continuity offset；中修正和大修正保持连续 odom、拒绝把跳变写入控制 TF，并通过 `/lio_sam/localization_fault` 取消当前导航。相邻输出超过 `0.05 m / 1°` 时直接拦截该输出。
- D1 生产配置已改为回环检查 `1 Hz`、fitness `0.30`、历史时间差 `60 s`、最大对应距离 `2 m`、最小 overlap `0.55`。
- D2 增加点数、overlap、有限刚体变换、独立相邻关键帧二次 ICP、重复历史约束抑制、`0.30 m / 5°` 硬拒绝和 Huber robust noise。超过硬阈值的回环当前直接拒绝，比“导航外等待 pending”更保守。
- D3 对预测到 scan-to-map 结果的单帧修正执行 `0.15 m / 2°` 门禁；越界时恢复预测位姿。连续 3 帧 rejected 或退化发布定位故障。
- D4 新增 `useGpsFactor`，生产默认 `false`。重新启用前仍需满足协方差、5 m 绝对 innovation、Mahalanobis `9.21` 和 3 个新鲜一致样本门禁。

### 10.3 阶段 C 实施记录（2026-11-19）

- 后端优化结果和 mapping 点云、轨迹统一标为 `lio_map`；原 `lio_map -> lidar_link` 诊断 TF 已删除。TF 树中的 `odom -> base_link` 只由连续化 TransformFusion 发布。
- 新增 `/lio_sam/map_odom_correction`，携带 LIO instance、generation、epoch、`lio_map -> odom` 目标和修正状态。
- `map_odom_authority` 仍是唯一 `map -> odom` 发布者。首个当前 LIO correction 只建立 `map -> lio_map` 基准，不改变重定位结果；后续小修正以最大 `0.05 m/s、0.5°/s` 应用，中修正要求重新定位，大修正拒绝。
- 每一个动态插值姿态都组合当前 `odom -> base_link`，使用完整矩形足迹检查当前静态地图；触碰 occupied、unknown、地图外时拒绝修正并取消导航。新定位 session 会清除旧 LIO identity、epoch、目标和插值状态。
- 测试录包和监控已加入 `/lio_sam/map_odom_correction` 与 `/lio_sam/localization_fault`。

当前验证边界：消息包和 LIO-SAM 均已编译；针对 `map_odom_authority`、地图切换和导航监控的 28 项单元测试通过。下一步必须依次完成 7.3 分段实车，不能因编译或离线回放通过直接视为实车验收完成。

### 10.4 阶段 B～D 历史传感器回放记录（2026-11-19）

使用 ROS_DOMAIN_ID=99 隔离运行新编译的四个 LIO-SAM 节点，只输入历史 `/imu`、`/rslidar_Pointcloud2` 和 `/tf_static`，没有启动导航管理栈、车辆驱动或任务发布。回放录下新 `/odom`、epoch、correction 和 fault 话题：

| 数据窗口 | 新链路结果 | 判定 |
|---|---|---|
| 11 月 18 日第一轮实际导航窗口（11:52:39 起，覆盖原始起步后 30～36 秒跳变区） | 10,277 个 `/odom` 相邻样本最大 `0.03996 m/10 ms`、99.9 分位 `0.03951 m/10 ms`，最大航向步长 `0.123°`，超过 `0.05 m/1°` 为 0 次；首次修改回放中暴露的 `23.95 m/10 ms` 假跳变在修复后未再出现。D3 连续拒绝和连续性 fault 均被记录。 | 通过硬门限，但未达到建议的 99.9 分位 `<=0.02 m`；此段必须由 fault 结束任务，不能继续导航 |
| 11 月 18 日第二轮窗口（11:35:33 起） | 9,785 个样本最大 `0.00304 m`、最大航向步长 `0.0049°`，超过硬门限为 0 次；记录 2 次中修正 fault。 | 通过连续性硬门限 |
| 11 月 18 日短段烟雾回放 | 4,090 个样本最大 `0.00165 m`，identity、epoch、source 均完整。 | 通过 |

第一轮回放还发现并修复了一条上游泄漏：scan-to-map 标记为 rejected 时，mapping odometry 仍可能携带尝试过的图优化位姿；TransformFusion 现在对 rejected event 使用同一时间戳的预测位姿作为 anchor，避免 rejected 结果绕过门禁重新进入 `/odom`。修复后才得到上述 3.996 cm 最大步长。

同一 LIO 进程发生 IMU 时间线 reset 时，TransformFusion 现保留最后合法 `/odom` 位姿、发布定位故障并等待新 epoch 连续重锚，不再清空连续性基准后直接接受恢复首帧。

11 月 14 日 11:13 录包只保存了 IMU、旧 mapping odometry 和 `/odom`，没有原始 `/rslidar_Pointcloud2`，因此不能用新 LIO-SAM 做等价传感器回放；该窗口仍保留原有离线证据，必须用包含原始点云的完整录包或实车分段验证。

