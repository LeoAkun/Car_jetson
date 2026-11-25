# TransformFusion 连续性补偿释放：第一阶段修复与离线回放

日期：2026-11-23

## 修复目标

此前 TransformFusion 在每次 scan-to-map 修正后用 `continuityOffset` 保持 `/odom`
连续，但释放函数没有被调用。结果是已经接受的小修正长期滞留在 `map->odom`，在
2026-11-23 录包中表现为 mapping odometry 与 incremental odometry 的 Z 基本稳定，
而最终 `/odom` 在 200 秒内下沉约 2.19 m。

本阶段只修复这个释放路径，不改车辆控制、导航目标或定位接受门限。

## 实现

- 对已接受的修正才释放；拒绝结果和等待 epoch 时不释放。
- 在最终输出位姿空间中释放，避免直接缩放左乘旋转时因地图坐标远离原点而产生
  大平移。
- XY、Z 和姿态分别限速：XY 0.05 m/s、Z 0.05 m/s、姿态 0.5 deg/s。
- 单次使用的 `dt` 上限为 0.1 s，数据间隙不会形成一次性大跳变。
- 每个新 epoch 仍以最后合法输出重锚，释放进度不会因重锚丢失。

涉及文件：

- `slam/src/LIO-SAM/include/lio_sam/continuity_offset_release.hpp`
- `slam/src/LIO-SAM/src/imuPreintegration.cpp`
- `slam/src/LIO-SAM/test/test_continuity_offset_release.cpp`
- `slam/src/LIO-SAM/CMakeLists.txt`

## 构建和单元测试

`lio_sam` 构建成功。测试汇总为 19 tests、0 errors、0 failures、0 skipped；其中新增
5 个测试覆盖分轴限速、远离地图原点、长时间间隔、跨 epoch 重锚收敛以及非法
时间间隔。

## 实包隔离回放

输入录包：

`navigation_monitoring/runs/2026-11-23_20-10-26_manual_publish_monitor/navigation_core_bag`

隔离回放只输入以下三条生产记录，不回放原 `/odom` 或 `/tf`：

- `/lio_sam/mapping/odometry`
- `/lio_sam/epoch_odometry`
- `/lio_sam/correction_event`

回放输出：

`navigation_monitoring/offline_runs/2026-11-23_continuity_release/fusion_replay_v1/patched_output`

结果：

| 指标 | 修复前同一类 200 s 窗口 | 修复后 200 s 窗口 |
|---|---:|---:|
| `/odom` Z 首尾变化 | -2.1851 m | -0.00331 m |
| `/odom` 最大相邻平移 | 0.000728 m | 0.02741 m |
| 相邻平移大于 5 cm | 0 | 0 |
| `/odom` 最大相邻 yaw | 0.00893 deg | 0.04181 deg |
| 相邻 yaw 大于 1 deg | 0 | 0 |
| localization/continuity fault | 0 | 0 |
| `map->odom target.z` 首尾累积 | +2.6262 m | 约 0 m |

修复后完整 265.86 秒输出中，`/odom` Z 首尾变化为 -0.02002 m；最后 60 秒为
-0.01679 m。200 秒窗口中 `map->odom target.z` 有短时释放过程，范围为
-0.58399～+0.06340 m，但窗口结束回到 0，没有继续累积为数米偏移。

## 结论与边界

第一阶段离线验收通过：原先由未释放连续性补偿造成的米级垂直漂移已消失，且没有
引入超过 5 cm / 1 deg 的相邻输出跳变。

这不是实车验收。上线前仍需完成生产链静止验证，并联合 `map_odom_authority` 检查
`map->base_link`、静态 footprint 和局部代价地图。另有三项状态机健壮性工作不属于
本阶段：重复/倒序时间戳不得推进释放；TF 查询失败时 `/odom` 发布状态与内部提交
状态需保持一致；accepted/skipped/rejected 的 correction 输出语义需增加契约测试。
