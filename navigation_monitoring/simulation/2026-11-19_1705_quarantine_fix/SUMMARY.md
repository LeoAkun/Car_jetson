# 2026-11-19 17:05 导航失败修复验证

## 原始故障

- LIO-SAM 启动后的第 7 个扫描修正为 `0.066498 m`。旧逻辑在首个超过
  `0.05 m` 的修正上立即发布 `LIO_CORRECTION_REQUIRES_RELOCALIZATION`，导航任务随即清理。
- 原始录包中第 7 帧之后存在 10 个连续小修正，说明首个 6.65 cm 修正不足以单独证明定位失效。
- 更晚的第 28、33、38 帧在 5 秒内连续超过 5 cm；再往后出现连续超过
  15 cm 的扫描匹配拒绝。后半段属于持续异常，应保留故障保护。

## 修改后的策略

- 单次 5～15 cm 修正进入 `STATE_MEDIUM_QUARANTINED`，冻结 `map->odom`，不立即终止任务。
- 5 秒内累计 3 次中等修正才发布定位故障。
- 隔离后必须连续出现 10 个小修正才解除隔离，随后继续限速更新 `map->odom`。
- LIO-SAM 启动时也必须出现 10 个连续、接受且非退化的小修正，ProcessManager 才判定其健康并允许 Nav2 启动。
- 保留超过 15 cm 的扫描匹配拒绝、连续 3 次拒绝故障和 `/odom` 单步 5 cm/1° 硬保护。

## 验证

- `multi_map_navigation_msgs`、`lio_sam`、`multi_map_navigation` 正式工作区编译成功。
- 相关单元测试：74 passed。
- 隔离回放观察到 `LIO_CORRECTION_QUARANTINED`，证明中等修正进入观察状态；连续扫描匹配拒绝仍发布 `LIO_SCAN_TO_MAP_UNRELIABLE`。
- 回放出现录包开头 0.21 秒 IMU 间隔、静态 TF 不完整及调度导致的 `/odom` 5 cm 硬门触发，因此回放用于验证保护分支，不作为定位精度验收。
- 全测试目录默认收集会遇到系统 Matplotlib 与用户级 NumPy 2.2.6 的 ABI 冲突；切换到系统 NumPy 后可继续执行，但仓库级 `test_flake8.py` 会扫描整个工作区并报告 23,939 个既有格式问题，旧 NavigationManager 集成夹具还会在 ROS client/context 清理时挂起。与本次修改相关的 74 项测试已独立通过。

## 实车关注项

- 启动日志应先出现 `LIO-SAM扫描匹配稳定门槛已满足: 10/10`，随后才启动 Nav2。
- 单次中等修正应记录 `LIO_CORRECTION_QUARANTINED`，不应立即清理任务。
- `LIO_CORRECTION_REQUIRES_RELOCALIZATION` 应包含 `medium_count=3; window_sec=5.0`。
- 若出现 `LIO_SCAN_TO_MAP_UNRELIABLE` 或 `LIO_ODOM_CONTINUITY_FAULT`，仍应按真实定位故障清理导航栈。
