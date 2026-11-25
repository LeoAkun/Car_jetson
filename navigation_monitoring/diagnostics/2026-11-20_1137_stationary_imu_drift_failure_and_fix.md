# 2026-11-20 11:37 静止预积分漂移故障分析与修复

## 结论

任务 `task_1795145873`（map2，航点 13 到 17）在车辆静止、Nav2 尚未下发目标时失败。直接故障为：

```text
LIO_SCAN_TO_MAP_UNRELIABLE: consecutive_rejects=3; consecutive_degenerate=0
```

本次不是上一轮 `map->odom` 累计滞后误报。新版本已经生效，已接受的中等修正正常进入限速跟随。实际问题是连续里程计的静止预测快速漂移，激光匹配连续尝试把预测位置拉回，修正量超过 15 cm 硬门限后被拒绝。

## 时间线

| 时间 | 事件 |
|---|---|
| 11:37:53.920 | 收到任务，车辆速度 0 km/h |
| 11:38:12.607 | 重定位成功并通过 TF 一致性检查，fitness `0.011950` |
| 11:38:17.987 | LIO 扫描匹配稳定检查达到 10/10 |
| 11:38:18.193 | 开始启动 Nav2 |
| 11:38:24.829 | epoch 33，修正 `0.180865 m`，拒绝 |
| 11:38:24.930 | epoch 34，修正 `0.169393 m`，拒绝 |
| 11:38:25.129 | epoch 35，修正 `0.215357 m`，第三次拒绝并触发故障 |
| 11:38:27.830 | epoch 36，修正增至 `0.458659 m` |
| 11:38:34.329 | epoch 41，修正增至 `1.825428 m` |
| 11:38:36.736 | Nav2 健康检查完成，但任务已锁存故障并开始清理 |

故障前 `/vehicle_status.cur_speed` 始终为 0，`/cmd_vel` 为 0，且动作目标状态为空，因此漂移发生在车辆尚未开始导航时。

## 根因证据

- scan-to-map 特征数量充分，epoch 33～35 均有约 870～902 个角点、7564～7625 个面点、4233～4331 个有效优化点，没有退化。
- 静止 IMU 现场采样 297 帧：频率约 100 Hz；姿态补偿后的平均水平加速度残差约 `0.0151 m/s²`。
- 该残差持续积分 10 秒会产生约 `0.75 m` 的位移量级，与本轮后期 0.67 m 修正及最大 1.83 m 修正一致。
- 原预积分图的初速度先验标准差为 `1e4 m/s`，没有利用底盘明确报告的静止状态；Nav2 启动带来的处理延迟扩大了两次激光校正之间的预测误差。
- 15 cm/2° 硬门限按设计拒绝了这些大修正。直接放宽门限会允许真正的位置跳变，因此门限保持不变。

## 已修改

在 `IMUPreintegration` 中增加基于 `/vehicle_status` 的零速更新：

1. 只读取底盘状态，不发送任何驻车、刹车或控制命令。
2. 只有速度反馈新鲜、连续至少 5 帧且 `|cur_speed| <= 0.10 km/h` 时，才在速度节点加入零速度因子。
3. 零速因子的标准差为 `0.05 m/s`。
4. 速度超过门限后立即清除静止计数；反馈超过 0.25 秒未更新时不使用零速约束。
5. 保留 15 cm/2° 单帧硬拒绝、连续三次拒绝故障、里程计输出连续性和静态足迹保护。

相关参数位于 `params_rslidar.yaml`：

```yaml
useVehicleZeroVelocityUpdate: true
vehicleStationarySpeedThresholdKmh: 0.10
vehicleStatusTimeoutSec: 0.25
vehicleStationaryMinSamples: 5
zeroVelocitySigma: 0.05
```

## 验证

- `lio_sam` 已成功编译并安装到工作区正式 `install`。
- 独立 ROS 域回放上午实车数据时，零速更新持续生效。
- 回放日志中约束前速度预测为 `0.009～0.061 m/s`，约束后为 `0.004～0.040 m/s`，单个 0.1 秒周期对应毫米级位移，显著低于 15 cm 门限。
- 当前管理栈没有重启；本轮 LIO 已在故障清理中退出。下一任务启动 LIO 时会直接使用新二进制和新参数。

## 录制状态

本次失败发生时上一段 rosbag 已停止，因此本轮使用监控 JSON 和 ROS 节点日志完成诊断。11:55 左右已启动 segment 3 录包，并增加以下诊断话题：

- `/lio_sam/mapping/correction_event`
- `/lio_sam/correction_event`
- `/lio_sam/epoch_odometry`
- `/lio_sam/map_odom_correction`
- `/localization_session`
- `/localization_result`

目录：

```text
/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-20_10-25-54_manual_publish_monitor/navigation_core_bag_segment_3
```
