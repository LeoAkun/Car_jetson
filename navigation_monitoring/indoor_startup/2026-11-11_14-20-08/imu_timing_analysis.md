# 真实 IMU 连续性与时间分析：2026-11-11_14-20-08

本报告仅离线读取已结束运行的原始消息和日志。所有精确样本时间均为北京时间 UTC+8；`header − received` 为正表示消息抵达观察节点时 header 已指向未来。

共 22343 条 IMU，实际接收跨度 223.371 秒，100.022 Hz。
接收间隔中位 / P99 / 最大为 10.017 / 12.815 / 22.844 ms。
接收 >0.2 秒空洞 0；header >0.2 秒间隔 1；header 回退 / 重复 0 / 0。
header − received 中位 / P99 / 最大为 365.746 / 367.465 / 367.944 ms。

| 阶段 | IMU 条数 | 接收 Hz | 接收最大 ms | header 最大 ms | offset 中位 ms | offset 最大 ms | 异常四元数范数 |
|---|---:|---:|---:|---:|---:|---:|---:|
| sensor_baseline | 1933 | 100.240 | 22.844 | 330.000 | 364.505 | 365.645 | 2 |
| lio_start | 807 | 100.163 | 22.671 | 10.041 | 364.659 | 365.918 | 0 |
| nav2_start | 1603 | 100.033 | 14.805 | 10.041 | 365.738 | 366.050 | 0 |
| hold | 18000 | 100.001 | 17.060 | 10.045 | 365.909 | 367.944 | 0 |

第 2–7 秒 offset 中位 364.459 ms，最后 5 秒 366.775 ms，差 2.317 ms。时间变化不单独证明设备振荡器、host 时钟或消息延迟中哪个是原因。

角速度非有限 0；加速度非有限 0；四元数非有限 0、近零范数（<1e-6，非仅字面零）0、范数偏离 1 超过 0.1 共 2 条。完整轴向及范数分布见 JSON。

## 姿态、角速度与加速度

有效四元数 22341 条、排除 2 条；先归一化再计算姿态，yaw 连续展开，q 与 -q 视作同一旋转。

| 角度（度） | 首条有效值 | 末条有效值 | 最小 | 最大 | 首尾差 |
|---|---:|---:|---:|---:|---:|
| roll | 0.497575 | 0.496677 | 0.449859 | 0.539474 | -0.000899 |
| pitch | 0.073958 | 0.055375 | 0.031962 | 0.101685 | -0.018584 |
| yaw | 76.402129 | 76.365447 | 76.359701 | 76.413850 | -0.036682 |

首尾旋转夹角 0.041130°；相对首条最大旋转 0.063780°，发生在 `2026-11-11 14:23:47.388049503+08:00`。

| 阶段 | 有效姿态条数 | 首尾旋转度 | 相对阶段首条最大旋转度 |
|---|---:|---:|---:|
| sensor_baseline | 1931 | 0.022763 | 0.028353 |
| lio_start | 807 | 0.005328 | 0.034597 |
| nav2_start | 1603 | 0.021731 | 0.032856 |
| hold | 18000 | 0.032781 | 0.072143 |

角速度向量范数中位 / P99 / 最大：0.002156362 / 0.006203902 / 0.022802111 rad/s。
加速度向量范数最小 / 中位 / P99 / 最大：9.001041800 / 9.796366806 / 9.885309081 / 10.192716497 m/s²。
这些测量描述 IMU 自身记录的姿态和运动量，不能单独判断 mapping 位置变化来自人工移动还是定位误差；加速度包含重力，不能据此排除匀速平移。

## 时间戳和四元数异常样本

- header 异常第 6 行：received `2026-11-11 14:20:11.283596656+08:00`，header `2026-11-11 14:20:11.644754606+08:00`；接收间隔 1.334101 ms，header 间隔 330.000000 ms，offset 361.157950 ms，q 范数 `2.6161326552252416e+16`。
- quaternion 异常第 1 行：received `2026-11-11 14:20:11.276424147+08:00`，header `2026-11-11 14:20:11.274754606+08:00`；接收间隔 — ms，header 间隔 — ms，offset -1.669541 ms，q 范数 `6.673866032852771e+36`。
- quaternion 异常第 6 行：received `2026-11-11 14:20:11.283596656+08:00`，header `2026-11-11 14:20:11.644754606+08:00`；接收间隔 1.334101 ms，header 间隔 330.000000 ms，offset 361.157950 ms，q 范数 `2.6161326552252416e+16`。

每个异常前 5 条、后 3 条完整 q/gyro/accel 与整数纳秒时间戳已保存在 JSON；启动前 15 条也单独保留。

## CRC、设备时间与 LIO 日志

- `2026-11-11 14:20:11.278286235+08:00`：`[WARN] [1794378011.278286235] [ahrs_bringup]: check crc16 failed (ahrs), total_crc_errors=1`；最近 header 异常行 6 的接收时间比 CRC 日志晚 5.310421 ms。
CRC 与样本异常时间相近只能证明时序关联，不能仅凭 ROS 消息证明底层因果。
- 短读：`[WARN] [1794378243.575748741] [ahrs_bringup]: Short IMU serial read for frame type: received=0 expected=1 total_short_reads=1`；是否在关闭信号后：True。
- 设备时间 reset/re-anchor/non-monotonic 诊断：0 条。
- `lio_featureExtraction.log`：gap/reset/recovery 0 条；fatal/优化异常 0 条；WARN/ERROR/FATAL 0 条。
- `lio_imageProjection.log`：gap/reset/recovery 0 条；fatal/优化异常 0 条；WARN/ERROR/FATAL 0 条。
- `lio_imuPreintegration.log`：gap/reset/recovery 0 条；fatal/优化异常 0 条；WARN/ERROR/FATAL 0 条。
- `lio_mapOptimization.log`：gap/reset/recovery 0 条；fatal/优化异常 0 条；WARN/ERROR/FATAL 0 条。

最终 ready 汇总：`{"ready": true, "ever_ready": true, "transitions": 1, "ready_to_not_ready_count": 0, "last_transition_age_sec": 196.44977608699992}`。
首个 LIO 进程启动于 `2026-11-11T14:20:30.567407+08:00`；发生在此前的 header 异常共 1 次。这些启动前样本不代表 LIO 已运行时经历了同一跳变。

## 对照 2026-11-11_13-24-18

| 指标 | 前轮 | 本轮 |
|---|---:|---:|
| count | 21842 | 22343 |
| received_span_sec | 218.358881165 | 223.37054806 |
| received_hz | 100.0234104675419 | 100.02213897061608 |
| arrival_gap_gt_0_2_count | 0 | 0 |
| header_gap_gt_0_2_count | 1 | 1 |
| header_nonpositive_count | 0 | 0 |
| bad_quaternion_norm_count | 2 | 2 |
| hold offset 中位 ms | 357.805 | 365.909 |
| lio_start offset 中位 ms | 355.670 | 364.659 |
| nav2_start offset 中位 ms | 356.760 | 365.738 |
| sensor_baseline offset 中位 ms | 355.504 | 364.505 |
| 首尾姿态旋转度 | 0.174388066 | 0.041129834 |
| 最大相对姿态旋转度 | 0.176275958 | 0.063779832 |
| gyro 范数中位 rad/s | 0.002085462 | 0.002156362 |
| gyro 范数最大 rad/s | 0.007451149 | 0.022802111 |
| accel 范数中位 m/s² | 9.795384466 | 9.796366806 |

## 结论范围

- ROS messages do not include the original serial frames or independent device timestamp values.
- Header-minus-receipt is not an independently calibrated cross-sensor acquisition-time offset.
- Raw recording and aggregate observer have separate callbacks; their arrival jitter and phase boundaries can differ slightly.
- IMU orientation/gyro/acceleration alone cannot attribute a mapping position change to physical motion or localization error.
- Startup/freshness success does not establish correct timestamp synchronization or motion accuracy.

室内启动程序结果：`passed=True`；IMU 时间质量按上述指标独立判断。

![IMU timing](/home/akun/workspace/Car_jetson/navigation_monitoring/indoor_startup/2026-11-11_14-20-08/imu_timing.png)
