# 真实 IMU 连续性与时间分析：2026-11-11_13-24-18

本报告仅离线读取已结束运行的原始消息和日志。所有精确样本时间均为北京时间 UTC+8；`header − received` 为正表示消息抵达观察节点时 header 已指向未来。

共 21842 条 IMU，实际接收跨度 218.359 秒，100.023 Hz。
接收间隔中位 / P99 / 最大为 10.012 / 11.050 / 23.110 ms。
接收 >0.2 秒空洞 0；header >0.2 秒间隔 1；header 回退 / 重复 0 / 0。
header − received 中位 / P99 / 最大为 357.445 / 359.822 / 360.296 ms。

| 阶段 | IMU 条数 | 接收 Hz | 接收最大 ms | header 最大 ms | offset 中位 ms | offset 最大 ms | 异常四元数范数 |
|---|---:|---:|---:|---:|---:|---:|---:|
| sensor_baseline | 1933 | 100.245 | 15.776 | 320.000 | 355.504 | 356.785 | 2 |
| lio_start | 307 | 100.430 | 23.110 | 10.040 | 355.670 | 356.802 | 0 |
| nav2_start | 1602 | 99.996 | 13.469 | 10.042 | 356.760 | 357.127 | 0 |
| hold | 18000 | 100.002 | 15.603 | 10.044 | 357.805 | 360.296 | 0 |

第 2–7 秒 offset 中位 355.383 ms，最后 5 秒 359.290 ms，差 3.906 ms。时间变化不单独证明设备振荡器、host 时钟或消息延迟中哪个是原因。

角速度非有限 0；加速度非有限 0；四元数非有限 0、近零范数（<1e-6，非仅字面零）1、范数偏离 1 超过 0.1 共 2 条。完整轴向及范数分布见 JSON。

## 姿态、角速度与加速度

有效四元数 21840 条、排除 2 条；先归一化再计算姿态，yaw 连续展开，q 与 -q 视作同一旋转。

| 角度（度） | 首条有效值 | 末条有效值 | 最小 | 最大 | 首尾差 |
|---|---:|---:|---:|---:|---:|
| roll | 0.579783 | 0.578248 | 0.563981 | 0.594731 | -0.001534 |
| pitch | 0.109930 | 0.093289 | 0.090504 | 0.121168 | -0.016640 |
| yaw | 75.439903 | 75.613486 | 75.426036 | 75.615303 | 0.173583 |

首尾旋转夹角 0.174388°；相对首条最大旋转 0.176276°，发生在 `2026-11-11 13:27:59.512985512+08:00`。

| 阶段 | 有效姿态条数 | 首尾旋转度 | 相对阶段首条最大旋转度 |
|---|---:|---:|---:|
| sensor_baseline | 1931 | 0.016188 | 0.017847 |
| lio_start | 307 | 0.014846 | 0.016694 |
| nav2_start | 1602 | 0.035884 | 0.036143 |
| hold | 18000 | 0.146599 | 0.148410 |

角速度向量范数中位 / P99 / 最大：0.002085462 / 0.004888145 / 0.007451149 rad/s。
加速度向量范数最小 / 中位 / P99 / 最大：9.666170715 / 9.795384466 / 9.892649545 / 9.936754682 m/s²。
这些测量描述 IMU 自身记录的姿态和运动量，不能单独判断 mapping 位置变化来自人工移动还是定位误差；加速度包含重力，不能据此排除匀速平移。

## 时间戳和四元数异常样本

- header 异常第 6 行：received `2026-11-11 13:24:21.531150409+08:00`，header `2026-11-11 13:24:21.882182400+08:00`；接收间隔 1.390256 ms，header 间隔 320.000000 ms，offset 351.031991 ms，q 范数 `3.660730276894887e+36`。
- quaternion 异常第 1 行：received `2026-11-11 13:24:21.523678198+08:00`，header `2026-11-11 13:24:21.522182400+08:00`；接收间隔 — ms，header 间隔 — ms，offset -1.495798 ms，q 范数 `4.6028593522900777e-35`。
- quaternion 异常第 6 行：received `2026-11-11 13:24:21.531150409+08:00`，header `2026-11-11 13:24:21.882182400+08:00`；接收间隔 1.390256 ms，header 间隔 320.000000 ms，offset 351.031991 ms，q 范数 `3.660730276894887e+36`。

每个异常前 5 条、后 3 条完整 q/gyro/accel 与整数纳秒时间戳已保存在 JSON；启动前 15 条也单独保留。

## CRC、设备时间与 LIO 日志

- `2026-11-11 13:24:21.524846584+08:00`：`[WARN] [1794374661.524846584] [ahrs_bringup]: check crc16 failed (ahrs), total_crc_errors=1`；最近 header 异常行 6 的接收时间比 CRC 日志晚 6.303825 ms。
CRC 与样本异常时间相近只能证明时序关联，不能仅凭 ROS 消息证明底层因果。
- 短读：`[WARN] [1794374888.830643558] [ahrs_bringup]: Short IMU serial read for frame type: received=0 expected=1 total_short_reads=1`；是否在关闭信号后：True。
- 设备时间 reset/re-anchor/non-monotonic 诊断：0 条。
- `lio_featureExtraction.log`：gap/reset/recovery 0 条；fatal/优化异常 0 条；WARN/ERROR/FATAL 0 条。
- `lio_imageProjection.log`：gap/reset/recovery 0 条；fatal/优化异常 0 条；WARN/ERROR/FATAL 0 条。
- `lio_imuPreintegration.log`：gap/reset/recovery 0 条；fatal/优化异常 0 条；WARN/ERROR/FATAL 0 条。
- `lio_mapOptimization.log`：gap/reset/recovery 0 条；fatal/优化异常 0 条；WARN/ERROR/FATAL 0 条。

最终 ready 汇总：`{"ready": true, "ever_ready": true, "transitions": 1, "ready_to_not_ready_count": 0, "last_transition_age_sec": 196.68697941399978}`。
首个 LIO 进程启动于 `2026-11-11T13:24:40.813663+08:00`；发生在此前的 header 异常共 1 次。这些启动前样本不代表 LIO 已运行时经历了同一跳变。

## 对照 2026-11-11_13-08-19

| 指标 | 前轮 | 本轮 |
|---|---:|---:|
| count | 16246 | 21842 |
| received_span_sec | 162.411666758 | 218.358881165 |
| received_hz | 100.02360251745776 | 100.0234104675419 |
| arrival_gap_gt_0_2_count | 0 | 0 |
| header_gap_gt_0_2_count | 1 | 1 |
| header_nonpositive_count | 0 | 0 |
| bad_quaternion_norm_count | 0 | 2 |
| hold offset 中位 ms | 354.330 | 357.805 |
| lio_start offset 中位 ms | 351.154 | 355.670 |
| nav2_start offset 中位 ms | 352.531 | 356.760 |
| sensor_baseline offset 中位 ms | 350.552 | 355.504 |
| 首尾姿态旋转度 | 0.019396374 | 0.174388066 |
| 最大相对姿态旋转度 | 0.050299160 | 0.176275958 |
| gyro 范数中位 rad/s | 0.002147358 | 0.002085462 |
| gyro 范数最大 rad/s | 0.007415974 | 0.007451149 |
| accel 范数中位 m/s² | 9.796526735 | 9.795384466 |

## 结论范围

- ROS messages do not include the original serial frames or independent device timestamp values.
- Header-minus-receipt is not an independently calibrated cross-sensor acquisition-time offset.
- Raw recording and aggregate observer have separate callbacks; their arrival jitter and phase boundaries can differ slightly.
- IMU orientation/gyro/acceleration alone cannot attribute a mapping position change to physical motion or localization error.
- Startup/freshness success does not establish correct timestamp synchronization or motion accuracy.

室内启动程序结果：`passed=True`；IMU 时间质量按上述指标独立判断。

![IMU timing](/home/akun/workspace/Car_jetson/navigation_monitoring/indoor_startup/2026-11-11_13-24-18/imu_timing.png)
