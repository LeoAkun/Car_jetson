# LIO-SAM IMU 预积分 GTSAM 健壮性修复

## 现场结论

- 本轮 2026-11-10 15:53:21 的异常为 `gtsam::IndeterminantLinearSystemException`，求解器报告 nearby variable `x3`。
- 2026-11-09 15:49:01 的同类异常报告 `b0`。该变量是求解器发现病态线性系统的位置，不保证是病因来源。
- 本轮和 11 月 9 日都没有保存异常瞬间的线性系统或 core，因此不能把某一个初始化或时间条件写成已证实的唯一根因。

## 代码修改

生产源码：`/home/akun/workspace/Car_jetson/slam/src/LIO-SAM/src/imuPreintegration.cpp`

- 保留 TransformFusion 弹空队列后的保护。
- 初始化 `prevState_`、`prevStateOdom` 和 `prevBiasOdom`，使首个预测与初始位姿、速度和偏置先验一致。
- 只有本轮存在有效 IMU 样本且累计积分时间有效时才创建 IMU 因子；零样本、零累计时间只等待，不构造奇异因子。
- IMU 首帧回退间隔改为本车实测频率对应的 `1/100 s`；有效单帧 `dt` 范围为 `(1e-6, 0.1] s`。
- 拒绝时间回退、重复、NaN/Inf、超过 0.1 秒的 IMU 间隔以及非有限校正/状态。
- 每次优化后无条件重置实时积分器，并把重传播最后时间同步给实时追加积分。
- 捕获初始化、增量更新、重线性化、结果提取、重传播和实时预测处的 GTSAM/标准异常。
- GTSAM 病态系统发生时记录阶段、key、nearby variable、样本数、累计 `dt`、队列长度和时间锚，随后清图、重建 ISAM2、重置积分器并清队列。
- 故障处理后调用 ROS shutdown，`main()` 返回 `EXIT_FAILURE`。当前任务不会在静默复位后继续；现有 LIO 子进程联锁会终止导航任务。

## 验证

- `git diff --check`：通过。
- 最终目标构建：`cmake --build build/lio_sam --target lio_sam_imuPreintegration -j2`，退出码 0。
- 安装：`cmake --install build/lio_sam`，退出码 0；安装日志显示刷新 `lio_sam_imuPreintegration`。
- 源码 SHA256：`a8a3a589e1717efdcc4f7d94b4898ca36132c69c2f9cf628fa1e895fb87410f3`。
- 安装二进制 SHA256：`9075976a5c79503ef574ad0651301468c4e33f1540b02e0346ac8bc8b1a1bff2`。
- Build ID：`97c2c4aaf2f0009f0000c021c39cb89e408e5409`。
- build/install `.text`：均为 445197 bytes，SHA256 均为 `9b4315fb4832266ff7d74d5956f9962cec5f082c3f4238619a65fd818b870324`。
- 安装二进制包含新的结构化 FATAL 日志和零样本等待日志。
- 11 月 9 日实际首窗口 10 帧，每帧 `dt=0.009899855..0.010138035 s`，累计 `0.100204706 s`；全部通过新单帧规则，离线 GTSAM 初始化和两次 update 均通过。
- 边界检查确认 `0`、负数、NaN、Inf、`0.100001`、本轮 `0.4801` 和 `0.6799 s` 缺口均被拒绝。
- 独立源码复核未发现阻断级问题。

验证过程没有启动 ROS、传感器、导航、底盘，也没有回放录包。

## 已知边界

- 补丁解决异常未捕获导致 SIGABRT、状态初始化和无效积分进入因子图等明确健壮性缺口；它不证明已锁定 GTSAM 病态矩阵的唯一来源。
- process manager 通过子进程健康检查发现退出。补充修复后，已到达 `HEALTHY` 的进程组一旦缺少必需子进程就会在下一次约 1 秒健康检查时立即判定故障；启动阶段仍保留连续缺失阈值以过滤启动抖动。相关 21 项进程健康测试通过。
