# 2026-11-19 当前修改仿真测试报告

## 结论

**总体：不通过，禁止把本轮结果视为可部署。**

当前源码可以完整编译，核心回归测试通过，map-odom authority 单实例保护和 NavigationManager 故障取消单元测试通过。首次自定义播放器回放出现 IMU 预积分发散和 fatal，但独立复核证明这个结果不可重复：完全相同输入和当前二进制使用官方播放器做两次 1x 回放，一次明显发散，另一次保持连续；0.5x 回放也保持连续。因此不能把首次 fatal 单独归因为车辆传感器或当前算法的确定性故障。

可重复的问题是：三次官方回放都出现定位修正 fault，其中两次连续回放分别出现 8 次和 5 次 `LIO_CORRECTION_REQUIRES_RELOCALIZATION`。如果接入 NavigationManager，会中止当前导航。当前测试没有运行 NavigationManager、map-odom authority、Nav2 或车辆动力学，也没有真值轨迹，不能证明定位精度、任务取消链路或车辆可部署性。

本轮没有启动底盘、ECU、CAN、MQTT、门控、真实传感器驱动或实际任务。所有 ROS 测试均使用 `ROS_LOCALHOST_ONLY=1` 和隔离 domain。

## 测试环境

- Git HEAD: `0dba9db0fe0a598392b780695692048484662bcb`
- 分支：`codex/traffic-light-p0`
- 测试 overlay：本目录下 `install/`
- LIO instance：`sim-20261119-1328`
- 构建包：`ndt_omp_ros2`、`multi_map_navigation_msgs`、`multi_map_navigation`、`lio_sam`、`re_localization`、`nav2_init_pose`

## 结果矩阵

| 项目 | 结果 | 证据 |
|---|---|---|
| 当前源码隔离构建 | 通过 | 6 个包完成；LIO、重定位、消息和 Python 包均成功 |
| Python 编译检查 | 通过 | 源码目录定向检查无语法错误 |
| 核心回归 | 通过 | 224 passed，2 个第三方 NumPy deprecated warning |
| fault→Nav2 取消聚焦回归 | 通过 | 12 passed；取消目标、零速度消息模拟、清理、旧目标隔离；未连接车辆或发布驻车命令 |
| map-odom authority 主实例启动 | 通过 | `/map_odom_authority` 可发现 |
| map-odom authority 重复实例保护 | 通过 | 第二实例退出码 1，提示已有实例持有 TF 所有权 |
| NavigationManager 旧端到端夹具 | 阻塞 | 缺少当前必需的 `/route_planner/pub_new_path` mock，构造函数持续等待 |
| LIO 45 秒自定义播放器回放 | **失败但不可重复** | IMU 预积分速度约 38 m/s，readiness 变 false，进程 fatal 退出 |
| 官方播放器独立复核 | **不稳定** | 相同输入的两次 1x 结果差异很大；0.5x 连续，但仍有 5 次中修正 fault |

## 输入时间戳

输入窗口包含 4,483 条 `/imu`、365 帧 `/rslidar_Pointcloud2` 和 8 条 `/tf_static`。

- IMU 头时间严格单调：周期中位数 10 ms，最大 90 ms，无倒退、无超过 200 ms 的头时间断流。
- IMU `录包接收时间 - header` 中位数为 **-0.369434 s**，即 IMU header 持续超前接收时间约 369 ms。
- LiDAR `录包接收时间 - header` 中位数为 **+0.103991 s**，即 LiDAR header 落后接收时间约 104 ms。
- 按录包接收时间比较，IMU 与 LiDAR 的发布/记录延迟中位数相差约 **0.473 s**。这包含驱动、点云组包、网络、DDS 和 rosbag 回调延迟，不能直接解释为两个传感器时钟基准相差 0.473 s。
- 以 LiDAR header 为基准寻找最近的 IMU header，绝对差中位数为 **1.477 ms**，99 分位为 **1.945 ms**。除截取窗口第一帧缺少前置 IMU 外，两个 header 时间轴实际高度对齐。
- IMU 第 1247 个样本处，录包接收间隔出现 341.77 ms，而 header 只前进 10 ms，随后多帧以微秒级被 rosbag 记录。录包只能证明 recorder 观察到了排队/调度停顿，不能区分停顿发生在驱动、DDS 还是 recorder 回调。

## 首次自定义播放器输出

| 指标 | 结果 |
|---|---:|
| correction event | 23（接受 6，拒绝 17） |
| epoch odometry | 3,806，epoch 2～21，未倒退，instance/generation 一致 |
| `/odom` 样本 | 1,009 |
| `/odom` 最大相邻平移 | 0.024620 m |
| `/odom` 99.9 分位相邻平移 | 0.024570 m |
| `/odom` 超过 0.05 m / 1° | 0 / 0 |
| `/odom` 最大时间空洞 | 14.24 s |
| `/odom_incremental` 最大相邻平移 | 19.5295 m |
| `/odom_incremental` 超过 0.05 m | 2,424 |
| fault | 7（1 次 scan-to-map unreliable，6 次 continuity fault） |

`/odom` 的 0.05 m 硬门限挡住了大步长直接输出，但没有达到建议的 99.9 分位 `<=0.02 m`，且长时间停止更新。内部预积分状态继续发散，最终因 `velocity_norm=38.004892` 触发 fatal。故障后 readiness 正确变为 false。

该播放器按 rosbag 接收时间调度，却没有发布 `/clock`，LIO 节点也没有统一使用 `use_sim_time`。历史 header 与当前墙钟相差约 26 小时，诊断中的 `arrival_delay_sec` 因而约为 94,675 秒。它还会原样重现 recorder 的 331.77 ms 停顿和突发发送。该结果可以作为压力失败样本，不能作为标准离线回放基线。

## 独立复核

独立复核使用同一个 45 秒输入、同一套隔离构建、官方 `ros2 bag play --clock`，四个 LIO 节点统一设置 `use_sim_time=true`。截取包与原包对应窗口的 4,856 条序列化消息逐条一致。

| 回放 | correction | `/odom` 最大步长 | `/odom` 99.9 分位 | 最大空洞 | fault |
|---|---:|---:|---:|---:|---:|
| 1x 第一次 | 86，接受 31 | 0.049953 m | 0.049617 m | 2.33 s | 18 |
| 1x 第二次 | 98，接受 97 | 0.004686 m | 0.001541 m | 0.09 s | 8 |
| 0.5x | 95，接受 93 | 0.005787 m | 0.001384 m | 0.09 s | 5 |

三次 `/odom` 都没有越过 `0.05 m / 1°` 硬门限，说明连续性保护有效。第一次 1x 的输出长期贴近 5 cm 限幅，只能说明保护层在削峰，不能说明定位准确。相同输入、相同二进制的两次 1x 结果差异巨大，说明回放受执行调度、最佳努力 QoS、深度为 1～5 的队列和处理吞吐影响；单次结果不能用于确定性归因。

当前统计脚本按相邻消息计算位移，但没有把样本限定为 10 ms，也没有按 `dt` 计算速度。存在输出空洞时，不能把所有相邻步长直接表述为“每 10 ms 位移”。

## 处理建议

1. 标准回放固定使用官方播放器、`--clock` 和 `use_sim_time=true`；输入窗口增加至少 1 秒前置 IMU，并记录 deskew、feature cloud_info 和每级收发计数。
2. 将算法正确性与实时性能分开：0.25x～0.5x 至少重复 3 次验证确定性，再用 1x 重复 5 次评估队列丢帧和 CPU 负载。验收报告必须给出每一级点云保留率。
3. 统计按 `dt` 分桶：10 ms 连续样本计算步长分位，同时单独报告速度、超过 200 ms 的断流、fault storm 和输出限幅占比。
4. 检查为何正常回放仍出现 5～8 次中修正 fault；在原因明确前，当前 `0.05 m` 中修正策略会导致导航频繁取消。
5. 补齐 NavigationManager 端到端夹具中的 `PubNewPath` mock，并在同一隔离 domain 启动 authority、NavigationManager 和 Nav2 mock，验证 fault 到 action cancel 的真实话题链路。
6. 检查 IMU 和点云驱动时间戳生成以及 recorder 接收抖动，但不要依据 `bag timestamp - header` 单项数据修改传感器时间补偿。
