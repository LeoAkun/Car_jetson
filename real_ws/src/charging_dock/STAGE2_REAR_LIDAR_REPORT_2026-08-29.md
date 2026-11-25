# 阶段 2 后雷达离线几何算法报告

> 归档说明（2026-09-18）：本报告保留阶段 2 的历史结果；`stage2_lidar_replay` 离线回放命令已在阶段 5 控制链稳定后移除，运行时仍保留并复用 `RearLidarProcessor`。

日期：2026-08-29  
范围：`FIXED_TRAJECTORY_LIDAR_DOCKING_PLAN.md` 阶段 2  
结论：阶段 2 已完成；正式 48 包全部通过，75 个带条件包的通过集合与阶段 1 正式集完全一致。

## 1. 已实现内容

1. `rear_lidar_processor.py`
   - LaserScan 合法性和时间戳检查。
   - ROI、连续段分割、点数/宽度/残差门控。
   - 直线拟合、径向距离、桩面法向距离和航向输出。
   - `rear_laser -> base_link` 几何转换。
   - 连续 10 帧稳定门和 0.10 m 目标跳变保护。
   - 默认禁止横向中心输出，防止从无限墙面或局部线段虚构桩体中心。
2. `stage2_lidar_replay.py`
   - rosbag 离线批处理。
   - 每包 JSON/CSV 明细和跨包汇总。
   - 帧数、频率、帧间隔、有效率、距离、航向、残差及目标切换验收。
3. `rear_lidar.yaml`
   - 固化后雷达外参和算法门限。
   - 固化最终停车窗口：`base_link` 到桩体平面 `0.45~0.47 m`。
4. 自动测试覆盖空场、孤立人员类回波、窄箱体、无限墙面、过宽结构、错误 frame、过期消息和目标跳变。

## 2. 正式数据结果

正式输入：16 个工况、48 个 rosbag。

| 指标 | 最小 | 中位 | 最大 | 门限 |
| --- | ---: | ---: | ---: | ---: |
| 分析帧数 | 285 | 356 | 381 | `>=250` |
| 发布频率 | 29.7947 Hz | 29.8006 Hz | 30.5245 Hz | `>=25 Hz` |
| 最大帧间隔 | 34.54 ms | 34.80 ms | 36.89 ms | `<=200 ms` |
| 稳定有效帧率 | 96.58% | 97.45% | 97.64% | `>=95%` |
| 径向距离标准差 | 0.797 mm | 1.234 mm | 2.858 mm | `<=10 mm` |
| 航向标准差 | 0.073 deg | 0.203 deg | 0.625 deg | `<=1 deg` |
| 拟合残差均值 | 0.801 mm | 1.129 mm | 2.207 mm | `<=10 mm` |
| 目标切换次数 | 0 | 0 | 0 | `0` |

正式结果：`48/48` 通过。

有效率低于阶段 1 的 100% 是预期行为：每包开始的 10 帧用于时序稳定预热，不对外标记为有效观测。

## 3. 全量审计和负样本

对 75 个带条件 rosbag 运行同一套处理和验收门：

- 通过：48 包。
- 拒绝：27 包。
- 通过集合与阶段 1 正式 48 包精确一致。
- 27 个异常包的主要拒绝原因：有效率 26 包、最大帧间隔 19 包、距离偏差 7 包、距离方差 6 包、拟合残差 6 包、航向波动 6 包、帧数不足 2 包、目标切换 1 包。单个包可同时触发多个原因。
- 所有正式和审计包的横向中心输出率均为 0，未从长墙面或局部直线拟合结果生成虚假中心。

确定性合成负样本测试：

- 空场景：无目标，输出无效。
- 孤立人员类少量回波：点数不足，输出无效。
- 窄箱体类回波：可见宽度不足，输出无效。
- 无限墙面：可输出距离/航向几何，但 `center_available=false`。
- 过宽结构：超过 0.60 m 候选宽度，输出无效。

## 4. 固定参数

```text
final_parking_window_min_m = 0.45
final_parking_window_max_m = 0.47
sensor_xyz_yaw = (-0.43 m, -0.28 m, -135 deg)
message_timeout = 0.20 s
minimum_points = 5
segment_jump = 0.08 m
stable_frames = 10
target_distance_jump = 0.10 m
visible_width = 0.05~0.60 m
maximum_fit_residual = 0.01 m
lateral_center_enabled = false
```

历史包包含旧 `frame_id=laser`，批量审计通过显式命令行兼容该值；在线默认仍只接受 `rear_laser`。

## 5. 输出文件

- 正式汇总：`test_data/docking_stage2/stage2_formal_summary.csv`
- 正式逐帧结果：`test_data/docking_stage2/formal/*.stage2.json|csv`
- 全量汇总：`test_data/docking_stage2/stage2_all_conditioned_summary.csv`
- 全量逐帧结果：`test_data/docking_stage2/audit/*.stage2.json|csv`

同一 `0.45 m` 正式包重复运行两次，JSON SHA-256 均为：

```text
7f3f0dfa97d84e36615da5eed920f942f7e6f0fd161e00ba098651a5aa1c7968
```

输出逐字节一致。

## 6. 验证命令

```bash
cd /home/akun/workspace/Car_jetson/real_ws/src/charging_dock
source /opt/ros/humble/setup.bash
python3 -m pytest -q test
```

结果：`36 passed`。

构建命令：

```bash
cd /home/akun/workspace/Car_jetson/real_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select charging_dock --symlink-install
```

阶段 2 处理器不发布 `/cmd_vel`、`/cmd_vel_dock` 或任何其他底盘控制指令。
