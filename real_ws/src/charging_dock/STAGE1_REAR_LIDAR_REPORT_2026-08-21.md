# 阶段 1 后单线雷达专项验证报告

> 归档说明（2026-09-18）：本报告保留阶段 1 的历史结果；其中 `stage1_lidar_capture` 和 `stage1_lidar_analyze` 离线测试命令已在阶段 5 控制链稳定后移除，不属于当前运行接口。

日期：2026-08-21  
范围：`FIXED_TRAJECTORY_LIDAR_DOCKING_PLAN.md` 阶段 1  
结论：软件、TF 和在线传输基线已完成；历史雷达矩阵与实际充电窗口正对数据已统一归档，当前仍因里程计随包记录缺失等问题不能宣布阶段 1 整体性能验收通过。

## 1. 已完成内容

1. 两路原始扫描使用独立坐标系：
   - `/laser/scan_front`：`front_laser`
   - `/laser/scan_back`：`rear_laser`
2. 已发布并在线验证静态 TF：
   - `base_link -> front_laser`：`xyz=(0.43, 0.28, 0.15) m`，`yaw=45 deg`
   - `base_link -> rear_laser`：`xyz=(-0.43, -0.28, 0.15) m`，`yaw=225 deg`（等价于 `-135 deg`）
3. 已同步双雷达融合器的安装偏移。
4. 已实现单工况 rosbag 采集、条件元数据和离线统计工具。
5. 已录制 TF 部署前、部署后两份基线 bag。
6. 已修复整周 ROI `[-180, 180] deg` 被错误归一化为单角度的分析缺陷，并添加回归测试。

## 2. 在线验证结果

2026-08-21 16:20 CST 启动本仓库构建的双雷达驱动：

```bash
systemctl --user status car-jetson-stage1-lidar
```

服务状态为 `active (running)`。两个扫描话题均有 1 个发布者，消息头分别为：

```text
/laser/scan_front.header.frame_id = front_laser
/laser/scan_back.header.frame_id  = rear_laser
```

TF 在线测量值：

```text
base_link -> front_laser
translation = [ 0.430,  0.280, 0.150] m
RPY         = [ 0.000,  0.000, 45.000] deg

base_link -> rear_laser
translation = [-0.430, -0.280, 0.150] m
RPY         = [ 0.000,  0.000, -135.000] deg
```

当前整车父 launch 来自 `/home/akun/workspace/sensor/real_ws/install`，本次新版雷达通过用户级 transient systemd 服务单独运行。该服务不会跨主机重启永久保留；正式部署时必须停止旧硬件栈，并从 `/home/akun/workspace/Car_jetson/real_ws/install/setup.bash` 启动整车 launch，避免旧安装覆盖本仓库版本。

## 3. 部署前后基线

| 指标 | 部署前 | 部署后 |
| --- | ---: | ---: |
| bag 时长 | `11.68 s` | `13.77 s` |
| 后雷达消息 | `349` | `380` |
| 前雷达消息 | `316` | `379` |
| 后雷达分析帧（首尾各裁 1 s） | `289` | `320` |
| 后雷达频率 | `29.798 Hz` | `29.800 Hz` |
| 后雷达最大帧间隔 | `0.0346 s` | `0.0347 s` |
| 后雷达 frame | `laser` | `rear_laser` |
| `tf_static` 消息 | `6` | `8` |
| `/vehicle_status` 消息 | `1159` | `1235` |
| 里程计消息 | `0` | `0` |

部署后 bag：

```text
/home/akun/workspace/Car_jetson/test_data/docking_stage1/baseline_post_tf_20260821_1624
```

部署后分析 JSON：

```text
/home/akun/workspace/Car_jetson/test_data/docking_stage1/baseline_post_tf_20260821_1624.stage1.json
```

后雷达传输门槛结果：

| 检查项 | 实测 | 门槛 | 结果 |
| --- | ---: | ---: | --- |
| 分析帧数 | `320` | `>=250` | 通过 |
| 发布频率 | `29.800 Hz` | `>=25 Hz` | 通过 |
| 最大帧间隔 | `0.0347 s` | `<=0.20 s` | 通过 |
| 消息年龄 P95 | `0.000552 s` | 记录项 | 正常 |
| 全扫描有效回波比例 | `67.76%` | 记录项 | 有回波 |

全角度分析会把车体或环境中的最大连续段当作候选。其距离均值 `0.2708 m`、标准差 `0.0042 m` 不能解释为充电桩性能，也不能用于确定停止距离、ROI 或可信最小距离。由于没有目标真值，工具正确给出 `provisional_pass=false`。

## 4. 里程计阻塞项

当前 ROS 图中唯一的 `nav_msgs/msg/Odometry` 话题是 `/NED_odometry`。它有 1 个发布者，但部署后 13.77 秒 bag 中消息数为 `0`；`/odom` 和两个 LIO-SAM 里程计话题不存在。

`fdilink_ahrs` 仅在收到 `TYPE_INSGPS` 数据包时发布 `/NED_odometry`。当前设备或串口数据流没有提供该类数据，因此“bag 同时包含里程计消息”的阶段 1 要求尚未满足。开始正式矩阵前必须选择并验证一个持续发布的定位/里程计源。

验证命令：

```bash
ros2 topic hz /NED_odometry
ros2 topic echo --once /NED_odometry
```

若 10 秒内仍没有消息，不得把仅存在 publisher 的状态记录为“里程计正常”。

## 5. 正式采集矩阵

统一数据集的实际使用距离为 `0.45、0.47、0.50、0.75、1.00、1.50、2.00、2.50 m`；历史 `0.15~0.40 m` 数据保留归档。实际充电窗口只要求 `0.45、0.47 m / 0 deg`，`0.42 m` 因外壳贴合取消。  
远距离/中距离角度数据沿用已采集的 `-15, -10, -5, 0, 5, 10, 15 deg` 工况；窗口点不再强制大角度。  
每个正式工况通常 3 次重复。统一数据集当前为 `16` 个工况、`48` 个正式包，见 `test_data/docking_stage1/STAGE1_UNIFIED_DATASET.md`。

在 RViz 中确认桩体位于后雷达局部坐标系的中心角后，将下方 `<ROI_CENTER_DEG>` 替换为实测值。距离必须明确以桩体参考面到后雷达扫描原点为准；不能混用 `base_link` 距离。

单工况采集命令示例：

```bash
source /opt/ros/humble/setup.bash
source /home/akun/workspace/Car_jetson/real_ws/install/setup.bash

ros2 run charging_dock stage1_lidar_capture \
  --distance-m 0.45 \
  --angle-deg 0 \
  --repeat 1 \
  --roi-center-deg <ROI_CENTER_DEG> \
  --roi-half-width-deg 20 \
  --duration-sec 15 \
  --note "dock target, static vehicle"
```

每次摆放后必须重新测量实际距离和角度，再执行采集；不允许在无人移动目标的情况下自动循环生成矩阵包。

完成采集后的批量分析：

```bash
mapfile -t bags < <(find \
  /home/akun/workspace/Car_jetson/test_data/docking_stage1 \
  -mindepth 1 -maxdepth 1 -type d -name 'd*_a*_r*' | sort)

ros2 run charging_dock stage1_lidar_analyze \
  "${bags[@]}" \
  --summary-csv \
  /home/akun/workspace/Car_jetson/test_data/docking_stage1/stage1_summary.csv
```

## 6. 每包验收指标

| 指标 | 暂定门槛 |
| --- | ---: |
| 有效分析帧数 | `>=250` |
| 发布频率 | `>=25 Hz` |
| 最大帧间隔 | `<=0.20 s` |
| 有效目标帧率 | `>=95%` |
| 距离标准差 | `<=0.01 m` |
| 距离绝对偏差 | `<=0.02 m` |
| 离群率 | `<=1%` |

有限宽度结构只有在可见宽度及中心位置跨帧稳定、并且不会选择背景墙时才算可分割。无限墙面只能提供距离和航向，不能输出横向中心。

## 7. 阶段 1 完成判定

| 任务 | 状态 | 证据或缺口 |
| --- | --- | --- |
| 独立 `front_laser` / `rear_laser` | 已完成 | 在线消息头验证 |
| 静态 TF | 已完成 | `tf2_echo` 在线验证 |
| 统一距离静态采集 | 已完成（静态部分） | 16 个工况、48 个正式包；`0.42 m` 因外壳贴合取消 |
| 7 个角度测试 | 已完成（2.50 m） | `-15、-10、-5、0、+5、+10、+15 deg` 均有正式包；角度绝对真值仍为目测 |
| 有效回波/均值/标准差/离群率 | 已完成（正式包） | 48/48 通过当前静态门槛；全量审计另见批量报告 |
| 有限宽度结构可分割性 | 部分完成 | 右侧目标 ROI 已确认并稳定拟合；真实两端中心误差仍未标定 |
| 可信最小距离和有效 ROI | ROI 已确定，最小距离待充电验证 | 当前窗口为 `0.45/0.47 m / 0 deg`，不能直接写成最终控制阈值 |
| rosbag 含后雷达、TF、里程计 | 部分完成 | 后雷达和 TF 已有，里程计 0 条 |

因此当前可确认的是“TF、雷达传输及已采集工况的静态指标通过”，不能确认“阶段 1 整体通过”。统一数据集见 `test_data/docking_stage1/STAGE1_UNIFIED_DATASET.md` 和 `test_data/docking_stage1/stage1_unified_summary.csv`。进入阶段 2 前仍需补齐必要工况/实际充电连接验证，并解决里程计随包记录、外壳/接收面板几何和可信最小距离确认问题。

## 8. 2026-08-29 批量复核补充

已对统一数据集的 48 个正式 rosbag 重新执行同一分析器（首尾各裁剪 1 s，ROI 从每包条件 YAML 读取）：

| 项目 | 结果 |
| --- | ---: |
| 正式工况 | 16 |
| 正式包 | 48 |
| 静态门槛通过 | 48 / 48 |
| 最小/中位/最大分析帧数 | 285 / 356 / 381 |
| 频率范围 | 29.7947~30.5246 Hz |
| 最大帧间隔范围 | 34.54~36.89 ms |
| 有效目标帧率 | 100%（正式包） |
| 距离标准差范围 | 0.799~2.825 mm |
| 离群率范围 | 0~0.321% |

距离验收语义保持分层：1.50 m / 0 deg 的 3 个正式包使用人工法向真值 `1.15 m`，其余 45 包使用各工况独立人工径向值进行目标段选择和静态一致性验收。径向距离不能直接替代 `base_link` 到桩体平面的距离。

75 个带条件包的全量审计也已完成，结果为 48 / 75 通过；未通过的 27 个包主要是历史错误结构、断流、近场污染或不计入正式重复的替代尝试，不能覆盖正式 48 包的判定。批量结果文件：

- `test_data/docking_stage1/stage1_formal_right_batch_summary_20260829.csv`
- `test_data/docking_stage1/stage1_batch_summary_20260829.csv`

本次复核还修正了 1.00 m 正式包条件 YAML 中残留的旧 ROI（原为 `82~97 deg`，会选中约 `1.02 m` 结构）；现统一为人工确认的右侧目标 `61~78 deg`，对应径向真值 `0.71 m`。原始 rosbag 未修改。

## 9. 阶段 1 剩余现场任务

批量离线任务已经完成，以下项目不能由现有 rosbag 推断，仍需实车执行并把新包/记录补入报告：

1. 让一个连续发布的定位/里程计源与后雷达同时录包，并确认 `/NED_odometry` 或替代话题消息数和频率大于 0；当前 48 个正式包该话题均为 0 条。
2. 在 `0.45 m / 0 deg`、`0.47 m / 0 deg` 各执行实际充电连接，记录连接成功、连接时间、充电电流/功率、外壳最小间隙和是否接触。
3. 实测 `base_link -> receiver_link`、外壳几何以及桩端参考面；将结果写入独立对接配置，不能直接把 `0.45~0.47 m` 当作控制器最终阈值。
4. 完成上述现场记录后，再决定是否进入阶段 2 的离线控制算法联调；在此之前禁止自动倒车对接。
