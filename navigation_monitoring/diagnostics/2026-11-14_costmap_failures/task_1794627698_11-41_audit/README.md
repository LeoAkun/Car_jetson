# task_1794627698（11:41）证据来源与限制

## 原始记录来源

- 持续观察器：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-14_11-02-27_manual_publish_monitor/detail_observer.jsonl`
  - 本任务窗口内共提取 60 条 `/plan`：JSONL 第 13289–14141 行。
  - 首次 lethal（ROS 时间 `1794627805.581230176`）前有 59 条有效 plan；之后在 `11:43:43.979+08:00` 短暂恢复 1 条。
  - 60 条 plan 的首条为 `11:42:23.050+08:00`，起点 `(-76.549995, 177.450005)`；首次 lethal 前最后一条为 `11:43:24.538+08:00`，起点 `(-77.749995, 199.550005)`。
  - 同一 ROS 时间窗口内提取 560 条 `/odom` pose：JSONL 第 13218–14379 行。
- Nav2 日志：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-14_11-38-38_manual_publish_monitor/nav2_2026-11-14_11-42-04.log`
  - 首次 `Starting point in lethal space` 位于第 102 行，ROS 时间 `1794627805.581230176`。
  - 第 102–219 行记录后续重复 lethal、全局 costmap 清除超时及最终失败。
- 导航管理器日志：`/home/akun/.ros/log/python3_13870_1794625030878.log`
  - 第 160–180 行覆盖 task 接收、重定位、Nav2 启动、目标接受和 action 失败。
- 静态地图：
  - `/home/akun/workspace/Car_jetson/multi_map_navigation/src/map/map2/map2_clean.yaml`
  - `/home/akun/workspace/Car_jetson/multi_map_navigation/src/map/map2/map2_clean.pgm`
- 足迹与 costmap 配置：`/home/akun/workspace/Car_jetson/nav2/src/nav2/config/nav2_real_params.yaml`

`plan_records.csv` 与 `odom_records.csv` 保留原 JSONL 行号、host time 和 ROS stamp。运行 `extract_observer_records.py` 可重新生成两份 CSV 与 `extract_manifest.txt`。

## 静态叠加图

- 生成脚本：`static_map_overlay_task7698.py`
- 输出：`static_map_overlay_estimated.png`

脚本用观察器中 plan 起点与 `/odom` 样本拟合一个二维刚体变换，再估算首次 lethal 时刻的 map pose，并将配置足迹叠加在 `map2_clean.pgm` 上。它是离线空间相关性分析；图中的红色 footprint 是估计值。

## 证据限制

- 本轮专用 rosbag 只覆盖 `11:38:38.846–11:40:49.823`；长 rosbag 只覆盖 `11:02:28.025–11:39:54.744`。NavigateToPose action 实际发生在 `11:42:22–11:44:09`，两者均未覆盖。
- 已扫描工作区内 462 个 `.db3`，没有数据库与 `11:42:22–11:44:09` 相交。因此本轮没有 first-lethal 时刻的原始 scan、filtered scan、TF 或 `global_costmap/costmap_raw`。
- plan 起点经过全局 costmap 栅格量化，观察器记录与 ROS 消息到达也有时间差。拟合残差约为平均 `0.085 m`、最大 `0.231 m`，因此叠加图能证明静态障碍边缘与失败位姿高度相关，不能证明当时致命格一定来自 static layer。
- 图和 CSV 不能区分 static layer、obstacle layer 残留、激光回波或 TF 瞬态对最终 master costmap 的各自贡献。需要覆盖故障时刻的完整 rosbag 才能做确定归因。
