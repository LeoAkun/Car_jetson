# 旧 footprint 长宽轴核验与影响审计

日期：2026-11-12

本轮只读源码、配置、Git 历史和已停止的录包数据库；没有启动 ROS、传感器、底盘、导航或录包回放，也没有修改导航参数。

## 结论

旧 local costmap 的 footprint 确实把车辆长宽轴写反，并且在实车运行中实际生效，不是仅存在于未使用的配置中。

- 项目坐标约定为 `base_link +x` 向前、`+y` 向左。
- 旧配置为 `x=±0.27 m、y=±0.43 m`，原始尺寸是前后 `0.54 m`、左右 `0.86 m`。
- 当时未显式设置 `footprint_padding`，Nav2 使用默认 `0.01 m`；运行时有效尺寸为前后 `0.56 m`、左右 `0.88 m`。
- 2026-11-07 录包中的 2756 帧 `/local_costmap/published_footprint`，短边均约 `0.56 m`、长边均约 `0.88 m`。开头、中间、结尾样本转回车体方向后都是 `x=±0.28 m、y=±0.44 m`。
- 有效 footprint 字符串会使 Nav2 关闭 radius 模式，所以同段配置中的 `robot_radius: 0.45` 没有覆盖该矩形。

用户后续更新的实测信息为：车体约长 `0.84 m`、宽 `0.54 m`；前左和后右雷达中心的纵向、横向跨度约为 `0.86 m × 0.56 m`；按雷达最外沿计总包络约为 `0.90 m × 0.59 m`。以 `base_link` 位于外廓中点为前提：

| 几何 | 前后半长 x | 左右半宽 y | 总尺寸 |
|---|---:|---:|---:|
| 车体估计 | 0.42 m | 0.27 m | 0.84 × 0.54 m |
| 雷达中心范围 | 0.43 m | 0.28 m | 0.86 × 0.56 m |
| 雷达最外沿总包络 | 0.45 m | 0.295 m | 0.90 × 0.59 m |
| 旧配置原值 | 0.27 m | 0.43 m | 0.54 × 0.86 m |
| 旧运行值（默认 padding 后） | 0.28 m | 0.44 m | 0.56 × 0.88 m |
| 当前运行值（0.02 m padding 后） | 0.45 m | 0.30 m | 0.90 × 0.60 m |

相对雷达中心范围，旧运行轮廓每端少覆盖约 `0.15 m` 的前后空间，并在左右各多覆盖约 `0.16 m`。相对更新后的车体估计，则每端少覆盖约 `0.14 m`、左右各多覆盖约 `0.17 m`。

## 为什么能够确认坐标轴

1. 双雷达生产启动文件把前左雷达放在 `(x,y)=(+0.43,+0.28)`，把后右雷达放在 `(-0.43,-0.28)`；这与现场安装方位一致。
2. 仿真 URDF 虽不是实车尺寸，但也把车体写成 `box size="0.5 0.30 0.1"`，轮子位于 `x=±0.25、y=±0.17`。项目内部一直以第一维 x 表示前后方向、第二维 y 表示左右方向。
3. Nav2 footprint 解析器直接把每个点的第一个数赋给 `Point.x`、第二个数赋给 `Point.y`，不存在内部交换。
4. Git 历史显示旧值自 2026-01-27 的初始配置提交起就是 `x=±0.27、y=±0.43`，后续长期沿用；没有发现这是某个特殊坐标系下的有意定义。

## 运行影响

### 1. 局部障碍清除方向错误

ObstacleLayer 默认把 footprint 内部清为空闲。旧轮廓左右过宽、前后过短，因此：

- 车辆左右约 `0.28～0.44 m` 范围的动态回波可能被当成“车体内部”清除，其中一部分实际已经在雷达中心包络之外；这会掩盖贴近车侧的真实障碍或路沿。
- 车辆前后约 `0.28～0.43 m` 范围没有被旧轮廓覆盖，实际车体、雷达或安装件附近的回波可能保留为障碍，造成近车体伪障碍。

这两种结果并不矛盾：同一个错误矩形会同时产生横向过度清除和纵向清除不足。

### 2. MPPI 碰撞检查方向错误

旧参数中 `CostCritic`、`ObstaclesCritic` 都设置了 `consider_footprint: true`，会用局部矩形检查预测轨迹：

- 左右被按约 `0.88 m` 宽处理，在窄路、贴边或转弯时可能过早提高成本、拒绝候选轨迹，促成减速、停滞或 optimizer failure。
- 前后只按约 `0.56 m` 长处理，会低估保险杠、车轮和雷达安装范围的前后碰撞风险，允许实际车体无法安全通过的前后间隙。

现有日志只能证明旧轮次确有 MPPI 无可接受轨迹，不能把每一次 optimizer failure 唯一归因于 footprint；当时还存在全局障碍高度过滤、转向模型不一致、速度坐标约定等独立问题。

### 3. 倒车等恢复行为受影响

behavior server 订阅 `/local_costmap/published_footprint` 做碰撞预测，所以旧矩形也进入倒车和直行恢复检查。2026-11-07 的 `Collision Ahead`/backup failed 与该检查链一致，但录包没有保留足以逐格锁定当时碰撞来源的分层代价图，不能断言是长宽写反单独触发。

### 4. 旧全局规划没有使用这块写反矩形

2026-11-10 修复前，global costmap 没有 polygon footprint，而是使用 `robot_radius: 0.22 m` 的小圆。因此旧轮次的 `Starting point in lethal space` 是全局小圆和全局 costmap 的判定，不能直接归因于 local 的长宽轴互换。

不过，旧全局小圆明显小于真实车辆，可能生成局部矩形和实际车辆无法接受的路线，造成“全局能规划、局部不肯走”的模型不一致。

### 5. 不会直接旋转定位、雷达或控制方向

footprint 写反只改变代价地图清除范围与碰撞包络；它不会把 TF、雷达点、全局路径或转向命令旋转 90°。因此不能用它单独解释 11 月 9 日 106→107 的整体偏左。它可以改变近障碍处理和控制器能否接受轨迹，是偏离/停滞链条中的一个影响因素。

## 与 11 月 11 日近车体回波的关系

11 月 11 日 16:45 首次 lethal 附近反复出现点 `(-0.208,+0.306) m`：

- 对当前 padded footprint `x=±0.45、y=±0.30`，该点位于左边缘外约 `5.8 mm`，能在 0.10 m global 栅格中留下边界致命格。
- 对旧 padded footprint `x=±0.28、y=±0.44`，该点在几何上位于 footprint 内部，通常会被障碍层 footprint clearing 清除。

所以旧写反轮廓很可能掩盖过这一类左侧近车体回波。几何改正确后问题暴露出来，不代表新轮廓本身制造了雷达回波；真正还要处理的是回波来源、雷达外壳外缘、粗栅格边界清除和 global/local 清除时序。

## 当前配置判断

当前 global/local 均为：

```yaml
footprint: "[[-0.43, -0.28], [-0.43, 0.28], [0.43, 0.28], [0.43, -0.28]]"
footprint_padding: 0.02
```

轴向已经正确。按雷达中心范围理解，原始矩形是 `0.86 × 0.56 m`，padding 后是 `0.90 × 0.60 m`。它与更新后的雷达最外沿总包络 `0.90 × 0.59 m` 基本一致。

总长宽一致仍不等于四个方向都被覆盖。当前多边形以 `base_link` 对称；如果“向前延长 6 cm、向左延长 5 cm”表示相对车体仅向前、向左的单边突出，则需要分别测量 `base_link` 到前、后、左、右最外沿的距离。只有它们分别不超过 `0.45、0.45、0.30、0.30 m`，才能确认当前对称轮廓完全覆盖。

当前配置注释把 `0.86 × 0.56 m` 写成“雷达安装外沿”不准确；这是雷达**中心**跨度，更新后的雷达最外沿总包络为 `0.90 × 0.59 m`。若四个方向相对 `base_link` 的实际最外缘超出 padded 的 `±0.45/±0.30 m`，仍需改为相应的不对称轮廓。

安装目录中的 `nav2_real_params.yaml` 是源配置的符号链接；11 月 11 日室内测试也已读取到当前 global/local polygon。下一次生产启动会使用当前正确轴向配置。

## 主要证据位置

- 当前配置：`/home/akun/workspace/Car_jetson/nav2/src/nav2/config/nav2_real_params.yaml`
- 修复前配置：`/home/akun/workspace/Car_jetson/navigation_monitoring/config_changes/2026-11-10_global_planner_geometry_fix/before/nav2/src/nav2/config/nav2_real_params.yaml`
- 双雷达外参：`/home/akun/workspace/Car_jetson/real_ws/src/driver/scan_laser/Lakibeam_ROS2_Driver-main/launch/lakibeam1_scan_dual_lidar.launch.py`
- 旧运行 footprint 录包：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-07_16-25-58/navigation_raw_bag`
- 旧 global/local 失败链审计：`/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-07_16-25-58/deviation_analysis_costmap.md`
- 11 月 11 日近车体回波审计：`/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-11_first_lethal_spatial_analysis/REPORT.md`

