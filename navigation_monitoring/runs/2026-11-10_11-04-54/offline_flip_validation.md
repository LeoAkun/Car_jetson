# 雷达合并 flip 离线验证

## 结论

现有录包可以确认：保留 Lakibeam 驱动的 `inverted=true`，把合并器的
`flip1`、`flip2` 从 `true` 改成 `false` 后，首败位置车前的横向障碍线消失，
同一批前雷达点恢复为车辆左侧、沿车辆前后方向延伸的边界。

本验证直接只读 rosbag2 的 SQLite/CDR 数据，没有启动 ROS 节点、传感器、
底盘、导航或 rosbag 回放。

## 方法

脚本按当前源码逐步重建两条链：

1. 从 `/laser/scan_front`、`/laser/scan_back` 读取原始 `LaserScan`。
2. 使用实车外参重现 `ros2_laser_scan_merger/src/main.cpp` 的索引和坐标变换。
3. 分别以 `flip=true` 和 `flip=false` 计算点云。
4. 按现有 `pointcloud_to_laserscan` 参数重新投影到 722 个角度格。
5. 先用录包中的 `/laser/scan_merge_laser` 校验重建程序，再比较修改后的几何。

脚本：`offline_flip_validation.py`

复现命令：

```bash
cd /home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-10_11-04-54
source /opt/ros/humble/setup.bash
python3 offline_flip_validation.py
```

结果会写入：

- `offline_flip_validation.json`
- `offline_flip_validation.png`

## 首次规划失败帧

目标时刻为 2026-11-10 11:13:07.809，选中的传感器时间戳为
11:13:07.821。

以当前 `flip1=true`、`flip2=true` 重建后：

- 录包合并扫描有 387 个有限角度格；重建也有 387 个。
- 387/387 个共同角度格误差均小于 1 cm。
- 中位绝对误差为 0.0035 mm，P95 为 0.0099 mm。

这说明离线程序准确重现了运行时的合并结果，之后的 `flip=false` 对照不是只看
原始点云作出的推测。

2.5 m 内的几何变化：

| 数据 | PCA 主轴角（模 180°） | X 方向 P05-P95 跨度 | Y 方向 P05-P95 跨度 | Y/X 跨度比 |
|---|---:|---:|---:|---:|
| 录包合并扫描 | 100.75° | 0.53 m | 2.71 m | 5.09 |
| 离线重建 `flip=true` | 100.75° | 0.53 m | 2.71 m | 5.09 |
| 离线改为 `flip=false` | 165.36° | 2.81 m | 0.85 m | 0.30 |

车辆 X 轴指向前方、Y 轴指向左侧。当前扫描主要沿 Y 方向延伸，位于车前约
1.6 m，表现为横墙；取消二次反转后主要沿 X 方向延伸，位于车辆左侧约
1.5 m，恢复为纵向路侧边界。

## 多个失败时刻重复性

另外抽取了 11:14:12、11:15:09、11:16:32 和 11:17:55，共五个失败时刻。
为避免前后雷达异步影响判断，这一组统计只比较时间戳完全对应的前雷达点。

- `flip=true` 时，Y/X 跨度比为 4.89–9.92，五帧都表现为横向结构。
- `flip=false` 时，Y/X 跨度比为 0.105–0.200，五帧都恢复为纵向结构。
- 五帧 `flip=true` 的 PCA 主轴为 80.05°–102.97°，接近横向 90°。
- 五帧 `flip=false` 的主轴为 166.71°、174.89°、9.70°、171.67°、
  178.98°；直线方向按 180° 等价，均接近车辆前后方向。

因此，横墙消失并非首帧偶然现象。修改 `flip1=false` 可以直接修复本轮横墙的
主要来源；两台驱动使用同一 `inverted=true` 语义，`flip2=false` 同时消除后
雷达链上的第二次反转。

## 限制

这是扫描几何和合并链的离线验证。它能确认错误横墙会消失，但不等同于验证
修改后的全局代价地图、规划器成功率或车辆实际轨迹；这些仍需下一次受控实车
测试确认。
