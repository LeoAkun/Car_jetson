# 11月2日导航测试：原始里程计记录查找结果

查找日期：2026-11-06。目标是3b2mus地图113→106→107任务，车辆日志北京时间2026-11-02 17:36～17:54（UTC+8）。只读检查历史数据；另保存证据副本和本报告，没有启动ROS节点、录包、回放或发送车辆控制命令。

**当前可访问文件中未找到该时段完整的原始 `/odom` 消息，但新找到479个LIO-SAM关键帧的位姿记录。**

## 找到的额外记录

原始位置：

- `/home/akun/workspace/Car_jetson/slam/src/pcd_global/test/trajectory.pcd`
- `/home/akun/workspace/Car_jetson/slam/src/pcd_global/test/transformations.pcd`

两份文件各479点，写入时间为北京时间11月2日17:52:44.761和17:52:44.766，与本次LIO退出时刻吻合，轨迹末端也与本次观测相符。这是LIO-SAM关闭时保存的优化关键帧轨迹。

`trajectory.pcd` 只有x/y/z和索引；`transformations.pcd` 有x/y/z、索引、roll/pitch/yaw和time。

**它们没有原始线速度/角速度twist、协方差、header.frame_id和child_frame_id，不能代替完整Odometry。** 关键帧位姿也不等于当时发布的每一条原始里程计消息。

此外，ASCII格式把时间写成 `1.7936122e+09` 等，已经丢失精度：479条记录只剩10个不同时间值，非零时间步长均为100秒。保存值范围为17:36:40～17:51:40，不能把这个量化后的末值当作实际记录终点，也不能据此精确对齐速度命令或计算逐帧真实速度。

已将两份文件原样复制到本次run的 `odometry_evidence/`，保留修改时间并验证SHA-256一致，避免下次LIO保存覆盖原路径。副本：

- [trajectory.pcd](odometry_evidence/trajectory.pcd)
- [transformations.pcd](odometry_evidence/transformations.pcd)

## 现有监控为什么不能补出完整消息

- `pose_observer.jsonl`：1012条，约每2秒一次，仅查询 `map→base_link` TF并保留xyz/yaw/stamp。有效更新到北京时间17:52:44.735，此后重复旧TF；没有原始 `/odom` 的姿态四元数、twist、坐标系和协方差消息体。
- `monitor.raw.jsonl`：订阅里程计后只统计到达频率和新鲜度，没有保存原始消息。相关代码为 `navigation_monitoring/nav_task_monitor.py` 的 `on_odom()`。
- 当时LIO日志只记录启动和退出等状态，没有完整里程计消息输出。
- 已检查的当时会话命令没有发现对本次导航启动rosbag录制或将 `/odom` 完整导出的记录。

## 录包检索证据及范围

检查了 `/home/akun`、`/tmp`、`/media`、`/mnt` 中可访问的录包文件，包含隐藏和被git忽略的文件。共找到96份ROS2录包元数据和96个SQLite录包数据库；元数据均成功解析，数据库均以只读方式成功打开。

没有任何录包的元数据时间范围覆盖本次测试窗口。直接查询这96个数据库的消息时间戳，也没有发现测试窗口内的消息。虽然94个数据库有某种里程计相关话题，它们属于其他日期；11月4～6日的 `/charging_dock/local_odom` 是后续充电对接测试数据，不能用来代表11月2日导航时的LIO `/odom`。

本机可见旧ROS录包归档修改时间为10月15日，早于本次测试；未发现另外的11月2日录包归档。

部分systemd私有临时目录和 `/media/admin1` 无读取权限，未检查其内容。当前没有挂载其他数据盘。本结论限于已访问的本机文件，不能据此证明其他主机、未挂载硬盘或外部备份中也不存在。

## 对后续分析的影响

479个关键帧可用于补充检查局部轨迹形状和姿态；仍不足以重建当时完整 `/odom.twist`，或精确量化速度坐标系错误在失败瞬间的影响。仅凭当前这些摘要和关键帧文件，不能恢复已经未保存的消息字段和精确时间。

若要验证当前链路的速度坐标系，需要新的同步原始消息记录；它只能验证当前问题，不能冒充本次历史记录。

机器可读结果及副本SHA-256见 `result_metrics.json` 中的 `odometry_record_search`。
