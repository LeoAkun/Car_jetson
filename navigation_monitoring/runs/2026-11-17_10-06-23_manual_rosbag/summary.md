# Rosbag 录制摘要

- 开始：2026-11-17 10:06:23（Asia/Shanghai）
- 结束：2026-11-17 10:07:32（Asia/Shanghai）
- 结束原因：操作者要求关闭录包
- 模式：只录包、observation-only
- 车辆控制命令：未发送
- 正常退出耗时：220.874 ms
- SQLite 分片：1 个，24,576 字节
- `metadata.yaml`：已生成
- SQLite `PRAGMA quick_check`：`ok`
- 消息：`/rosout` 6 条；其余目标话题 0 条
- 结论：录包器正常启动和正常落盘，但录制期间传感器、LIO 和 Nav2 未发布目标话题，因此没有车辆测试数据。
