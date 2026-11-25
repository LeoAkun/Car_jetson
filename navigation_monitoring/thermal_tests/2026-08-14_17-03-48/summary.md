# CPU Thermal Test Summary

- 状态: **completed**
- 原因: `all_phases_completed`
- 开始: 2026-08-14T17:03:48.191+08:00
- 结束: 2026-08-14T17:04:18.281+08:00
- boot_id 变化: `False`
- A/B 标签: `pre_sim_recheck`
- 室温: 未填写 C
- CPU 最高温度: 59.0 C
- 保护上限: 78.0 C（警告 70.0 C）
- 节流计数增量: 0

| 阶段 | worker 数 | 样本数 | 起始 C | 峰值 C | 结束 C | 平均 C | 峰值 CPU |
|---|---:|---:|---:|---:|---:|---:|---:|
| monitor_only | 0 | 56 | 59.0 | 59.0 | 41.0 | 41.82 | 10.6 |

原始数据: `samples.jsonl`；表格: `samples.csv`；硬件/传感器快照: `metadata.json`。
风扇 cooling_device 状态仅表示内核请求，不等同于物理 RPM；请结合现场声音或 BIOS RPM 判断。
