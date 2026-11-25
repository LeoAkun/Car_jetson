# CPU Thermal Test Summary

- 状态: **aborted**
- 原因: `warning_limit_one_core`
- 开始: 2026-08-14T17:05:35.726+08:00
- 结束: 2026-08-14T17:08:06.391+08:00
- boot_id 变化: `False`
- A/B 标签: `fan_full_navigation_sim`
- 室温: 未填写 C
- CPU 最高温度: 72.0 C
- 保护上限: 78.0 C（警告 70.0 C）
- 节流计数增量: 0

| 阶段 | worker 数 | 样本数 | 起始 C | 峰值 C | 结束 C | 平均 C | 峰值 CPU |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 0 | 56 | 59.0 | 59.0 | 41.0 | 42.3 | 11.0 |
| one_core | 1 | 2 | 41.0 | 72.0 | 72.0 | 56.5 | 12.4 |
| emergency_cooldown | 0 | 223 | 72.0 | 72.0 | 41.0 | 41.5 | 21.9 |

原始数据: `samples.jsonl`；表格: `samples.csv`；硬件/传感器快照: `metadata.json`。
风扇 cooling_device 状态仅表示内核请求，不等同于物理 RPM；请结合现场声音或 BIOS RPM 判断。
