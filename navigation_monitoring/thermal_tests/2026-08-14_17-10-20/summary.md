# CPU Thermal Test Summary

- 状态: **completed**
- 原因: `all_phases_completed`
- 开始: 2026-08-14T17:10:20.586+08:00
- 结束: 2026-08-14T17:19:50.818+08:00
- boot_id 变化: `False`
- A/B 标签: `fan_full_navigation_sim_hardware_only`
- 室温: 未填写 C
- CPU 最高温度: 99.0 C
- 软件温度停止: **禁用**（温度线仅记录，CPU/固件保护保留）
- 节流计数增量: 69

| 阶段 | worker 数 | 样本数 | 起始 C | 峰值 C | 结束 C | 平均 C | 峰值 CPU |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 0 | 56 | 61.0 | 63.0 | 43.0 | 45.75 | 50.3 |
| one_core | 1 | 176 | 42.0 | 97.0 | 91.0 | 89.69 | 41.6 |
| settle_after_one_core | 0 | 28 | 89.0 | 89.0 | 50.0 | 56.86 | 10.7 |
| half_cores | 3 | 174 | 76.0 | 99.0 | 90.0 | 91.14 | 42.0 |
| settle_after_half_cores | 0 | 28 | 90.0 | 90.0 | 54.0 | 62.14 | 27.9 |
| physical_cores | 6 | 229 | 75.0 | 96.0 | 92.0 | 90.91 | 56.8 |
| settle_after_physical_cores | 0 | 28 | 92.0 | 92.0 | 57.0 | 65.86 | 50.6 |
| logical_cores | 8 | 343 | 79.0 | 96.0 | 90.0 | 90.66 | 86.4 |
| settle_after_logical_cores | 0 | 28 | 90.0 | 90.0 | 60.0 | 68.07 | 59.8 |

原始数据: `samples.jsonl`；表格: `samples.csv`；硬件/传感器快照: `metadata.json`。
风扇 cooling_device 状态仅表示内核请求，不等同于物理 RPM；请结合现场声音或 BIOS RPM 判断。
