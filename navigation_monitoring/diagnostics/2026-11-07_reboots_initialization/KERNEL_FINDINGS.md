# Root 导出的内核及系统日志补充结论

仅分析已导出的文件；未执行新的 sudo、启动 ROS 或修改系统。时间为北京时间。

## 范围与持久化缺口

已取得 `system_logs/` 中四个 boot 的完整已持久化内核 journal，以及各 boot 最后 1500 行系统 journal。stderr 均为空，采集完成标志已存在。

| Boot | 已保存内核末行 | 已保存系统末行 | 独立 CSV 最后记录 | CSV 晚于内核 / 系统 |
|---|---|---|---|---|
| `271a95a2`，第一次异常前 | 16:14:41.023347 | 16:14:41.023347 | 16:15:06.973 | 25.950 / 25.950 秒 |
| `7a56ea4e`，第二次异常前 | 16:17:55.835807 | 16:17:59.994912 | 16:18:28.859 | 33.023 / 28.864 秒 |
| `ca32f49f`，后续人工关机前 | 16:45:30.700076 | 16:45:40.721223 | 16:46:05.549 | 34.849 / 24.828 秒 |

第二次导航监控还在 16:18:29.038 保存了 LIO-SAM 进程出现事件，略晚于上述资源 CSV。所有 journal 末尾都早于独立文件，故无法据其末行确定复位瞬间；最终未落盘时间段发生的故障无法从这些 journal 还原。

## 未发现已保存的致命故障记录

两次异常 boot 的内核与系统末尾中均未发现：

- Kernel panic、Oops、BUG / Call Trace；
- OOM killer、Out of memory 或进程被 OOM 杀死；
- MCE / Hardware Error；
- soft/hard lockup、watchdog timeout；
- 临界温度关机、磁盘 I/O / 文件系统错误；
- AER Fatal / Uncorrectable / Non-Fatal 或 DPC containment；
- systemd-shutdown、正常 Power-Off / Reboot target 或实际电源按键关机请求。

启动时的 `NMI watchdog: Enabled`、`Power Button` 输入设备注册、`Started Unattended Upgrades Shutdown` 服务启动都不是故障或关机动作。第二 boot 的 `geoclue ... Service not used for 60 seconds. Shutting down..` 只是 geoclue 自己退出，不能误读为系统关机。启动时未连接的 SATA 端口 `link down` 也没有运行中 I/O 错误的证据。

上述结论仅为“已持久化记录未发现”，不能排除完全未落盘的硬件/内核故障。

## PCIe 的已知异常

四个 boot 都出现 `pcieport 0000:00:1c.2` 的 **AER Correctable / Physical Layer / Receiver ID / RxErr**。按每条 `AER: ... Correctable error message received` 计数（不是按四行展开日志计数）：

| Boot | 消息数 | 首条 | 末条 |
|---|---:|---|---|
| `271a95a2` | 190 | 12:48:43.046429 | 16:14:41.022506 |
| `7a56ea4e` | 7 | 16:16:11.257791 | 16:17:55.834832 |
| `ca32f49f` | 895 | 16:19:33.403205 | 16:45:30.699129 |
| `ca0a8345`，采集当时当前 boot | 83 | 17:03:04.159701 | 17:34:41.256643 |

单条消息可能包含 `Multiple`，因此此表是**日志消息数**，不是硬件逐错误计数。

启动枚举和当前 PCIe 拓扑共同确认：该 root port 下是 `0000:02:00.0` Realtek RTL8168h/8111h 网卡，Linux 名称 `enp2s0`。这是此前已发现的网口 PCIe 链路问题，值得隔离检查。但当前没有致命或不可纠正升级、网卡 watchdog hang、链路恢复失败，且第一次从开机不久就报错、之后仍运行数小时；不能直接将它定为整机重启原因。

日志中网卡驱动启动时还记录 `can't disable ASPM; OS doesn't have ASPM control`。这是固件/操作系统控制权限的线索，不能仅凭此认定 ASPM 导致复位；本轮未改 ASPM 或 BIOS。

## 崩溃留存

root 导出的 `/sys/fs/pstore` 和 `/var/lib/systemd/pstore` 列表均为空，pstore tar 内只有 `./` 空目录。`systemd-pstore` 在对应开机中因条件不满足跳过；没有可供读取的持久化 panic 转储。

## 硬件身份纠正

本轮完整内核明确 CPU 为 **12th Gen Intel Core i7-12700T**，BIOS `7151I101 03/19/2025`；当前 `lscpu` 为 20 个逻辑 CPU。仓库 10 月温度与复位报告写的是 **i7-8700**。两份材料不能直接视为同一硬件上的对照测试，因此不把旧主机的满载稳定数据用于排除本轮硬件问题。

机器可读证据及匹配结果在 `kernel_evidence.json`。16:58:52 的再次系统启动按用户最新纠正归为人工关机后开机，不计为异常重启。
