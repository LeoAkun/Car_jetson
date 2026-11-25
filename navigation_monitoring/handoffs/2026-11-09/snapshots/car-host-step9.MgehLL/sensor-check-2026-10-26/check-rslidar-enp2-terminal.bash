#!/usr/bin/env bash
set -uo pipefail
task_dir=/home/akun/car-host-step9.MgehLL/sensor-check-2026-10-26
echo '检查速腾雷达在 enp2s0 的网络响应与点云；若无数据，再核对新增链路的 enp4s0。'
echo '只使用临时静态地址；不改 Wi-Fi、已有网络配置或雷达设置，不启动底盘。'
echo '请在 sudo 提示处输入本机密码；测试结束自动清理临时连接。'
sudo /bin/bash "$task_dir/check-rslidar-enp2.bash" 2>&1 | tee "$task_dir/rslidar-enp2-terminal.log"
result=${PIPESTATUS[0]}
printf '\n检查完成，返回码 %s。按回车关闭窗口。\n' "$result"
read -r _reply || true
exit "$result"
