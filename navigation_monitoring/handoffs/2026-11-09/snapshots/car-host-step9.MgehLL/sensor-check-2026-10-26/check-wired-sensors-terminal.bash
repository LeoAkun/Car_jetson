#!/usr/bin/env bash
set -uo pipefail
task_dir=/home/akun/car-host-step9.MgehLL/sensor-check-2026-10-26
echo '仅临时测试 eno1 上的速腾雷达和 enp2s0 上的前后单线雷达。'
echo '使用不保存到磁盘的静态网络连接，不修改已有网络配置、Wi-Fi、默认路由或传感器设置。'
echo '请在 sudo 提示处输入本机密码；结束后自动删除测试连接。'
sudo /bin/bash "$task_dir/check-wired-sensors.bash" 2>&1 | tee "$task_dir/wired-sensors-terminal.log"
result=${PIPESTATUS[0]}
printf '\n检查完成，返回码 %s。按回车关闭窗口。\n' "$result"
read -r _reply || true
exit "$result"
