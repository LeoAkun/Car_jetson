#!/usr/bin/env bash
set -uo pipefail
task_dir=/home/akun/car-host-step9.MgehLL/sensor-check-2026-10-26
echo '交换机换口后复测：只测试 eno1 上的前后单线雷达。'
echo '临时配置有线地址，观察网络和扫描数据，不修改雷达设置、Wi-Fi 或持久网络配置。'
echo '请在 sudo 提示处输入本机密码；测试完成后自动清理临时连接。'
sudo /bin/bash "$task_dir/check-switch-eno1.bash" 2>&1 | tee "$task_dir/switch-eno1-terminal.log"
result=${PIPESTATUS[0]}
printf '\n测试结束，返回码 %s。按回车关闭窗口。\n' "$result"
read -r _reply || true
exit "$result"
