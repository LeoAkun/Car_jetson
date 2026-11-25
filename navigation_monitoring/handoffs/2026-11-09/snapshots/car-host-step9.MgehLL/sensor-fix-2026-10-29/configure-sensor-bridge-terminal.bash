#!/usr/bin/env bash
set -uo pipefail
task_dir=/home/akun/car-host-step9.MgehLL/sensor-fix-2026-10-29
echo '安装可回滚的传感器网桥配置；不会改动 Wi-Fi 默认路由。'
echo '请在本窗口的 sudo 提示处输入本机密码，不要发送到聊天。'
sudo /bin/bash "$task_dir/configure-sensor-bridge.bash" 2>&1 | tee "$task_dir/network-config.log"
result=${PIPESTATUS[0]}
printf '\n配置脚本返回码 %s。按回车关闭窗口。\n' "$result"
read -r _reply || true
exit "$result"
