#!/usr/bin/env bash
set -uo pipefail
echo '逐个有线口临时配置 192.168.1.140/24 并检查雷达数据，结束后自动移除。'
echo '不改 Wi-Fi、默认路由、持久网络配置或传感器设置。请在 sudo 提示中输入密码。'
sudo /bin/bash /home/akun/car-host-step9.MgehLL/sensor-check-2026-10-26/temporary-networks.bash 2>&1 |
  tee /home/akun/car-host-step9.MgehLL/sensor-check-2026-10-26/temporary-networks.log
result=${PIPESTATUS[0]}
printf '\n测试结束，返回码 %s。按回车关闭窗口。\n' "$result"
read -r _reply || true
exit "$result"
