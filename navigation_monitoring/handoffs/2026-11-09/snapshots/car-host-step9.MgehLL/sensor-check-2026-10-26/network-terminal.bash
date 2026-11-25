#!/usr/bin/env bash
set -uo pipefail
echo '只读检查传感器网口：分别抓取 enp2s0 和 eno1 的数据，不修改 IP 或传感器配置。'
echo '请在 sudo 提示处输入本机密码。'
sudo /bin/bash /home/akun/car-host-step9.MgehLL/sensor-check-2026-10-26/capture-network.bash 2>&1 |
  tee /home/akun/car-host-step9.MgehLL/sensor-check-2026-10-26/network-capture.log
result=${PIPESTATUS[0]}
printf '\n检查完成，返回码 %s。按回车关闭窗口。\n' "$result"
read -r _reply || true
exit "$result"
