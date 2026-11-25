#!/usr/bin/env bash
set -u
umask 077
out=/home/akun/workspace/Car_jetson/navigation_monitoring/diagnostics/2026-11-07_reboots_initialization/system_logs
mkdir -p "$out"
printf '%s\n' '只读采集系统/内核日志，不启动ROS，不控制车辆，不修改系统配置。' '需要本机sudo密码以读取系统日志；请勿在聊天中发送密码。'
if ! sudo -v; then
  printf '%s\n' '未获得日志读取权限，已退出。'
  exit 1
fi
for bootid in 271a95a206a6472889559195feec0b76 7a56ea4ec01f4c9ebe994775707dd7f3 ca32f49f62a34e77aee9af55c9ac556a ca0a8345ba374d71b8b733c458980bd0; do
  sudo -n journalctl -b "$bootid" -k --no-pager -o short-iso-precise > "$out/$bootid.kernel.log" 2> "$out/$bootid.kernel.stderr"
  sudo -n journalctl -b "$bootid" -n 1500 --no-pager -o short-iso-precise > "$out/$bootid.system_tail.log" 2> "$out/$bootid.system_tail.stderr"
done
sudo -n journalctl --list-boots --no-pager > "$out/system_boot_list.txt" 2>&1
sudo -n ls -la /sys/fs/pstore /var/lib/systemd/pstore > "$out/pstore_listing.txt" 2>&1
sudo -n tar -C /sys/fs/pstore -cf - . > "$out/pstore_sys.tar" 2> "$out/pstore_sys.stderr"
sudo -n journalctl -u systemd-pstore --no-pager > "$out/pstore_service.log" 2>&1
sudo -n dmesg -T > "$out/current_dmesg.txt" 2>&1
printf '%s\n' 'collection_finished' > "$out/COLLECTION_COMPLETE"
printf '%s\n' '采集结束，可以关闭此窗口。'
read -r -p '按回车关闭窗口。' _reply
