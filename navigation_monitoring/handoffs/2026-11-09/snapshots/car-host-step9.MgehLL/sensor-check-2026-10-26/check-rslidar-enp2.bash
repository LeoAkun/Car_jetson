#!/usr/bin/env bash
set -euo pipefail
[[ $EUID == 0 ]] || exit 1
task_dir=/home/akun/car-host-step9.MgehLL/sensor-check-2026-10-26
exec 9>"$task_dir/switch-test.lock"
flock -n 9 || { echo 'Another sensor network test is running'; exit 1; }
test_dir=$(mktemp -d "$task_dir/rslidar-enp2-XXXXXX")
chown akun:akun "$test_dir"
echo "TEST_DIRECTORY=$test_dir"
ip -4 route show default > "$test_dir/default-route-before.txt"
ip -br address > "$test_dir/addresses-before.txt"
active_connection=
capture_pid=
cleanup_stage() {
  if [[ -n "$capture_pid" ]]; then
    kill -INT "$capture_pid" 2>/dev/null || true
    wait "$capture_pid" 2>/dev/null || true
    capture_pid=
  fi
  if [[ -n "$active_connection" ]]; then
    nmcli --wait 10 connection down id "$active_connection" || true
    nmcli connection delete id "$active_connection" || true
    active_connection=
  fi
}
cleanup() {
  cleanup_stage
  ip -4 route show default > "$test_dir/default-route-after.txt"
  ip -br address > "$test_dir/addresses-after.txt"
  cmp -s "$test_dir/default-route-before.txt" "$test_dir/default-route-after.txt" && echo DEFAULT_ROUTE_UNCHANGED
  chown -R akun:akun "$test_dir"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
[[ -z $(ss -H -lunp 'sport = :6699 or sport = :7788') ]] || { echo 'RoboSense ports already occupied'; exit 1; }
# Interface names cannot be inferred from the physical socket label.
timeout --signal=INT --kill-after=2 6 tcpdump -l -nn -e -i any -c 60 \
  'arp or udp dst port 6699 or udp dst port 7788' > "$test_dir/passive-network.txt" 2>&1 || true
for interface in enp2s0 enp4s0; do
  [[ $(cat "/sys/class/net/$interface/carrier") == 1 ]] || continue
  [[ -z $(ip -4 -o address show dev "$interface") ]] || { echo "$interface has IPv4; leaving unchanged"; continue; }
  [[ -z $(ip -4 -o address show | /usr/bin/grep -F '192.168.1.140/') ]] || { echo 'Sensor host address is already configured elsewhere'; exit 1; }
  echo "TEST_INTERFACE=$interface"
  cat /sys/class/net/enp2s0/device/../aer_dev_correctable > "$test_dir/$interface-aer-before.txt"
  /usr/bin/busybox arping -D -c 2 -w 3 -I "$interface" 192.168.1.140 || { echo ADDRESS_CONFLICT; exit 1; }
  active_connection="rslidar-check-$interface-$$"
  nmcli connection add save no type ethernet ifname "$interface" con-name "$active_connection" \
    connection.autoconnect no ipv4.method manual ipv4.addresses 192.168.1.140/24 \
    ipv4.never-default yes ipv4.ignore-auto-dns yes ipv6.method disabled
  nmcli --wait 15 connection up id "$active_connection" ifname "$interface"
  for property in carrier speed duplex; do
    printf '%s=' "$property"
    cat "/sys/class/net/$interface/$property"
  done > "$test_dir/$interface-link.txt"
  ip -s link show dev "$interface" > "$test_dir/$interface-statistics-before.txt"
  tcpdump -U -nn -i "$interface" -s 0 -w "$test_dir/$interface.pcap" 'arp or icmp or udp' > "$test_dir/$interface-capture.log" 2>&1 &
  capture_pid=$!
  /usr/bin/busybox arping -c 3 -w 3 -I "$interface" 192.168.1.200 || true
  ping -n -I 192.168.1.140 -c 2 -W 1 192.168.1.200 || true
  touch "$test_dir/ready-$interface"
  chown akun:akun "$test_dir/ready-$interface"
  runuser -u akun -- bash -c '
    source /home/akun/car-host-step9.MgehLL/environment.bash
    export ROS_DOMAIN_ID=88 ROS_LOG_DIR="$3/ros-log"
    mkdir -p "$ROS_LOG_DIR"
    python3 "$1/check-lidar.py" "$2" rslidar --output-dir "$3" --duration 35
  ' rslidar-test "$task_dir" "$interface" "$test_dir"
  ip neigh show dev "$interface" > "$test_dir/$interface-neighbors.txt"
  ip -s link show dev "$interface" > "$test_dir/$interface-statistics-after.txt"
  cat /sys/class/net/enp2s0/device/../aer_dev_correctable > "$test_dir/$interface-aer-after.txt"
  cleanup_stage
  # Only check the newly linked spare port if enp2s0 did not produce points.
  if python3 - "$test_dir/$interface-rslidar-result.json" <<'PY'
import json,sys
result=json.load(open(sys.argv[1]))
raise SystemExit(0 if result['/rslidar_points']['message_count']>0 else 1)
PY
  then break; fi
done
journalctl -k --since '-5 min' --no-pager | /usr/bin/grep -E 'enp2s0|r8169|NETDEV WATCHDOG|AER:|RxErr' | tail -80 > "$test_dir/kernel-network.txt" || true
echo RSLIDAR_PORT_TEST_COMPLETED
