#!/usr/bin/env bash
# Test the sensor switch on the Intel port, without changing any sensor settings.
set -euo pipefail
[[ $EUID == 0 ]] || exit 1
task_dir=/home/akun/car-host-step9.MgehLL/sensor-check-2026-10-26
exec 9>"$task_dir/switch-test.lock"
flock -n 9 || { echo 'Another switch test is already running'; exit 1; }
test_dir=$(mktemp -d "$task_dir/switch-eno1-XXXXXX")
chown akun:akun "$test_dir"
echo "TEST_DIRECTORY=$test_dir"
ip -4 route show default > "$test_dir/default-route-before.txt"
ip -br address > "$test_dir/addresses-before.txt"
active_connection=
capture_pid=
factory_address_added=false
cleanup() {
  if [[ -n "$capture_pid" ]]; then
    kill -INT "$capture_pid" 2>/dev/null || true
    wait "$capture_pid" 2>/dev/null || true
  fi
  if [[ "$factory_address_added" == true ]]; then
    ip address del 192.168.198.1/24 dev eno1 2>/dev/null || true
  fi
  if [[ -n "$active_connection" ]]; then
    nmcli --wait 10 connection down id "$active_connection" || true
    nmcli connection delete id "$active_connection" || true
  fi
  ip -4 route show default > "$test_dir/default-route-after.txt"
  ip -br address > "$test_dir/addresses-after.txt"
  cmp -s "$test_dir/default-route-before.txt" "$test_dir/default-route-after.txt" && echo DEFAULT_ROUTE_UNCHANGED
  chown -R akun:akun "$test_dir"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
[[ $(cat /sys/class/net/eno1/carrier) == 1 ]] || { echo NO_LINK_ON_ENO1; exit 1; }
[[ -z $(ip -4 -o address show dev eno1) ]] || { echo 'eno1 already has IPv4; stopping'; exit 1; }
[[ -z $(ss -H -lunp 'sport = :2368 or sport = :2369') ]] || { echo 'Scan UDP ports already occupied; stopping'; exit 1; }
/usr/bin/busybox arping -D -c 2 -w 3 -I eno1 192.168.1.140 || { echo ADDRESS_CONFLICT; exit 1; }
active_connection="switch-check-eno1-$$"
nmcli connection add save no type ethernet ifname eno1 con-name "$active_connection" \
  connection.autoconnect no ipv4.method manual ipv4.addresses 192.168.1.140/24 \
  ipv4.never-default yes ipv4.ignore-auto-dns yes ipv6.method disabled
nmcli --wait 15 connection up id "$active_connection" ifname eno1
tcpdump -U -nn -i eno1 -s 0 -w "$test_dir/eno1.pcap" 'arp or icmp or udp' > "$test_dir/capture.log" 2>&1 &
capture_pid=$!
echo SWITCH_ENO1_READY
for target in 192.168.1.12 192.168.1.13 192.168.1.187 192.168.1.200; do
  echo "ARP_PROBE=$target"
  /usr/bin/busybox arping -c 2 -w 2 -I eno1 "$target" || true
done
touch "$test_dir/ready-eno1"
chown akun:akun "$test_dir/ready-eno1"
runuser -u akun -- bash -c '
  source /home/akun/car-host-step9.MgehLL/environment.bash
  export ROS_DOMAIN_ID=88 ROS_LOG_DIR="$2/ros-log"
  mkdir -p "$ROS_LOG_DIR"
  python3 "$1/check-lidar.py" eno1 scan --output-dir "$2" --duration 35
' sensor-switch-test "$task_dir" "$test_dir"
# If configured sensor addresses are silent, check the factory subnet named
# in the locally supplied Lakibeam driver documentation. No sensor writes.
if python3 - "$test_dir/eno1-scan-result.json" <<'PY'
import json,sys
data=json.load(open(sys.argv[1]))
raise SystemExit(0 if all(s['message_count']==0 for s in data.values()) else 1)
PY
then
  echo CHECK_LOCAL_DRIVER_DEFAULT_SUBNET
  if /usr/bin/busybox arping -D -c 2 -w 3 -I eno1 192.168.198.1; then
    ip address add 192.168.198.1/24 dev eno1
    factory_address_added=true
    /usr/bin/busybox arping -c 3 -w 4 -I eno1 192.168.198.2 || true
    ping -n -I 192.168.198.1 -c 2 -W 1 192.168.198.2 || true
  fi
fi
ip -s link show dev eno1 > "$test_dir/link-statistics.txt"
ip neigh show dev eno1 > "$test_dir/neighbors.txt"
echo SWITCH_TEST_COMPLETED
