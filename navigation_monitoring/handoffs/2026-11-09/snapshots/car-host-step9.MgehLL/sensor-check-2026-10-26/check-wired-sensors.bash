#!/usr/bin/env bash
# Bounded sensor-only check using NetworkManager profiles kept in memory.
set -euo pipefail
[[ $EUID == 0 ]] || exit 1
task_dir=/home/akun/car-host-step9.MgehLL/sensor-check-2026-10-26
test_dir=$(mktemp -d "$task_dir/wired-XXXXXX")
chown akun:akun "$test_dir"
echo "TEST_DIRECTORY=$test_dir"
ip -4 route show default > "$test_dir/default-route-before.txt"
journalctl -k --since '-15 min' --no-pager | /usr/bin/grep -E 'enp2s0|r8169|AER:|RxErr|NETDEV WATCHDOG' | tail -100 > "$test_dir/kernel-network-before.txt" || true
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
  if cmp -s "$test_dir/default-route-before.txt" "$test_dir/default-route-after.txt"; then
    echo DEFAULT_ROUTE_UNCHANGED
  else
    echo DEFAULT_ROUTE_CHANGED >&2
  fi
  chown -R akun:akun "$test_dir"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
for interface in eno1 enp2s0; do
  [[ -z $(ip -4 -o address show dev "$interface") ]] || { echo "$interface already has an IPv4 address; stopping"; exit 1; }
  echo "TEST_INTERFACE=$interface"
  /usr/bin/busybox arping -D -c 2 -w 3 -I "$interface" 192.168.1.140 || { echo ADDRESS_CONFLICT; exit 1; }
  active_connection="sensor-check-$interface-$$"
  nmcli connection add save no type ethernet ifname "$interface" \
    con-name "$active_connection" \
    connection.autoconnect no ipv4.method manual ipv4.addresses 192.168.1.140/24 \
    ipv4.never-default yes ipv4.ignore-auto-dns yes ipv6.method disabled
  nmcli --wait 15 connection up id "$active_connection" ifname "$interface"
  ip -br address show dev "$interface" | tee "$test_dir/$interface-address.txt"
  for property in carrier speed duplex; do
    printf '%s=' "$property"
    cat "/sys/class/net/$interface/$property"
  done > "$test_dir/$interface-link.txt"
  ip -s link show dev "$interface" > "$test_dir/$interface-statistics-before.txt"
  tcpdump -U -nn -i "$interface" -s 0 -w "$test_dir/$interface.pcap" 'arp or icmp or udp' > "$test_dir/$interface-capture.log" 2>&1 &
  capture_pid=$!
  for host in 192.168.1.12 192.168.1.13 192.168.1.187 192.168.1.200; do
    ping -n -I 192.168.1.140 -c 1 -W 1 "$host" || true
  done
  touch "$test_dir/ready-$interface"
  chown akun:akun "$test_dir/ready-$interface"
  if [[ "$interface" == eno1 ]]; then kind=rslidar; else kind=scan; fi
  runuser -u akun -- bash -c '
    source /home/akun/car-host-step9.MgehLL/environment.bash
    export ROS_DOMAIN_ID=88 ROS_LOG_DIR="$4/ros-log"
    mkdir -p "$ROS_LOG_DIR"
    python3 "$1/check-lidar.py" "$2" "$3" --output-dir "$4"
  ' sensor-test "$task_dir" "$interface" "$kind" "$test_dir"
  ip -4 -o address show dev "$interface" | tee "$test_dir/$interface-address-end.txt"
  ip neigh show dev "$interface" | tee "$test_dir/$interface-neighbors.txt"
  ip -s link show dev "$interface" > "$test_dir/$interface-statistics-after.txt"
  cleanup_stage
done
journalctl -k --since '-5 min' --no-pager | /usr/bin/grep -E 'enp2s0|r8169|AER:|RxErr|NETDEV WATCHDOG' | tail -100 > "$test_dir/kernel-network-after.txt" || true
echo WIRED_SENSOR_TESTS_COMPLETED
