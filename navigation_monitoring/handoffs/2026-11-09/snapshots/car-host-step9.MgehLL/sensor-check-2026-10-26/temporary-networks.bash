#!/usr/bin/env bash
# Sequential, bounded tests: preserve Wi-Fi/default route, remove only our address.
set -euo pipefail
[[ $EUID == 0 ]] || exit 1
task_dir=/home/akun/car-host-step9.MgehLL/sensor-check-2026-10-26
active_interface=
cleanup() {
  if [[ -n "$active_interface" ]]; then
    ip address del 192.168.1.140/24 dev "$active_interface" || true
    rm -f "$task_dir/ready-$active_interface"
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
ip -4 route show default > "$task_dir/default-route-before.txt"
for interface in eno1 enp2s0; do
  printf '\nTEMPORARY_NETWORK_STAGE %s\n' "$interface"
  [[ -z $(ip -4 -o address show dev "$interface") ]] || {
    echo "IPv4 already configured on $interface; leaving it unchanged."; exit 1;
  }
  /usr/bin/busybox arping -D -c 2 -w 3 -I "$interface" 192.168.1.140 || {
    echo 'Address conflict detected; leaving the interface unchanged.'; exit 1;
  }
  ip address add 192.168.1.140/24 dev "$interface"
  active_interface=$interface
  ip -brief address show dev "$interface"
  rm -f "$task_dir/done-$interface"
  touch "$task_dir/ready-$interface"
  timeout --signal=INT --kill-after=2 8 tcpdump -l -nn -e -i "$interface" -s 128 -c 80 2>&1 || true
  for host in 192.168.1.12 192.168.1.13 192.168.1.187 192.168.1.200; do
    ping -n -I "$interface" -c 1 -W 1 "$host" || true
  done
  ip neigh show dev "$interface"
  # Leave a bounded window for the unprivileged single-sensor tests.
  for ((attempt=0; attempt<80; attempt++)); do
    [[ -e "$task_dir/done-$interface" ]] && break
    sleep 1
  done
  cleanup
  active_interface=
done
ip -4 route show default > "$task_dir/default-route-after.txt"
cmp "$task_dir/default-route-before.txt" "$task_dir/default-route-after.txt"
echo TEMPORARY_NETWORKS_REMOVED_DEFAULT_ROUTE_UNCHANGED
