#!/usr/bin/env bash
set -Eeuo pipefail

readonly old_switch_uuid=ae23001a-b9a8-3c6f-81d5-bad6ec5e75bd
readonly old_rs_uuid=a93c5323-f084-302f-910a-1221fcdcc5b2
readonly bridge_name=car-sensor-bridge
readonly switch_port_name=car-sensor-eno1
readonly rs_port_name=car-sensor-enp4s0
readonly bridge_mac=10:0c:29:57:98:74

nm() { /usr/bin/nmcli "$@"; }

rollback() {
  set +e
  nm connection down "$bridge_name" >/dev/null 2>&1
  nm connection delete "$switch_port_name" >/dev/null 2>&1
  nm connection delete "$rs_port_name" >/dev/null 2>&1
  nm connection delete "$bridge_name" >/dev/null 2>&1
  nm connection modify uuid "$old_switch_uuid" connection.autoconnect yes
  nm connection modify uuid "$old_rs_uuid" connection.autoconnect yes
  nm -w 3 connection up uuid "$old_switch_uuid" >/dev/null 2>&1
  nm -w 3 connection up uuid "$old_rs_uuid" >/dev/null 2>&1
}

if [[ ${1:-} == --rollback ]]; then
  rollback
  echo SENSOR_BRIDGE_ROLLED_BACK
  exit 0
fi

[[ ${EUID} -eq 0 ]] || { echo 'Run this script with sudo.' >&2; exit 2; }
[[ -e /sys/class/net/eno1/carrier && $(</sys/class/net/eno1/carrier) == 1 ]] || {
  echo 'eno1 has no carrier; preserving the current network configuration.' >&2; exit 3;
}
[[ -e /sys/class/net/enp4s0/carrier && $(</sys/class/net/enp4s0/carrier) == 1 ]] || {
  echo 'enp4s0 has no carrier; preserving the current network configuration.' >&2; exit 3;
}
[[ $(nm -g connection.interface-name connection show uuid "$old_switch_uuid") == eno1 ]]
[[ $(nm -g connection.interface-name connection show uuid "$old_rs_uuid") == enp4s0 ]]

/usr/bin/python3 /home/akun/car-host-step9.MgehLL/sensor-fix-2026-10-29/observe-network.py

if nm connection show "$bridge_name" >/dev/null 2>&1; then
  [[ $(nm -g ipv4.addresses connection show "$bridge_name") == 192.168.1.140/24 ]]
  [[ $(nm -g ipv4.never-default connection show "$bridge_name") == yes ]]
  echo SENSOR_BRIDGE_ALREADY_CONFIGURED
  exit 0
fi

trap 'status=$?; if ((status != 0)); then echo "Configuration failed ($status); restoring DHCP profiles." >&2; rollback; fi' EXIT

nm connection modify uuid "$old_switch_uuid" connection.autoconnect no
nm connection modify uuid "$old_rs_uuid" connection.autoconnect no

nm connection add type bridge ifname br-sensors con-name "$bridge_name" \
  connection.autoconnect yes connection.autoconnect-priority 100 \
  connection.autoconnect-slaves 1 \
  bridge.stp yes bridge.forward-delay 4 bridge.hello-time 1 bridge.max-age 6 \
  bridge.mac-address "$bridge_mac" \
  ipv4.method manual ipv4.addresses 192.168.1.140/24 \
  ipv4.never-default yes ipv4.ignore-auto-routes yes ipv4.ignore-auto-dns yes \
  ipv6.method disabled
nm connection add type ethernet ifname eno1 con-name "$switch_port_name" \
  connection.autoconnect yes connection.autoconnect-priority 100 \
  master "$bridge_name" slave-type bridge
nm connection add type ethernet ifname enp4s0 con-name "$rs_port_name" \
  connection.autoconnect yes connection.autoconnect-priority 100 \
  master "$bridge_name" slave-type bridge

nm -w 15 connection up "$bridge_name"
nm -w 15 connection up "$switch_port_name"
nm -w 15 connection up "$rs_port_name"

[[ $(nm -g ipv4.addresses connection show "$bridge_name") == 192.168.1.140/24 ]]
[[ $(nm -g ipv4.never-default connection show "$bridge_name") == yes ]]
ip -4 -o address show dev br-sensors | /usr/bin/grep -F '192.168.1.140/24'
ip route show default | /usr/bin/grep -F 'dev wlx90de800ed399'
echo SENSOR_BRIDGE_CONFIGURED
trap - EXIT
