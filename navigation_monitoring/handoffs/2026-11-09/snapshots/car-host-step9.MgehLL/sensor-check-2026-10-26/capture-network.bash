#!/usr/bin/env bash
# Passive sensor-network inventory only. No IP address, route, or sensor setting changes.
set -u
[[ $EUID == 0 ]] || exit 1
for interface in enp2s0 eno1; do
  printf '\nINTERFACE %s\n' "$interface"
  ip -brief address show dev "$interface"
  timeout --signal=INT --kill-after=2 12 tcpdump -l -nn -e -i "$interface" -s 128 -c 80 '(arp or ip)' 2>&1
  result=$?
  [[ $result == 0 || $result == 124 ]] || exit "$result"
done
printf '\nRECENT_NETWORK_KERNEL_MESSAGES\n'
journalctl -k -b --since '10 minutes ago' --no-pager |
  /usr/bin/python3 -c 'import sys; print("".join(line for line in sys.stdin if any(k in line for k in ["r8169", "enp2s0", "eno1", "cp210", "brltty", "usb 1-5"])), end="")'
