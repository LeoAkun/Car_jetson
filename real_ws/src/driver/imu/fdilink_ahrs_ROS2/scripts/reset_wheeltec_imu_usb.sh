#!/usr/bin/env bash
set -euo pipefail

# The IMU uses a CP2102 with this immutable USB serial number. Toggling
# authorization forces a fresh USB enumeration after the host has rebooted.
# USB hubs can enumerate after systemd starts, so wait up to 20 seconds.
for ((attempt = 1; attempt <= 20; ++attempt)); do
  for device in /sys/bus/usb/devices/*; do
    vendor_file="$device/idVendor"
    product_file="$device/idProduct"
    serial_file="$device/serial"
    authorized_file="$device/authorized"

    if [[ ! -r "$vendor_file" || ! -r "$product_file" || ! -r "$serial_file" || ! -w "$authorized_file" ]]; then
      continue
    fi

    if [[ "$(<"$vendor_file")" == "10c4" && "$(<"$product_file")" == "ea60" && "$(<"$serial_file")" == "0001" ]]; then
      echo 0 > "$authorized_file"
      sleep 1
      echo 1 > "$authorized_file"
      exit 0
    fi
  done
  sleep 1
done

echo "Wheeltec IMU CP2102 (10c4:ea60, serial 0001) was not found" >&2
exit 1
