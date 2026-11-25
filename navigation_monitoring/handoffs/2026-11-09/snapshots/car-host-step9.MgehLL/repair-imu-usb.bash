#!/usr/bin/env bash
# Stop the observed brltty conflict and bind only the verified IMU.
# --permanent persists service masks; the default uses runtime masks.
# No serial port is opened.
set -euo pipefail

usb_device=/sys/bus/usb/devices/1-5
usb_interface="$usb_device/1-5:1.0"
imu_link=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

fail() { echo "ERROR: $*" >&2; exit 1; }

[[ -r "$usb_device/idVendor" && -r "$usb_device/idProduct" && -r "$usb_device/serial" ]] ||
  fail 'Verified IMU is missing from USB port 1-5.'
[[ "$(<"$usb_device/idVendor")" == 10c4 &&
   "$(<"$usb_device/idProduct")" == ea60 &&
   "$(<"$usb_device/serial")" == 0001 ]] ||
  fail 'USB port 1-5 no longer matches the verified IMU.'

if [[ -L "$usb_interface/driver" ]]; then
  [[ "$(basename "$(readlink -f "$usb_interface/driver")")" == cp210x ]] ||
    fail 'IMU interface has an unexpected driver; leaving it unchanged.'
fi

if [[ ${1:-} == --check && $# == 1 ]]; then
  echo 'Verified IMU: USB 1-5, 10c4:ea60, serial 0001.'
  if [[ -L "$usb_interface/driver" ]]; then
    readlink -f "$usb_interface/driver"
  else
    echo 'IMU interface currently has no kernel driver.'
  fi
  if [[ -e "$imu_link" ]]; then ls -l "$imu_link"; fi
  exit 0
fi
mask_mode=runtime
if [[ $# == 1 && ${1:-} == --permanent ]]; then
  mask_mode=permanent
elif [[ $# != 0 ]]; then
  fail 'Usage: bash repair-imu-usb.bash [--check|--permanent]'
fi
[[ $EUID == 0 ]] || fail 'Run with sudo; --check works without root.'

if [[ $mask_mode == permanent ]]; then
  echo 'Permanently masking/stopping brltty services, including udev activation.'
  systemctl mask --now brltty-udev.service brltty.service
  for unit in brltty-udev.service brltty.service; do
    [[ $(readlink "/etc/systemd/system/$unit") == /dev/null ]] ||
      fail "Persistent mask is missing for $unit."
  done
else
  echo 'Temporarily masking/stopping brltty services until reboot.'
  systemctl mask --runtime --now brltty-udev.service brltty.service
fi
if systemctl is-active --quiet brltty-udev.service ||
   systemctl is-active --quiet brltty.service; then
  fail 'A brltty service is still active; not binding the IMU.'
fi
if pgrep -x brltty >/dev/null; then
  fail 'Another brltty process is still running; not binding the IMU.'
fi
if [[ $mask_mode == permanent ]]; then
  echo 'BRLTTY_PERMANENTLY_DISABLED: both services are masked and stopped.'
fi

modprobe cp210x
if [[ ! -L "$usb_interface/driver" ]]; then
  printf '%s' '1-5:1.0' > /sys/bus/usb/drivers/cp210x/bind
fi
udevadm settle --timeout=10
echo 'Waiting 15 seconds to check that the IMU remains attached.'
sleep 15

[[ -L "$usb_interface/driver" &&
   "$(basename "$(readlink -f "$usb_interface/driver")")" == cp210x ]] ||
  fail 'IMU did not remain bound to cp210x.'
[[ -c "$imu_link" ]] || fail 'The expected persistent IMU serial path is missing.'
serial_node=$(readlink -f "$imu_link")
[[ "$(readlink -f "/sys/class/tty/${serial_node##*/}/device")" == "$(readlink -f "$usb_interface")/"* ]] ||
  fail 'The persistent IMU path points at a different USB interface.'
ls -l "$imu_link" "$serial_node"
echo 'IMU_USB_BINDING_OK: cp210x and persistent serial path are available.'
echo 'No ROS node was started and no serial port was opened.'
if [[ $mask_mode == permanent ]]; then
  echo 'Persistent service masks remain after reboot. Manual rollback:'
  echo '  sudo systemctl unmask brltty-udev.service brltty.service'
else
  echo 'Runtime masks disappear on reboot. Manual rollback:'
fi
echo '  sudo systemctl unmask --runtime brltty-udev.service brltty.service'
echo '  sudo systemctl start brltty-udev.service'
