#!/usr/bin/env bash
set -Eeuo pipefail

# Install a restricted NTP server on the vehicle IPC (.140).
# This script deliberately does not enable routing, forwarding, or NAT.

readonly SENSOR_INTERFACE="${SENSOR_INTERFACE:-br-sensors}"
readonly SERVER_ADDRESS="${SERVER_ADDRESS:-192.168.1.140}"
readonly CLIENT_PRIMARY="${CLIENT_PRIMARY:-192.168.1.150}"
readonly CLIENT_SECONDARY="${CLIENT_SECONDARY:-192.168.1.160}"
readonly CHRONY_DROPIN="/etc/chrony/conf.d/90-traffic-sensor-time.conf"

if [[ ${EUID} -ne 0 ]]; then
  echo "ERROR: run this script as root (sudo $0)" >&2
  exit 2
fi

if [[ "$(sysctl -n net.ipv4.ip_forward)" != "0" ]]; then
  echo "ERROR: net.ipv4.ip_forward must remain 0 for this deployment" >&2
  exit 3
fi

if ! ip -4 -o address show dev "${SENSOR_INTERFACE}" |
    awk -v expected="${SERVER_ADDRESS}/24" '$4 == expected {found=1} END {exit !found}'; then
  echo "ERROR: ${SENSOR_INTERFACE} does not own ${SERVER_ADDRESS}/24" >&2
  exit 4
fi

# For a first install, require a trusted upstream clock before replacing
# systemd-timesyncd. On later idempotent runs chrony itself is authoritative.
if ! command -v chronyc >/dev/null 2>&1; then
  if [[ "$(timedatectl show -p NTPSynchronized --value)" != "yes" ]]; then
    echo "ERROR: .140 is not synchronized to its upstream NTP source" >&2
    exit 5
  fi
fi

backup_root="/var/backups/traffic-light-time/$(date -u +%Y%m%dT%H%M%SZ)"
install -d -m 0700 "${backup_root}"
timedatectl show >"${backup_root}/timedatectl-before.txt"
dpkg-query -W -f='${binary:Package}\t${Version}\n' chrony systemd-timesyncd \
  >"${backup_root}/packages-before.txt" 2>&1 || true
if [[ -e "${CHRONY_DROPIN}" ]]; then
  cp -a "${CHRONY_DROPIN}" "${backup_root}/chrony-dropin.before"
fi

apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y chrony

if ! grep -Eq '^[[:space:]]*confdir[[:space:]]+/etc/chrony/conf\.d([[:space:]]|$)' \
    /etc/chrony/chrony.conf; then
  echo "ERROR: chrony.conf does not load /etc/chrony/conf.d" >&2
  exit 6
fi

dropin_tmp="$(mktemp)"
trap 'rm -f "${dropin_tmp}"' EXIT
cat >"${dropin_tmp}" <<EOF
# Managed by traffic-light deploy/install_vehicle_time_server.sh
# Listen only on the isolated sensor bridge and answer only the two Jetsons.
bindaddress ${SERVER_ADDRESS}
allow ${CLIENT_PRIMARY}/32
allow ${CLIENT_SECONDARY}/32
EOF
install -o root -g root -m 0644 "${dropin_tmp}" "${CHRONY_DROPIN}"

# Parse the full configuration before restarting the daemon.
chronyd -p -f /etc/chrony/chrony.conf >/dev/null
systemctl enable chrony.service
systemctl restart chrony.service
systemctl is-active --quiet chrony.service

# A new chrony drift model can report an intentionally huge frequency skew for
# several polling cycles. Gate on a selected upstream, normal leap state, and a
# bounded current correction; these are the properties needed by NTP clients.
synchronized=no
for _ in $(seq 1 60); do
  tracking="$(chronyc tracking)"
  ref_id="$(awk -F: '/^Reference ID/ {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2}' <<<"${tracking}")"
  leap_status="$(awk -F: '/^Leap status/ {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); print $2}' <<<"${tracking}")"
  correction="$(awk -F: '/^System time/ {value=$2; gsub(/^[[:space:]]+/, "", value); print value}' <<<"${tracking}")"
  if [[ -n "${ref_id}" && "${ref_id}" != 00000000* && "${leap_status}" == "Normal" ]] &&
      awk -v value="${correction}" 'BEGIN {exit !(value + 0 < 1.0 && value + 0 > -1.0)}'; then
    synchronized=yes
    break
  fi
  sleep 1
done
if [[ "${synchronized}" != "yes" ]]; then
  echo "ERROR: chrony did not select a usable upstream within 60 seconds" >&2
  chronyc tracking >&2 || true
  chronyc sources -v >&2 || true
  exit 7
fi

if ! ss -H -lun | grep -Eq "(^|[[:space:]])${SERVER_ADDRESS}:123([[:space:]]|$)"; then
  echo "ERROR: UDP/123 is not bound to ${SERVER_ADDRESS}" >&2
  ss -lun >&2 || true
  exit 8
fi

# Add interface- and source-specific firewall rules only when UFW is active.
# The chrony allow directives remain the mandatory application-layer ACL.
if command -v ufw >/dev/null 2>&1 && ufw status | grep -q '^Status: active'; then
  ufw allow in on "${SENSOR_INTERFACE}" from "${CLIENT_PRIMARY}" \
    to "${SERVER_ADDRESS}" port 123 proto udp comment 'traffic-time-primary'
  ufw allow in on "${SENSOR_INTERFACE}" from "${CLIENT_SECONDARY}" \
    to "${SERVER_ADDRESS}" port 123 proto udp comment 'traffic-time-secondary'
fi

if [[ -e /dev/rtc0 ]]; then
  hwclock --systohc --utc
fi

if [[ "$(sysctl -n net.ipv4.ip_forward)" != "0" ]]; then
  echo "ERROR: unexpected forwarding change" >&2
  exit 9
fi

echo "TIME_SERVER_READY=1"
echo "BACKUP_DIR=${backup_root}"
date --iso-8601=ns
chronyc tracking
chronyc sources -v
ss -lunp | grep -E "(^|[[:space:]])${SERVER_ADDRESS}:123([[:space:]]|$)"
sysctl net.ipv4.ip_forward

