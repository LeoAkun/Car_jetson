#!/usr/bin/env bash
set -Eeuo pipefail

# Point one Jetson at the vehicle IPC's restricted NTP service.
# Run only after install_vehicle_time_server.sh reports TIME_SERVER_READY=1.

readonly SERVER_ADDRESS="${SERVER_ADDRESS:-192.168.1.140}"
readonly CLIENT_DROPIN="/etc/systemd/timesyncd.conf.d/90-traffic-sensor-time.conf"

if [[ ${EUID} -ne 0 ]]; then
  echo "ERROR: run this script as root" >&2
  exit 2
fi

if ! systemctl list-unit-files systemd-timesyncd.service --no-legend |
    grep -q '^systemd-timesyncd.service'; then
  echo "ERROR: systemd-timesyncd is not installed on this Jetson" >&2
  exit 3
fi

route_line="$(ip -4 route get "${SERVER_ADDRESS}")"
if [[ "${route_line}" != *"${SERVER_ADDRESS}"* || "${route_line}" != *" src 192.168.1."* ]]; then
  echo "ERROR: ${SERVER_ADDRESS} is not reachable through the sensor subnet" >&2
  echo "${route_line}" >&2
  exit 4
fi

# Verify a valid NTP server response before changing the persistent client.
python3 - "${SERVER_ADDRESS}" <<'PY'
import socket
import struct
import sys
import time

server = sys.argv[1]
packet = bytearray(48)
packet[0] = 0x23  # LI=0, version=4, mode=3 (client)
now = time.time() + 2_208_988_800
seconds = int(now)
fraction = int((now - seconds) * (1 << 32))
packet[40:48] = struct.pack("!II", seconds, fraction)

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.settimeout(3.0)
    sock.sendto(packet, (server, 123))
    reply, peer = sock.recvfrom(512)

if peer[0] != server or len(reply) < 48:
    raise SystemExit("invalid NTP peer or short response")
li = reply[0] >> 6
mode = reply[0] & 0x07
stratum = reply[1]
if li == 3 or mode != 4 or not 1 <= stratum <= 15:
    raise SystemExit(f"unusable NTP response: li={li} mode={mode} stratum={stratum}")
if reply[24:32] != packet[40:48]:
    raise SystemExit("NTP originate timestamp mismatch")
print(f"NTP_PREFLIGHT_OK=1 server={server} stratum={stratum}")
PY

backup_root="/var/backups/traffic-light-time/$(date -u +%Y%m%dT%H%M%SZ)"
install -d -m 0700 "${backup_root}"
timedatectl show >"${backup_root}/timedatectl-before.txt"
if [[ -e "${CLIENT_DROPIN}" ]]; then
  cp -a "${CLIENT_DROPIN}" "${backup_root}/timesyncd-dropin.before"
  had_dropin=1
else
  had_dropin=0
fi

rollback() {
  status=$?
  trap - ERR
  if [[ ${had_dropin} -eq 1 ]]; then
    cp -a "${backup_root}/timesyncd-dropin.before" "${CLIENT_DROPIN}"
  else
    rm -f "${CLIENT_DROPIN}"
  fi
  systemctl restart systemd-timesyncd.service || true
  echo "ERROR: client configuration failed and was rolled back" >&2
  exit "${status}"
}
trap rollback ERR

install -d -o root -g root -m 0755 /etc/systemd/timesyncd.conf.d
dropin_tmp="$(mktemp)"
trap 'rm -f "${dropin_tmp}"' EXIT
cat >"${dropin_tmp}" <<EOF
# Managed by traffic-light deploy/install_jetson_time_client.sh
[Time]
NTP=${SERVER_ADDRESS}
FallbackNTP=
RootDistanceMaxSec=5
PollIntervalMinSec=32
PollIntervalMaxSec=2048
EOF
install -o root -g root -m 0644 "${dropin_tmp}" "${CLIENT_DROPIN}"

timedatectl set-ntp true
systemctl enable systemd-timesyncd.service
systemctl restart systemd-timesyncd.service

synchronized=no
for _ in $(seq 1 45); do
  synchronized="$(timedatectl show -p NTPSynchronized --value)"
  [[ "${synchronized}" == "yes" ]] && break
  sleep 2
done
if [[ "${synchronized}" != "yes" ]]; then
  timedatectl timesync-status >&2 || true
  journalctl -u systemd-timesyncd.service -n 40 --no-pager >&2 || true
  false
fi

server_seen="$(timedatectl show-timesync --property=ServerAddress --value)"
if [[ "${server_seen}" != "${SERVER_ADDRESS}" ]]; then
  echo "ERROR: synchronized with unexpected server ${server_seen}" >&2
  false
fi

trap - ERR
echo "TIME_CLIENT_READY=1"
echo "BACKUP_DIR=${backup_root}"
date --iso-8601=ns
timedatectl show -p Timezone -p NTP -p NTPSynchronized
timedatectl timesync-status
systemctl is-enabled systemd-timesyncd.service

