#!/usr/bin/env bash
set -Eeuo pipefail

# Run only in the offline CA environment. It never creates or copies a CA key.
# Leaf extensions are fixed here and are not copied blindly from the CSR.

usage() {
  echo "usage: $0 CA_CERT CA_KEY CSR_DIR OUTPUT_DIR [DAYS]" >&2
  exit 2
}

[[ $# -ge 4 && $# -le 5 ]] || usage
readonly ca_cert="$1"
readonly ca_key="$2"
readonly csr_dir="$3"
readonly output_dir="$4"
readonly days="${5:-365}"

for path in "${ca_cert}" "${ca_key}" "${csr_dir}" "${output_dir}"; do
  [[ "${path}" == /* ]] || { echo "ERROR: all paths must be absolute" >&2; exit 3; }
done
[[ "${days}" =~ ^[0-9]+$ ]] && (( days >= 1 && days <= 825 )) || {
  echo "ERROR: DAYS must be 1..825" >&2
  exit 4
}
[[ -f "${ca_cert}" && -f "${ca_key}" ]] || {
  echo "ERROR: CA certificate or key is missing" >&2
  exit 5
}
key_mode="$(stat -c '%a' "${ca_key}")"
[[ "${key_mode}" == 400 || "${key_mode}" == 600 ]] || {
  echo "ERROR: CA key mode must be 0400 or 0600" >&2
  exit 6
}

openssl x509 -in "${ca_cert}" -noout -checkend 86400 >/dev/null
tmp_ext=""
openssl pkey -in "${ca_key}" -pubout -outform DER 2>/dev/null |
  sha256sum | awk '{print $1}' >"${TMPDIR:-/tmp}/traffic-light-ca-key-pub.$$"
trap 'python3 - "$tmp_ext" "${TMPDIR:-/tmp}/traffic-light-ca-key-pub.$$" <<'PY'
import pathlib, sys
for value in sys.argv[1:]:
    if value:
        pathlib.Path(value).unlink(missing_ok=True)
PY' EXIT
cert_public_hash="$(openssl x509 -in "${ca_cert}" -pubkey -noout | openssl pkey -pubin -outform DER 2>/dev/null | sha256sum | awk '{print $1}')"
key_public_hash="$(cat "${TMPDIR:-/tmp}/traffic-light-ca-key-pub.$$")"
[[ "${cert_public_hash}" == "${key_public_hash}" ]] || {
  echo "ERROR: CA certificate and key do not match" >&2
  exit 7
}

umask 077
install -d -m 0700 "${output_dir}"
serial_file="${output_dir}/ca.srl"

issue() {
  local role="$1" expected_cn="$2" eku="$3" sans="$4"
  local csr="${csr_dir}/${role}.csr" cert="${output_dir}/${role}.crt"
  [[ -f "${csr}" ]] || { echo "ERROR: missing ${csr}" >&2; exit 8; }
  [[ ! -e "${cert}" ]] || { echo "ERROR: refusing to overwrite ${cert}" >&2; exit 9; }
  openssl req -in "${csr}" -noout -verify >/dev/null
  subject="$(openssl req -in "${csr}" -noout -subject -nameopt RFC2253)"
  [[ "${subject}" == "subject=CN=${expected_cn}" ]] || {
    echo "ERROR: unexpected subject for ${role}: ${subject}" >&2
    exit 10
  }
  tmp_ext="$(mktemp)"
  cat >"${tmp_ext}" <<EOF
basicConstraints=critical,CA:FALSE
keyUsage=critical,digitalSignature
extendedKeyUsage=${eku}
subjectAltName=${sans}
subjectKeyIdentifier=hash
authorityKeyIdentifier=keyid,issuer
EOF
  serial_args=(-CAserial "${serial_file}")
  [[ -e "${serial_file}" ]] || serial_args=(-CAcreateserial -CAserial "${serial_file}")
  openssl x509 -req -sha256 -days "${days}" \
    -in "${csr}" -CA "${ca_cert}" -CAkey "${ca_key}" \
    "${serial_args[@]}" -extfile "${tmp_ext}" -out "${cert}"
  chmod 0644 "${cert}"
  python3 - "${tmp_ext}" <<'PY'
import pathlib, sys
pathlib.Path(sys.argv[1]).unlink(missing_ok=True)
PY
  tmp_ext=""
  purpose=sslclient
  [[ "${eku}" == serverAuth ]] && purpose=sslserver
  openssl verify -CAfile "${ca_cert}" -purpose "${purpose}" "${cert}" >/dev/null
  cert_pub="$(openssl x509 -in "${cert}" -pubkey -noout | openssl pkey -pubin -outform DER 2>/dev/null | sha256sum | awk '{print $1}')"
  csr_pub="$(openssl req -in "${csr}" -pubkey -noout | openssl pkey -pubin -outform DER 2>/dev/null | sha256sum | awk '{print $1}')"
  [[ "${cert_pub}" == "${csr_pub}" ]] || { echo "ERROR: signed key mismatch for ${role}" >&2; exit 11; }
  echo "SIGNED role=${role} serial=$(openssl x509 -in "${cert}" -noout -serial | cut -d= -f2)"
}

issue service traffic-light-service-150 serverAuth \
  "DNS:traffic-light.service,IP:192.168.1.150,URI:urn:car-jetson:traffic-light:role:service"
issue video-sender traffic-light-video-sender-140 clientAuth \
  "URI:urn:car-jetson:traffic-light:role:video-sender"
issue controller traffic-light-controller-140 clientAuth \
  "URI:urn:car-jetson:traffic-light:role:controller"

cp "${ca_cert}" "${output_dir}/ca.crt"
chmod 0644 "${output_dir}/ca.crt"
echo "TLS_CERTIFICATES_READY=1"
echo "OUTPUT_DIR=${output_dir}"

