#!/usr/bin/env bash
set -Eeuo pipefail

# Generate one device key and CSR on the device that will use the key.
# The private key never leaves OUTPUT_DIR; only the CSR is exported for signing.

usage() {
  echo "usage: $0 ROLE OUTPUT_DIR" >&2
  echo "ROLE: service | video-sender | controller" >&2
  exit 2
}

[[ $# -eq 2 ]] || usage
readonly role="$1"
readonly output_dir="$2"

case "${role}" in
  service)
    common_name="traffic-light-service-150"
    eku="serverAuth"
    sans="DNS:traffic-light.service,IP:192.168.1.150,URI:urn:car-jetson:traffic-light:role:service"
    ;;
  video-sender)
    common_name="traffic-light-video-sender-140"
    eku="clientAuth"
    sans="URI:urn:car-jetson:traffic-light:role:video-sender"
    ;;
  controller)
    common_name="traffic-light-controller-140"
    eku="clientAuth"
    sans="URI:urn:car-jetson:traffic-light:role:controller"
    ;;
  *) usage ;;
esac

if [[ "${output_dir}" != /* ]]; then
  echo "ERROR: OUTPUT_DIR must be absolute" >&2
  exit 3
fi

umask 077
install -d -m 0700 "${output_dir}"
key_file="${output_dir}/${role}.key"
csr_file="${output_dir}/${role}.csr"

if [[ -e "${key_file}" || -e "${csr_file}" ]]; then
  echo "ERROR: refusing to overwrite ${role} identity" >&2
  exit 4
fi

openssl genpkey \
  -algorithm EC \
  -pkeyopt ec_paramgen_curve:P-256 \
  -out "${key_file}"
chmod 0600 "${key_file}"

openssl req \
  -new \
  -sha256 \
  -key "${key_file}" \
  -subj "/CN=${common_name}" \
  -addext "basicConstraints=critical,CA:FALSE" \
  -addext "keyUsage=critical,digitalSignature" \
  -addext "extendedKeyUsage=${eku}" \
  -addext "subjectAltName=${sans}" \
  -out "${csr_file}"
chmod 0644 "${csr_file}"

openssl req -in "${csr_file}" -noout -verify >/dev/null
key_public_hash="$(openssl pkey -in "${key_file}" -pubout -outform DER 2>/dev/null | sha256sum | awk '{print $1}')"
csr_public_hash="$(openssl req -in "${csr_file}" -pubkey -noout | openssl pkey -pubin -outform DER 2>/dev/null | sha256sum | awk '{print $1}')"
if [[ "${key_public_hash}" != "${csr_public_hash}" ]]; then
  echo "ERROR: CSR public key does not match private key" >&2
  exit 5
fi

echo "TLS_IDENTITY_READY=1"
echo "ROLE=${role}"
echo "CSR=${csr_file}"
echo "PUBLIC_KEY_SHA256=${key_public_hash}"
openssl req -in "${csr_file}" -noout -subject

