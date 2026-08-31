#!/usr/bin/env bash
set -euo pipefail
[[ ${EUID} -eq 0 ]] || { echo "Run with sudo."; exit 1; }
target_user="${1:-sky}"
id "$target_user" >/dev/null
target_group="$(id -gn "$target_user")"
workspace="/home/${target_user}/rk3576_footbath_ws"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
share_dir="$(cd -- "${script_dir}/.." && pwd)"
env_file=/etc/footbath/mobile.env
reset_pin="${2:-}"
install -d -m 0755 /etc/footbath
if [[ -s "$env_file" && "$reset_pin" != "--reset-pin" ]] \
   && grep -q '^FOOTBATH_MOBILE_PIN_SALT=.' "$env_file" \
   && grep -q '^FOOTBATH_MOBILE_PIN_HASH=.' "$env_file"; then
  echo "Preserving existing mobile PIN configuration."
else
  read -rsp "Set mobile control PIN (minimum 6 characters): " pin; echo
  [[ ${#pin} -ge 6 ]] || { echo "PIN too short."; exit 2; }
  salt="$(openssl rand -hex 16)"
  hash="$(PIN_VALUE="$pin" SALT_VALUE="$salt" python3 -c 'import hashlib,os; print(hashlib.pbkdf2_hmac("sha256",os.environ["PIN_VALUE"].encode(),bytes.fromhex(os.environ["SALT_VALUE"]),200000).hex())')"
  unset pin
  sed -e "s|^FOOTBATH_WS=.*|FOOTBATH_WS=/home/${target_user}/rk3576_footbath_ws|" \
      -e "s/^FOOTBATH_MOBILE_PIN_SALT=.*/FOOTBATH_MOBILE_PIN_SALT=${salt}/" \
      -e "s/^FOOTBATH_MOBILE_PIN_HASH=.*/FOOTBATH_MOBILE_PIN_HASH=${hash}/" \
      "${share_dir}/config/mobile.env.example" >"$env_file"
fi
chmod 0600 "$env_file"
write_dirs=(
  "/home/${target_user}/.ros"
  "${workspace}/maps"
  "${workspace}/src/rk3576_footbath_navigation/maps"
)
for write_dir in "${write_dirs[@]}"; do
  install -d -m 0755 -o "$target_user" -g "$target_group" "$write_dir"
  [[ -d "$write_dir" ]] || { echo "Failed to create ${write_dir}."; exit 3; }
done
install -m 0644 "${share_dir}/deploy/systemd/footbath-mobile.service" /etc/systemd/system/
systemctl disable --now rk3576-footbath.service 2>/dev/null || true
systemctl daemon-reload
systemctl enable footbath-mobile.service
systemctl restart footbath-mobile.service
for _ in {1..10}; do
  systemctl is-active --quiet footbath-mobile.service && break
  sleep 1
done
if ! systemctl is-active --quiet footbath-mobile.service; then
  systemctl stop footbath-mobile.service || true
  journalctl -u footbath-mobile.service -n 30 --no-pager || true
  echo "Mobile gateway failed to become active."
  exit 4
fi
echo MOBILE_GATEWAY_INSTALL_OK
echo 'Open http://192.168.8.10:8080 after joining G806P Wi-Fi.'
