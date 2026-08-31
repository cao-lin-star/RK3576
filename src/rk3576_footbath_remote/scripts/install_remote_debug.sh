#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  sudo bash install_remote_debug.sh --user sky --public-key /path/key.pub \
    [--harden] [--enable-motion-tools] [--enable-visualization]

Remote motion, including the narrowly whitelisted Foxglove input, remains
protected by a short root-owned arm lease. Visualization binds only to
127.0.0.1:8765 and is reached through an SSH port forward.
EOF
}

target_user="sky"
public_key=""
harden=0
enable_motion=0
enable_visualization=0
while (($#)); do
  case "$1" in
    --user) [[ $# -ge 2 ]] || exit 2; target_user="$2"; shift ;;
    --public-key) [[ $# -ge 2 ]] || exit 2; public_key="$2"; shift ;;
    --harden) harden=1 ;;
    --enable-motion-tools) enable_motion=1 ;;
    --enable-visualization) enable_visualization=1 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

[[ ${EUID} -eq 0 ]] || { printf 'Run with sudo.\n' >&2; exit 1; }
id "$target_user" >/dev/null 2>&1 || { printf 'User does not exist: %s\n' "$target_user" >&2; exit 1; }
[[ -n "$public_key" && -r "$public_key" ]] || { printf 'A readable --public-key file is required.\n' >&2; exit 1; }

key_line="$(head -n 1 "$public_key")"
case "$key_line" in
  ssh-ed25519\ *|ssh-rsa\ *|ecdsa-sha2-nistp*\ *) ;;
  *) printf 'Unsupported or invalid SSH public key.\n' >&2; exit 1 ;;
esac

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd -- "${script_dir}/.." && pwd)"
if [[ ! -r "${package_dir}/config/remote-debug.env.example" ]]; then
  installed_share="$(cd -- "${script_dir}/../../share/rk3576_footbath_remote" 2>/dev/null && pwd || true)"
  if [[ -n "$installed_share" && -r "${installed_share}/config/remote-debug.env.example" ]]; then
    package_dir="$installed_share"
  else
    printf 'Cannot locate package config/deploy files.\n' >&2
    exit 1
  fi
fi

target_home="$(getent passwd "$target_user" | cut -d: -f6)"
target_group="$(id -gn "$target_user")"
packages=(openssh-server openocd python3-serial rsync tmux ros-humble-foxglove-bridge)
missing_packages=()
for package_name in "${packages[@]}"; do
  if ! dpkg-query -W -f='${Status}' "$package_name" 2>/dev/null | grep -q 'ok installed'; then
    missing_packages+=("$package_name")
  fi
done
if ((${#missing_packages[@]})); then
  export DEBIAN_FRONTEND=noninteractive
  apt-get -o Acquire::ForceIPv4=true update
  apt-get -o Acquire::ForceIPv4=true install -y "${missing_packages[@]}"
fi

install -d -m 0700 -o "$target_user" -g "$target_group" "${target_home}/.ssh"
touch "${target_home}/.ssh/authorized_keys"
chown "$target_user:$target_group" "${target_home}/.ssh/authorized_keys"
chmod 0600 "${target_home}/.ssh/authorized_keys"
if ! grep -qxF "$key_line" "${target_home}/.ssh/authorized_keys"; then
  printf '%s\n' "$key_line" >>"${target_home}/.ssh/authorized_keys"
fi

commands=(
  footbath_remote_status footbath_collect_debug footbath_motion_arm
  footbath_safe_drive footbath_safe_drive.py
  footbath_uart4_console footbath_uart4_console.py
  footbath_loaded_deadband footbath_loaded_deadband.py
  footbath_loaded_pid_test footbath_flash_f407 footbath_visualization_bridge
)
for command_name in "${commands[@]}"; do
  install -m 0755 "${script_dir}/${command_name}" "/usr/local/bin/${command_name}"
done

install -d -m 0755 /etc/footbath
env_file=/etc/footbath/remote-debug.env
if [[ ! -e "$env_file" ]]; then
  sed \
    -e "s|^FOOTBATH_WS=.*|FOOTBATH_WS=${target_home}/rk3576_footbath_ws|" \
    -e "s|^FOOTBATH_DEBUG_DIR=.*|FOOTBATH_DEBUG_DIR=${target_home}/footbath_debug|" \
    -e "s|^FOOTBATH_TUNING_DIR=.*|FOOTBATH_TUNING_DIR=${target_home}/footbath_tuning|" \
    "${package_dir}/config/remote-debug.env.example" >"$env_file"
fi
if grep -qx 'FOOTBATH_SERVICE=rk3576-footbath.service' "$env_file"; then
  sed -i 's/^FOOTBATH_SERVICE=rk3576-footbath.service$/FOOTBATH_SERVICE=footbath-mobile.service/' \
    "$env_file"
fi
if [[ $enable_motion -eq 1 ]]; then
  sed -i 's/^FOOTBATH_ALLOW_REMOTE_MOTION=.*/FOOTBATH_ALLOW_REMOTE_MOTION=1/' "$env_file"
fi
chmod 0644 "$env_file"

install -d -m 0750 -o "$target_user" -g "$target_group" \
  "${target_home}/footbath_debug" \
  "${target_home}/footbath_tuning" \
  "${target_home}/f407_firmware_inbox"

install -d -m 0755 /etc/ssh/sshd_config.d
if [[ $harden -eq 1 ]]; then
  ssh_dropin=/etc/ssh/sshd_config.d/60-footbath-remote.conf
  backup=""
  if [[ -e "$ssh_dropin" ]]; then
    backup="${ssh_dropin}.bak.$(date +%Y%m%d_%H%M%S)"
    cp -a -- "$ssh_dropin" "$backup"
  fi
  install -m 0644 "${package_dir}/deploy/60-footbath-remote.conf" "$ssh_dropin"
  if ! /usr/sbin/sshd -t; then
    if [[ -n "$backup" ]]; then cp -a -- "$backup" "$ssh_dropin"; else rm -f -- "$ssh_dropin"; fi
    printf 'SSH validation failed; previous configuration restored.\n' >&2
    exit 1
  fi
fi

install -m 0644 "${package_dir}/deploy/systemd/footbath-foxglove.service" \
  /etc/systemd/system/footbath-foxglove.service
systemctl daemon-reload
systemctl enable --now ssh.service
systemctl enable --now tailscaled.service
if [[ $enable_visualization -eq 1 ]]; then
  systemctl enable --now footbath-foxglove.service
else
  systemctl disable --now footbath-foxglove.service 2>/dev/null || true
fi
if [[ $harden -eq 1 ]]; then
  systemctl reload ssh.service
fi

printf 'REMOTE_DEBUG_INSTALL_OK\n'
printf 'motion_policy=%s\n' "$(sed -n 's/^FOOTBATH_ALLOW_REMOTE_MOTION=//p' "$env_file")"
printf 'visualization_state=%s\n' "$(systemctl is-active footbath-foxglove.service 2>/dev/null || true)"
