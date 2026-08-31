#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  echo "请使用 sudo $0" >&2
  exit 2
fi

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$workspace_dir" != "/home/sky/rk3576_footbath_ws" ]]; then
  echo "systemd 文件按 /home/sky/rk3576_footbath_ws 设计，当前是 $workspace_dir" >&2
  exit 2
fi

legacy_matches="$(
  grep -RnsF 'SYMLINK+="footbath_stm32"' /etc/udev/rules.d 2>/dev/null | grep -v 'KERNEL=="ttyS3"' || true
)"
if [[ -n "$legacy_matches" ]]; then
  echo "ERROR: 检测到仍把USB/fireDAP命名为footbath_stm32的旧udev规则：" >&2
  echo "$legacy_matches" >&2
  echo "请先备份并删除这些旧规则行，再重新运行安装脚本。" >&2
  exit 2
fi

legacy_lidar_matches="$(
  grep -RnsF 'SYMLINK+="footbath_lidar_front"' /etc/udev/rules.d 2>/dev/null || true
)"
if [[ -n "$legacy_lidar_matches" ]]; then
  echo "ERROR: 检测到已废止的单雷达footbath_lidar_front规则：" >&2
  echo "$legacy_lidar_matches" >&2
  echo "请先备份并删除该旧别名，保留/改为high与low双雷达规则。" >&2
  exit 2
fi

install -m 0644 "$workspace_dir/deploy/systemd/rk3576-footbath.service" /etc/systemd/system/
install -m 0644 "$workspace_dir/deploy/systemd/rk3576-footbath.env" /etc/default/rk3576-footbath
install -m 0644 "$workspace_dir/deploy/udev/99-footbath-lidars.rules" /etc/udev/rules.d/99-footbath-lidars.rules
install -m 0644 "$workspace_dir/deploy/udev/99-footbath-uart3-m0.rules" /etc/udev/rules.d/99-footbath-uart3-m0.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
udevadm settle
systemctl daemon-reload
echo "已安装 systemd 和 UART3_M0 稳定别名规则。"
if [[ -c /dev/ttyS3 ]]; then
  uart_alias_target="$(readlink -f /dev/footbath_stm32 2>/dev/null || true)"
  if [[ "$uart_alias_target" != "/dev/ttyS3" ]]; then
    echo "ERROR: /dev/footbath_stm32 当前指向 '$uart_alias_target'，预期 /dev/ttyS3。" >&2
    echo "请检查并删除仍占用该别名的旧USB/fireDAP udev规则。" >&2
    exit 2
  fi
  echo "UART3_M0: /dev/footbath_stm32 -> /dev/ttyS3"
else
  echo "警告：/dev/ttyS3 不存在；先启用 uart3-m0 设备树插件并重启。" >&2
fi
for lidar_alias in /dev/footbath_lidar_high /dev/footbath_lidar_low; do
  if [[ ! -e "$lidar_alias" ]]; then
    echo "ERROR: 缺少双雷达别名 $lidar_alias。" >&2
    exit 2
  fi
done
echo "RPLIDAR C1: high=$(readlink -f /dev/footbath_lidar_high), low=$(readlink -f /dev/footbath_lidar_low)"
echo "请核对 /etc/default/rk3576-footbath 后执行 systemctl enable --now rk3576-footbath"
