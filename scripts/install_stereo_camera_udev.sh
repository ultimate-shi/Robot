#!/usr/bin/env bash
# 运行时机：首次连接当前 USB 双目相机，或重新安装 Debian 系统后执行一次。
# 典型命令：sudo bash scripts/install_stereo_camera_udev.sh
# 脚本安装仓库内已按实测 VID、PID、序列号和 video index 编写的 udev 规则。

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
RULE_SOURCE="${WORKSPACE_DIR}/config/udev/99-stereo-camera.rules"
RULE_TARGET="/etc/udev/rules.d/99-stereo-camera.rules"

if [[ "${EUID}" -ne 0 ]]; then
  echo "需要管理员权限，请执行：sudo bash scripts/install_stereo_camera_udev.sh" >&2
  exit 1
fi

install -D -m 0644 "${RULE_SOURCE}" "${RULE_TARGET}"
udevadm control --reload-rules
udevadm trigger --subsystem-match=video4linux

if [[ -e /dev/stereo_camera ]]; then
  echo "已创建稳定设备：$(readlink -f /dev/stereo_camera)"
else
  echo "规则已安装。请重新插拔相机，然后执行：ls -l /dev/stereo_camera"
fi
