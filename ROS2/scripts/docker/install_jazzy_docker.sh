#!/usr/bin/env bash
# 运行时机：新开发板首次部署或系统重装后执行一次，用于在 Debian 12 安装并启用 Docker。
# 典型命令：sudo bash scripts/docker/install_jazzy_docker.sh
# Docker 已正常安装时无需重复执行；执行后需要注销并重新登录，使 docker 用户组生效。

set -euo pipefail

# ROS 2 Jazzy 没有面向 Debian 12 ARM64 的官方 deb，使用官方 Ubuntu 容器承载。
if [[ "$(id -u)" -ne 0 ]]; then
  echo "请使用 root 权限运行：sudo bash scripts/docker/install_jazzy_docker.sh" >&2
  exit 1
fi

apt-get update
apt-get install -y --no-install-recommends docker.io ca-certificates
apt-get clean
systemctl enable --now docker

# 让当前登录用户后续无需 sudo 即可启动机器人容器。
TARGET_USER="${SUDO_USER:-}"
if [[ -n "${TARGET_USER}" && "${TARGET_USER}" != "root" ]]; then
  usermod -aG docker "${TARGET_USER}"
  echo "已将 ${TARGET_USER} 加入 docker 组；请注销并重新登录后再运行启动脚本。"
fi

echo "Docker 安装完成。重新登录后执行：bash scripts/docker/build_jazzy_image.sh"
