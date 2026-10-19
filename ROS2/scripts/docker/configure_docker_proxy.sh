#!/usr/bin/env bash
# 运行时机：Docker 首次拉取镜像无法联网，或宿主机代理地址、端口发生变化时执行。
# 典型命令：sudo bash scripts/docker/configure_docker_proxy.sh http://127.0.0.1:7897
# 该脚本会重启 Docker daemon，机器人容器正常运行期间不要执行。

set -euo pipefail

# Docker daemon 不继承登录终端代理，需要显式写入 systemd 服务环境。
if [[ "$(id -u)" -ne 0 ]]; then
  echo "请使用 root 权限运行：sudo bash scripts/docker/configure_docker_proxy.sh" >&2
  exit 1
fi

PROXY_URL="${1:-http://127.0.0.1:7897}"
TARGET_USER="${SUDO_USER:-radxa}"
DROP_IN_DIR="/etc/systemd/system/docker.service.d"
DROP_IN_FILE="${DROP_IN_DIR}/proxy.conf"

install -d -m 0755 "${DROP_IN_DIR}"
printf '%s\n' \
  '[Service]' \
  "Environment=\"HTTP_PROXY=${PROXY_URL}\"" \
  "Environment=\"HTTPS_PROXY=${PROXY_URL}\"" \
  'Environment="NO_PROXY=localhost,127.0.0.1,::1"' \
  > "${DROP_IN_FILE}"

systemctl daemon-reload
systemctl restart docker

# Docker 重启会重新创建 socket；给当前 Codex 会话保留访问权限。
if command -v setfacl >/dev/null 2>&1 && id "${TARGET_USER}" >/dev/null 2>&1; then
  setfacl -m "u:${TARGET_USER}:rw" /var/run/docker.sock
fi

echo "Docker daemon 代理已设置为 ${PROXY_URL}。"
systemctl show docker --property=Environment
