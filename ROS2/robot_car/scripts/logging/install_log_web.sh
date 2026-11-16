#!/usr/bin/env bash
# 作用：安装独立 9100 网页入口、设置开机自启并清理旧日志网页服务；使用方法：bash scripts/logging/install_log_web.sh。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WEB_VENV="${WORKSPACE_DIR}/model/web-venv"
WEB_CONFIG="${WORKSPACE_DIR}/src/robot_web/config/web.yaml"
UNIT_DIR="${HOME}/.config/systemd/user"
UNIT_FILE="${UNIT_DIR}/robot-web.service"

if [[ ! -d /var/lib/robot-logs/index ]]; then
  echo '请先运行 scripts/docker/run_jazzy_container.sh 创建宿主机日志目录。' >&2
  exit 1
fi
if [[ -z "${XDG_RUNTIME_DIR:-}" ]]; then
  echo '需要 systemd 用户会话与 XDG_RUNTIME_DIR 才能运行推理 Socket。' >&2
  exit 1
fi
python3 -m venv "${WEB_VENV}"
"${WEB_VENV}/bin/python" -m pip install -e "${WORKSPACE_DIR}/src/robot_logging[web]" -e "${WORKSPACE_DIR}/src/robot_web"
"${WEB_VENV}/bin/python" -c 'from robot_web.config import load_config; load_config()'

install -d -m 700 "${UNIT_DIR}"
systemctl --user disable --now robot-log-web.service 2>/dev/null || true
rm -f "${UNIT_DIR}/robot-log-web.service"
cat > "${UNIT_FILE}" <<EOF
# 作用：开机常驻运行唯一的 9100 网页入口；使用方法：systemctl --user status robot-web.service。
[Unit]
Description=Robot 9100 Web Gateway

[Service]
Type=simple
WorkingDirectory=${WORKSPACE_DIR}
Environment=ROBOT_WEB_CONFIG=${WEB_CONFIG}
ExecStart=${WEB_VENV}/bin/python -m robot_web.server
Restart=on-failure
RestartSec=3

[Install]
WantedBy=default.target
EOF
systemctl --user daemon-reload
systemctl --user enable robot-web.service
systemctl --user restart robot-web.service
if [[ "$(loginctl show-user "$(id -un)" -p Linger --value)" != yes ]]; then
  if ! loginctl enable-linger "$(id -un)"; then
    echo '网页入口已启动，但开机自启需要管理员执行：sudo loginctl enable-linger '"$(id -un)" >&2
    exit 1
  fi
fi
if ! "${WEB_VENV}/bin/python" - <<'PY'
from time import sleep
from urllib.request import urlopen

for _attempt in range(30):
    try:
        with urlopen('http://127.0.0.1:9100/health', timeout=1) as response:
            if response.status == 200:
                break
    except OSError:
        sleep(0.2)
else:
    raise SystemExit('网页入口未在 9100 正常启动，请检查 robot-web.service')
PY
then
  exit 1
fi
if ! systemctl --user is-active --quiet robot-web.service; then
  echo 'robot-web.service 未保持运行，请检查用户服务日志。' >&2
  exit 1
fi
echo '网页入口：http://127.0.0.1:9100/log/'
echo '健康检查：http://127.0.0.1:9100/health'
