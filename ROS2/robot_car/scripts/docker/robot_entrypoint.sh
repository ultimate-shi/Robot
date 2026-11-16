#!/usr/bin/env bash
# 作用：为所有 Jazzy 容器命令加载 Astra 驱动；用法：Dockerfile 的 ENTRYPOINT，不启动任何机器人节点。
set -e
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/robot/astra_ws/install/setup.bash ]]; then
  source /opt/robot/astra_ws/install/setup.bash
fi
exec "$@"
