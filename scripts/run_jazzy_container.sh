#!/usr/bin/env bash
# 运行时机：只需要 ROS 2 Jazzy 容器环境、不希望自动构建或启动 launch 时执行。
# 典型命令：bash scripts/run_jazzy_container.sh。
# 脚本会在后台启动容器；之后使用 docker exec -it robot-jazzy bash 进入。

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONTAINER_NAME="robot-jazzy"
IMAGE_NAME="robot-jazzy:local"

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "未找到 ${IMAGE_NAME}，请先运行 bash scripts/build_jazzy_image.sh" >&2
  exit 1
fi

if docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  if [[ "$(docker inspect --format '{{.State.Running}}' "${CONTAINER_NAME}")" == "true" ]]; then
    echo "容器 ${CONTAINER_NAME} 已经在运行。"
    echo "进入容器：docker exec -it ${CONTAINER_NAME} bash"
    exit 0
  fi

  echo "已存在同名的停止容器 ${CONTAINER_NAME}，请先处理该容器后再运行本脚本。" >&2
  exit 1
fi

# 只启动环境时也要在创建阶段映射相机；Docker 无法给运行中的容器追加设备。
CAMERA_DEVICE_ARGS=()
if [[ -e /dev/stereo_camera ]]; then
  CAMERA_DEVICE_ARGS=(--device /dev/stereo_camera:/dev/video0)
  echo "已映射双目相机：/dev/stereo_camera -> 容器 /dev/video0"
else
  echo "提示：未找到 /dev/stereo_camera，本次容器不映射真实双目相机。" >&2
fi

# 容器仅提供 Jazzy 环境，不执行 colcon build，也不启动任何 ROS launch。
docker run --rm --detach --interactive --tty \
  --name "${CONTAINER_NAME}" \
  --network host \
  --ipc host \
  --ulimit core=0 \
  "${CAMERA_DEVICE_ARGS[@]}" \
  --volume "${WORKSPACE_DIR}:/workspace" \
  --workdir /workspace \
  "${IMAGE_NAME}" \
  bash -lc '
    set -e

    # 让 docker exec 打开的交互式 Bash 自动加载 Jazzy 和已有的工作区环境。
    printf "%s\n" \
      "" \
      "# 自动加载 ROS 2 Jazzy 与 robot 工作区。" \
      "source /opt/ros/jazzy/setup.bash" \
      "if [[ -f /workspace/install/setup.bash ]]; then" \
      "  source /workspace/install/setup.bash" \
      "fi" \
      >> /root/.bashrc

    exec bash
  ' >/dev/null

echo "Jazzy 容器已在后台启动，未构建项目，也未启动 launch。"
if [[ ${#CAMERA_DEVICE_ARGS[@]} -gt 0 ]]; then
  echo "容器相机设备：/dev/video0"
fi
echo "进入容器：docker exec -it ${CONTAINER_NAME} bash"
echo "停止容器：docker stop ${CONTAINER_NAME}"
