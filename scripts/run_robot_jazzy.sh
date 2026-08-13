#!/usr/bin/env bash
# 运行时机：每次启动机器人 ROS 2 系统时执行；开发板重启或容器停止后需要再次执行。
# 典型命令：bash scripts/run_robot_jazzy.sh，也可在后面追加 robot.launch.py 的参数。
# 脚本会挂载当前仓库、增量构建 robot 包，并在前台运行主 launch；按 Ctrl+C 即停止。

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

if ! docker image inspect robot-jazzy:local >/dev/null 2>&1; then
  echo "未找到 robot-jazzy:local，请先运行 bash scripts/build_jazzy_image.sh" >&2
  exit 1
fi

# 真实相机存在时自动映射到容器标准视频节点；仿真场景没有相机也允许正常启动。
CAMERA_DEVICE_ARGS=()
if [[ -e /dev/stereo_camera ]]; then
  CAMERA_DEVICE_ARGS=(--device /dev/stereo_camera:/dev/video0)
  echo "已映射双目相机：/dev/stereo_camera -> 容器 /dev/video0"
else
  echo "提示：未找到 /dev/stereo_camera，本次容器不映射真实双目相机。" >&2
fi

# host 网络让 DDS 与 Foxglove Bridge 直接使用开发板 IP；工作区挂载便于持续开发。
exec docker run --rm --interactive --tty \
  --name robot-jazzy \
  --network host \
  --ipc host \
  "${CAMERA_DEVICE_ARGS[@]}" \
  --volume "${WORKSPACE_DIR}:/workspace" \
  --workdir /workspace \
  robot-jazzy:local \
  bash -lc '
    set -e

    # 让后续通过 docker exec 打开的交互式 Bash 自动加载 ROS 2 和当前工作区。
    printf "%s\n" \
      "" \
      "# 自动加载 ROS 2 Jazzy 与 robot 工作区。" \
      "source /opt/ros/jazzy/setup.bash" \
      "if [[ -f /workspace/install/setup.bash ]]; then" \
      "  source /workspace/install/setup.bash" \
      "fi" \
      >> /root/.bashrc

    source /opt/ros/jazzy/setup.bash
    colcon build --packages-up-to robot --symlink-install
    source install/setup.bash
    exec ros2 launch robot robot.launch.py "$@"
  ' bash "$@"
