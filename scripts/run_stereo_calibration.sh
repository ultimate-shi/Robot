#!/usr/bin/env bash
# 从 Debian 图形桌面的终端启动双目预览或 camera_calibration 图形界面。
# 预览与遮挡检查：bash scripts/run_stereo_calibration.sh preview
# 正式标定：bash scripts/run_stereo_calibration.sh calibrate <实测格子边长米>

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CALIBRATION_OUTPUT_DIR="${WORKSPACE_DIR}/calibration_output"
MODE="${1:-preview}"
SQUARE_SIZE="${2:-}"

if [[ "${MODE}" != "preview" && "${MODE}" != "calibrate" ]]; then
  echo "用法：" >&2
  echo "  bash scripts/run_stereo_calibration.sh preview" >&2
  echo "  bash scripts/run_stereo_calibration.sh calibrate <实测格子边长米>" >&2
  exit 2
fi

if [[ "${MODE}" == "calibrate" ]]; then
  if [[ ! "${SQUARE_SIZE}" =~ ^0\.[0-9]+$ ]]; then
    echo "正式标定必须提供以米为单位的实测格子边长，例如 0.02982。" >&2
    exit 2
  fi
fi

if ! docker image inspect robot-jazzy:local >/dev/null 2>&1; then
  echo "未找到 robot-jazzy:local，请先运行 bash scripts/build_jazzy_image.sh" >&2
  exit 1
fi

if [[ ! -e /dev/stereo_camera ]]; then
  echo "未找到 /dev/stereo_camera。请先执行：" >&2
  echo "  sudo bash scripts/install_stereo_camera_udev.sh" >&2
  echo "然后重新插拔相机。" >&2
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "当前不是带 DISPLAY 的图形终端。请打开 Debian 桌面中的终端后再运行本脚本。" >&2
  exit 1
fi

if [[ ! -d /tmp/.X11-unix ]]; then
  echo "未找到 X11 显示套接字 /tmp/.X11-unix，无法显示标定窗口。" >&2
  exit 1
fi

# 标定必须关闭自动曝光和自动白平衡；默认值来自本次板端检查时的稳定画面。
"${SCRIPT_DIR}/configure_stereo_camera.sh"
mkdir -p "${CALIBRATION_OUTPUT_DIR}"

GUI_ARGS=(
  --env "DISPLAY=${DISPLAY}"
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
)

# VNC 的 X Server 通常不支持 xhost 的 localuser 授权，改为复制当前显示的临时 cookie。
if ! command -v xauth >/dev/null 2>&1; then
  echo "缺少 xauth，无法把当前 VNC 显示授权给标定容器。" >&2
  exit 1
fi

XAUTH_SOURCE="${XAUTHORITY:-${HOME}/.Xauthority}"
if [[ ! -r "${XAUTH_SOURCE}" ]]; then
  echo "无法读取当前桌面的 Xauthority：${XAUTH_SOURCE}" >&2
  exit 1
fi

XAUTH_COOKIE_FILE="$(mktemp /tmp/robot-stereo-xauth.XXXXXX)"
cleanup_gui_auth() {
  rm -f "${XAUTH_COOKIE_FILE}"
}
trap cleanup_gui_auth EXIT INT TERM

XAUTH_DATA="$(xauth -i -f "${XAUTH_SOURCE}" nlist "${DISPLAY}" 2>/dev/null || true)"
if [[ -z "${XAUTH_DATA}" ]]; then
  echo "在 ${XAUTH_SOURCE} 中找不到显示 ${DISPLAY} 的授权 cookie。" >&2
  exit 1
fi

# FamilyWild 让通过 Unix socket 连接的容器不受随机容器 hostname 影响。
printf '%s\n' "${XAUTH_DATA}" \
  | sed -e 's/^..../ffff/' \
  | xauth -f "${XAUTH_COOKIE_FILE}" nmerge -
chmod 0600 "${XAUTH_COOKIE_FILE}"
GUI_ARGS+=(
  --env XAUTHORITY=/run/robot-xauthority
  --volume "${XAUTH_COOKIE_FILE}:/run/robot-xauthority:ro"
)

set +e
docker run --rm --interactive --tty \
  --name robot-jazzy-calibration \
  --network host \
  --ipc host \
  --ulimit core=0 \
  --device /dev/stereo_camera:/dev/video0 \
  "${GUI_ARGS[@]}" \
  --volume "${WORKSPACE_DIR}:/workspace" \
  --volume "${CALIBRATION_OUTPUT_DIR}:/tmp" \
  --workdir /workspace \
  robot-jazzy:local \
  bash -lc '
    # ROS 2 setup 脚本会探测尚未定义的环境变量，容器内部不能启用 nounset。
    set -eo pipefail

    MODE="$1"
    SQUARE_SIZE="$2"
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select robot --symlink-install
    source /workspace/install/setup.bash

    ros2 launch robot stereo_camera.launch.py \
      calibration_mode:=true video_device:=/dev/video0 &
    CAMERA_PID=$!
    cleanup() {
      if [[ -n "${LEFT_VIEW_PID:-}" ]]; then
        kill -TERM "${LEFT_VIEW_PID}" >/dev/null 2>&1 || true
      fi
      if [[ -n "${RIGHT_VIEW_PID:-}" ]]; then
        kill -TERM "${RIGHT_VIEW_PID}" >/dev/null 2>&1 || true
      fi
      kill -TERM "${CAMERA_PID}" >/dev/null 2>&1 || true
      for _ in $(seq 1 30); do
        if ! kill -0 "${CAMERA_PID}" >/dev/null 2>&1; then
          break
        fi
        sleep 0.1
      done
      kill -KILL "${CAMERA_PID}" >/dev/null 2>&1 || true
      wait "${CAMERA_PID}" >/dev/null 2>&1 || true
    }
    trap cleanup EXIT INT TERM

    timeout 30 ros2 topic echo /stereo/left/image_raw --once >/dev/null
    timeout 30 ros2 topic echo /stereo/right/image_raw --once >/dev/null

    if [[ "${MODE}" == "preview" ]]; then
      echo "正在打开左右图像窗口；依次遮挡物理镜头，确认话题对应关系。"
      ros2 run image_view image_view --ros-args \
        --remap image:=/stereo/left/image_raw &
      LEFT_VIEW_PID=$!
      ros2 run image_view image_view --ros-args \
        --remap image:=/stereo/right/image_raw &
      RIGHT_VIEW_PID=$!
      wait "${LEFT_VIEW_PID}" "${RIGHT_VIEW_PID}"
      exit 0
    fi

    ros2 run camera_calibration cameracalibrator \
      --size 8x6 \
      --square "${SQUARE_SIZE}" \
      --camera_name usb_camera_01_00_00_640x480 \
      --no-service-check \
      --queue-size 5 \
      --ros-args \
      --remap left:=/stereo/left/image_raw \
      --remap right:=/stereo/right/image_raw \
      --remap left_camera:=/stereo/left \
      --remap right_camera:=/stereo/right
  ' bash "${MODE}" "${SQUARE_SIZE}"
DOCKER_STATUS=$?
set -e
exit "${DOCKER_STATUS}"
