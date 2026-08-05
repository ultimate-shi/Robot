#!/usr/bin/env bash
# 在标定前固定相机曝光和白平衡，避免自动控制造成同一姿态亮度漂移。
# 可覆盖示例：STEREO_EXPOSURE=200 STEREO_WHITE_BALANCE=5000 bash scripts/configure_stereo_camera.sh
# 一次性自动调节：STEREO_AUTO_TUNE=1 bash scripts/configure_stereo_camera.sh

set -euo pipefail

CAMERA_DEVICE="${STEREO_CAMERA_DEVICE:-/dev/stereo_camera}"
EXPOSURE="${STEREO_EXPOSURE:-166}"
WHITE_BALANCE="${STEREO_WHITE_BALANCE:-4600}"
AUTO_TUNE="${STEREO_AUTO_TUNE:-0}"
AUTO_TUNE_SECONDS="${STEREO_AUTO_TUNE_SECONDS:-3}"

if [[ ! -e "${CAMERA_DEVICE}" ]]; then
  echo "未找到相机设备：${CAMERA_DEVICE}" >&2
  exit 1
fi

if [[ "${AUTO_TUNE}" != "0" && "${AUTO_TUNE}" != "1" ]]; then
  echo "STEREO_AUTO_TUNE 只能是 0 或 1。" >&2
  exit 2
fi

if [[ ! "${AUTO_TUNE_SECONDS}" =~ ^[1-9][0-9]*$ ]] || (( AUTO_TUNE_SECONDS > 15 )); then
  echo "STEREO_AUTO_TUNE_SECONDS 必须是 1～15 的整数秒。" >&2
  exit 2
fi

if [[ "${AUTO_TUNE}" == "1" ]]; then
  FRAME_COUNT=$((AUTO_TUNE_SECONDS * 20))
  echo "正在自动调整曝光和白平衡 ${AUTO_TUNE_SECONDS} 秒；请让棋盘格位于正常标定位置并保持静止。"

  v4l2-ctl -d "${CAMERA_DEVICE}" \
    --set-ctrl=brightness=0 \
    --set-ctrl=auto_exposure=3 \
    --set-ctrl=white_balance_automatic=1

  # 自动控制只有在相机持续出流时才会收敛，因此先按正式标定模式采集一小段但不保存图像。
  v4l2-ctl -d "${CAMERA_DEVICE}" \
    --set-fmt-video=width=1280,height=480,pixelformat=MJPG \
    --set-parm=20 \
    --stream-mmap=4 \
    --stream-count="${FRAME_COUNT}" \
    --stream-to=/dev/null \
    --stream-poll >/dev/null

  EXPOSURE="$(v4l2-ctl -d "${CAMERA_DEVICE}" \
    --get-ctrl=exposure_time_absolute | awk -F': ' 'NR == 1 {print $2}')"
  WHITE_BALANCE="$(v4l2-ctl -d "${CAMERA_DEVICE}" \
    --get-ctrl=white_balance_temperature | awk -F': ' 'NR == 1 {print $2}')"

  echo "自动调整结果：曝光=${EXPOSURE}，白平衡=${WHITE_BALANCE} K；现在切换为手动锁定。"
fi

if [[ ! "${EXPOSURE}" =~ ^[0-9]+$ ]] || (( EXPOSURE < 3 || EXPOSURE > 2047 )); then
  echo "STEREO_EXPOSURE 必须是 3～2047 的整数。" >&2
  exit 2
fi

if [[ ! "${WHITE_BALANCE}" =~ ^[0-9]+$ ]] || (( WHITE_BALANCE < 2800 || WHITE_BALANCE > 6500 )); then
  echo "STEREO_WHITE_BALANCE 必须是 2800～6500 K 的整数。" >&2
  exit 2
fi

v4l2-ctl -d "${CAMERA_DEVICE}" \
  --set-ctrl=brightness=0 \
  --set-ctrl=auto_exposure=1 \
  --set-ctrl="exposure_time_absolute=${EXPOSURE}" \
  --set-ctrl=white_balance_automatic=0 \
  --set-ctrl="white_balance_temperature=${WHITE_BALANCE}"

echo "相机控制已固定：亮度=0，曝光=${EXPOSURE}，白平衡=${WHITE_BALANCE} K"
