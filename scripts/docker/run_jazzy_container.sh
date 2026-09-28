#!/usr/bin/env bash
# 运行时机：只需要 ROS 2 Jazzy 容器环境、不希望自动构建或启动 launch 时执行。
# 典型命令：bash scripts/docker/run_jazzy_container.sh。
# 脚本会在后台启动容器；之后使用 docker exec -it robot-jazzy bash 进入。

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
CONTAINER_NAME="robot-jazzy"
IMAGE_NAME="robot-jazzy:local"

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "未找到 ${IMAGE_NAME}，请先运行 bash scripts/docker/build_jazzy_image.sh" >&2
  exit 1
fi

# 旧 hw_ctrl 会持续读写同一组控制板；实机 ROS 启动前必须停止，避免响应串包和双重控制。
if systemctl is-active --quiet vctrl.service 2>/dev/null; then
  echo "警告：vctrl.service 正在运行，会与 ROS 2 争用 DMC0、DMC1 和 SE2。" >&2
  echo "实机启动前请执行：sudo systemctl stop vctrl.service" >&2
fi

if docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  if [[ "$(docker inspect --format '{{.State.Running}}' "${CONTAINER_NAME}")" == "true" ]]; then
    echo "容器 ${CONTAINER_NAME} 已经在运行。"
    HOST_CHASSIS_COUNT=0
    MISSING_CHASSIS_DEVICES=()
    if ! docker exec "${CONTAINER_NAME}" test -d /host-dev; then
      echo "当前容器尚未启用动态 /host-dev 设备映射，请停止后用本脚本重建。" >&2
    fi
    for index in {0..8}; do
      device="/dev/ttyACM${index}"
      if [[ -e "${device}" ]]; then
        HOST_CHASSIS_COUNT=$((HOST_CHASSIS_COUNT + 1))
        echo "底盘串口：${device} -> 容器 ${device}（动态源 /host-dev/ttyACM${index}）"
        if ! docker exec "${CONTAINER_NAME}" test -e "/host-dev/ttyACM${index}" ||
          ! docker exec "${CONTAINER_NAME}" test -e "${device}"; then
          MISSING_CHASSIS_DEVICES+=("${device}")
        fi
      fi
    done
    if [[ ${#MISSING_CHASSIS_DEVICES[@]} -gt 0 ]]; then
      echo "检测到容器未映射当前底盘设备：${MISSING_CHASSIS_DEVICES[*]}" >&2
      echo "Docker 不能给运行中的容器追加设备；请先执行 docker stop ${CONTAINER_NAME}，再重新运行本脚本。" >&2
    elif [[ ${HOST_CHASSIS_COUNT} -gt 0 ]]; then
      echo "已确认容器可见 ${HOST_CHASSIS_COUNT} 个底盘 ttyACM 设备。"
    else
      echo "提示：宿主机当前未发现 /dev/ttyACM0..8，请检查 USB 枚举或设备类型。" >&2
    fi
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
  echo "双目相机：/dev/stereo_camera ($(readlink -f /dev/stereo_camera)) -> 容器 /dev/video0"
else
  echo "提示：未找到 /dev/stereo_camera，本次容器不映射真实双目相机。" >&2
fi

IMU_DEVICE_ARGS=()
IMU_HOST_DEVICE="${GY95T_DEVICE:-}"
if [[ -n "${IMU_HOST_DEVICE}" && ! -e "${IMU_HOST_DEVICE}" ]]; then
  echo "GY95T_DEVICE 指定的设备不存在：${IMU_HOST_DEVICE}" >&2
  exit 1
fi
if [[ -z "${IMU_HOST_DEVICE}" ]]; then
  # 优先稳定别名；未安装 udev 规则时兼容当前 CH340 by-id 和单设备 ttyUSB0。
  for candidate in \
    /dev/gy95t \
    /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
    /dev/ttyUSB0; do
    if [[ -e "${candidate}" ]]; then
      IMU_HOST_DEVICE="${candidate}"
      break
    fi
  done
fi
if [[ -n "${IMU_HOST_DEVICE}" ]]; then
  IMU_DEVICE_ARGS=(--device "${IMU_HOST_DEVICE}:/dev/gy95t")
  echo "GY95T：${IMU_HOST_DEVICE} ($(readlink -f "${IMU_HOST_DEVICE}")) -> 容器 /dev/gy95t"
else
  echo "提示：未找到 GY95T；可用 GY95T_DEVICE=/dev/ttyUSBx 显式指定。" >&2
fi

# 动态挂载宿主 /dev，并只放行 USB CDC ACM 主设备号 166，避免重枚举后绑定旧设备节点。
CHASSIS_DEVICE_ARGS=(
  --device-cgroup-rule 'c 166:* rmw'
  --mount type=bind,source=/dev,target=/host-dev,readonly
)
CHASSIS_DEVICE_COUNT=0
for index in {0..8}; do
  device="/dev/ttyACM${index}"
  if [[ -e "${device}" ]]; then
    CHASSIS_DEVICE_COUNT=$((CHASSIS_DEVICE_COUNT + 1))
    echo "底盘串口：${device} -> 容器 ${device}（动态源 /host-dev/ttyACM${index}）"
  fi
done
if [[ ${CHASSIS_DEVICE_COUNT} -gt 0 ]]; then
  echo "已发现 ${CHASSIS_DEVICE_COUNT} 个底盘 ttyACM 设备；重枚举后映射仍保持有效。"
else
  echo "提示：当前未找到 ttyACM 底盘控制板；容器仍允许之后热插并自动映射。" >&2
fi

# 容器仅提供 Jazzy 环境，不执行 colcon build，也不启动任何 ROS launch。
docker run --rm --detach --interactive --tty \
  --name "${CONTAINER_NAME}" \
  --network host \
  --ipc host \
  --ulimit core=0 \
  "${CAMERA_DEVICE_ARGS[@]}" \
  "${IMU_DEVICE_ARGS[@]}" \
  "${CHASSIS_DEVICE_ARGS[@]}" \
  --volume "${WORKSPACE_DIR}:/workspace" \
  --workdir /workspace \
  "${IMAGE_NAME}" \
  bash -lc '
    set -e

    # 固定容器内路径指向宿主动态设备目录，USB 重枚举后无需重建容器。
    for index in 0 1 2 3 4 5 6 7 8; do
      ln -sfn "/host-dev/ttyACM${index}" "/dev/ttyACM${index}"
    done

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

if [[ ${#CAMERA_DEVICE_ARGS[@]} -gt 0 ]]; then
  echo "容器相机设备：/dev/video0"
fi
if [[ ${#IMU_DEVICE_ARGS[@]} -gt 0 ]]; then
  echo "容器 IMU 设备：/dev/gy95t"
fi
echo "进入容器：docker exec -it ${CONTAINER_NAME} bash"
echo "停止容器：docker stop ${CONTAINER_NAME}"
