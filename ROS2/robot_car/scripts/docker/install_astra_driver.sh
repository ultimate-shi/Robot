#!/usr/bin/env bash
# 作用：为 Jazzy ARM64 安装锁定版本的 Astra Pro OpenNI 驱动；用法：在容器内以 root 执行 bash /workspace/scripts/docker/install_astra_driver.sh。
set -euo pipefail
ASTRA_DRIVER_REV=f7e71d9ce806e788cb48d8580aac2c778fba4214
ASTRA_DRIVER_WS=/opt/robot/astra_ws
apt-get update
apt-get install -y --no-install-recommends git libuvc-dev libgflags-dev libgoogle-glog-dev \
  libusb-1.0-0-dev nlohmann-json3-dev ros-jazzy-image-publisher ros-jazzy-depth-image-proc
mkdir -p "${ASTRA_DRIVER_WS}/src"
if [[ ! -d "${ASTRA_DRIVER_WS}/src/ros2_astra_camera/.git" ]]; then
  git init "${ASTRA_DRIVER_WS}/src/ros2_astra_camera"
  git -C "${ASTRA_DRIVER_WS}/src/ros2_astra_camera" remote add origin https://github.com/orbbec/ros2_astra_camera.git
  git -C "${ASTRA_DRIVER_WS}/src/ros2_astra_camera" fetch --depth 1 origin "${ASTRA_DRIVER_REV}"
fi
git -C "${ASTRA_DRIVER_WS}/src/ros2_astra_camera" checkout "${ASTRA_DRIVER_REV}"
# 保留上游许可证，仅修正 Jazzy 移除的头文件和参数回调别名；不改测距/标定算法。
python3 - <<'PY'
from pathlib import Path
root = Path('/opt/robot/astra_ws/src/ros2_astra_camera')
for path in root.rglob('*'):
    if path.suffix not in ('.cpp', '.h', '.hpp'):
        continue
    original = path.read_text()
    text = original
    text = text.replace('cv_bridge/cv_bridge.h>', 'cv_bridge/cv_bridge.hpp>')
    text = text.replace('image_geometry/pinhole_camera_model.h>', 'image_geometry/pinhole_camera_model.hpp>')
    text = text.replace('rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType',
                        'std::function<rcl_interfaces::msg::SetParametersResult(const std::vector<rclcpp::Parameter>&)>')
    if path.name == 'constants.h' and '#include <cstdint>' not in text:
        text = text.replace('#include <cstdlib>', '#include <cstdlib>\n#include <cstdint>')
    if text != original:
        path.write_text(text)
PY
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${ASTRA_DRIVER_WS}"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
printf '%s\n' 'Astra Pro 驱动已安装；运行前 source /opt/robot/astra_ws/install/setup.bash'
