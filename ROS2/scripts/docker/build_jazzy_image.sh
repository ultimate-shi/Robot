#!/usr/bin/env bash
# 运行时机：新板首次部署，以及 Dockerfile、ROS 系统包或镜像内 Python 依赖变化后执行。
# 典型命令：bash scripts/docker/build_jazzy_image.sh
# 仅修改工作区 Python 节点、launch、URDF、地图或 YAML 时通常无需重新构建镜像。

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

# 镜像同时支持 amd64 开发机与 RK3588 arm64，实际架构由 Docker 自动选择。
# host 网络使构建容器能够访问宿主机 127.0.0.1 上的代理服务。
BUILD_ARGS=()
if [[ -n "${HTTP_PROXY:-${http_proxy:-}}" ]]; then
  BUILD_ARGS+=(--build-arg "HTTP_PROXY=${HTTP_PROXY:-${http_proxy:-}}")
fi
if [[ -n "${HTTPS_PROXY:-${https_proxy:-}}" ]]; then
  BUILD_ARGS+=(--build-arg "HTTPS_PROXY=${HTTPS_PROXY:-${https_proxy:-}}")
fi
if [[ -n "${NO_PROXY:-${no_proxy:-}}" ]]; then
  BUILD_ARGS+=(--build-arg "NO_PROXY=${NO_PROXY:-${no_proxy:-}}")
fi

docker build \
  --network host \
  "${BUILD_ARGS[@]}" \
  --file "${WORKSPACE_DIR}/docker/Dockerfile.jazzy" \
  --tag robot-jazzy:local \
  "${WORKSPACE_DIR}"
