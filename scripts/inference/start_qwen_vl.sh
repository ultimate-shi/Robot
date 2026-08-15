#!/usr/bin/env bash
# 使用方法：bash scripts/inference/start_qwen_vl.sh /绝对路径/图片.jpg。
# 使用 RK3588 NPU 启动 Qwen2.5-VL-3B 的交互式图像问答程序。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
QWEN_DIR="${REPO_DIR}/model/qwen2.5-vl"
VISION_MODEL="${ROBOT_QWEN_VISION_MODEL:-${QWEN_DIR}/qwen2.5-vl-3b_vision_rk3588.rknn}"
LLM_MODEL="${ROBOT_QWEN_LLM_MODEL:-${QWEN_DIR}/qwen2.5-vl-3b-instruct_w8a8_rk3588.rkllm}"
EXECUTABLE="${ROBOT_QWEN_EXECUTABLE:-${QWEN_DIR}/bin/VLM_NPU}"
IMAGE_PATH="${1:-}"

if [[ -z "${IMAGE_PATH}" || ! -f "${IMAGE_PATH}" ]]; then
  echo "用法：$0 /绝对路径/图片.jpg [最大新生成数] [上下文长度]" >&2
  exit 2
fi
for required in "${VISION_MODEL}" "${LLM_MODEL}" "${EXECUTABLE}"; do
  if [[ ! -s "${required}" ]]; then
    echo "错误：Qwen 运行文件不存在或为空：${required}" >&2
    exit 1
  fi
done

export LD_LIBRARY_PATH="${QWEN_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export RKLLM_LOG_LEVEL="${RKLLM_LOG_LEVEL:-1}"

echo "启动后输入问题时必须包含一次 <image>，输入 exit 退出。"
exec "${EXECUTABLE}" "${IMAGE_PATH}" "${VISION_MODEL}" "${LLM_MODEL}" \
  "${2:-256}" "${3:-4096}"
