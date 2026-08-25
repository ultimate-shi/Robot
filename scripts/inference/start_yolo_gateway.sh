#!/usr/bin/env bash
# 使用方法：bash scripts/inference/start_yolo_gateway.sh；常驻启动 YOLO、纯文本 Qwen 和 9100 网关。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
MODEL_PATH="${ROBOT_YOLO_MODEL:-${REPO_DIR}/model/yolo/yolov8n.rknn}"
QWEN_DIR="${ROBOT_QWEN_DIR:-${REPO_DIR}/model/qwen2.5-3b-instruct}"
QWEN_NAME='Qwen2.5-3B-Instruct-rk3588-w8a8-opt-0-hybrid-ratio-0.5.rkllm'
QWEN_BYTES=3738346748
QWEN_SHA256='35dbbed39a2d3cec79a14fe252cd14801f81842769488b9867a9c42f472c7485'
RUNTIME_SHA256='a7e6f87f07bbb08058cad4871cc74e8069a054fe4f6259b43c29a4738b0affdd'
QWEN_MODEL="${ROBOT_LLM_MODEL_PATH:-${ROBOT_QWEN_LLM_MODEL:-${QWEN_DIR}/${QWEN_NAME}}}"
QWEN_RUNTIME="${ROBOT_LLM_RUNTIME_PATH:-${ROBOT_QWEN_RUNTIME_DIR:-${QWEN_DIR}/lib}/librkllmrt.so}"
LLM_HOST="${ROBOT_LLM_HOST:-127.0.0.1}"
LLM_PORT="${ROBOT_LLM_PORT:-9101}"
DEFAULT_PYTHON="${REPO_DIR}/model/python-venv/bin/python"
if [[ ! -x "${DEFAULT_PYTHON}" ]]; then
  DEFAULT_PYTHON=python3
fi
PYTHON_BIN="${ROBOT_AI_PYTHON:-${DEFAULT_PYTHON}}"
LLM_PID=""

cleanup() {
  if [[ -n "${LLM_PID}" ]] && kill -0 "${LLM_PID}" 2>/dev/null; then
    kill "${LLM_PID}" 2>/dev/null || true
    wait "${LLM_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ ! -s "${MODEL_PATH}" ]]; then
  echo "错误：YOLO 模型不存在或为空：${MODEL_PATH}" >&2
  echo "请先运行 scripts/inference/download_rk3588_models.sh yolo" >&2
  exit 1
fi
if [[ ! -r /sys/module/rknpu/version ]]; then
  echo "错误：未发现可读的 RKNN NPU 驱动版本。" >&2
  exit 1
fi
RKNPU_DRIVER_VERSION="$(< /sys/module/rknpu/version)"
if ! compgen -G '/dev/dri/renderD*' >/dev/null; then
  echo "错误：未发现 NPU 所需的 DRI render 设备节点。" >&2
  exit 1
fi
echo "RKNN NPU 驱动：${RKNPU_DRIVER_VERSION}"

"${PYTHON_BIN}" -c 'import cv2, numpy, rknnlite.api' 2>/dev/null || {
  echo "错误：缺少 OpenCV、NumPy 或 RKNNLite2。" >&2
  exit 1
}
"${PYTHON_BIN}" -c 'import fastapi, uvicorn' 2>/dev/null || {
  echo "错误：缺少 FastAPI/uvicorn。" >&2
  exit 1
}

if [[ -z "${ROBOT_LLM_ENDPOINT:-}" && -z "${ROBOT_VLM_ENDPOINT:-}" ]]; then
  if [[ ! -s "${QWEN_MODEL}" || ! -s "${QWEN_RUNTIME}" ]]; then
    echo "错误：纯文本 Qwen 模型或 RKLLM Runtime 不完整。" >&2
    echo "请先运行 scripts/inference/download_rk3588_models.sh qwen" >&2
    exit 1
  fi
  MODEL_BYTES="$(stat -c '%s' "${QWEN_MODEL}")"
  if (( MODEL_BYTES < 3000000000 )); then
    echo "错误：Qwen 模型文件过小，可能尚未下载完成：${MODEL_BYTES} bytes" >&2
    exit 1
  fi
  if [[ "$(basename -- "${QWEN_MODEL}")" == "${QWEN_NAME}" ]]; then
    if [[ "${MODEL_BYTES}" != "${QWEN_BYTES}" ]]; then
      echo "错误：Qwen 模型大小校验失败：${MODEL_BYTES} bytes" >&2
      exit 1
    fi
    printf '%s  %s\n' "${QWEN_SHA256}" "${QWEN_MODEL}" | sha256sum -c -
  fi
  printf '%s  %s\n' "${RUNTIME_SHA256}" "${QWEN_RUNTIME}" | sha256sum -c -
  AVAILABLE_MEMORY_KB="$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)"
  AVAILABLE_STORAGE_KB="$(df -Pk "${QWEN_DIR}" | awk 'NR==2 {print $4}')"
  if (( AVAILABLE_MEMORY_KB < 524288 )); then
    echo "错误：可用内存不足 512MiB，不能安全加载 Qwen。" >&2
    exit 1
  fi
  if (( AVAILABLE_STORAGE_KB < 524288 )); then
    echo "错误：模型分区可用空间不足 512MiB。" >&2
    exit 1
  fi
  echo "可用内存：$((AVAILABLE_MEMORY_KB / 1024))MiB；模型分区可用：$((AVAILABLE_STORAGE_KB / 1024))MiB"
  echo "正在常驻加载 Qwen2.5-3B，请稍候……"
  "${PYTHON_BIN}" "${SCRIPT_DIR}/rkllm_text_server.py" \
    --model "${QWEN_MODEL}" --runtime "${QWEN_RUNTIME}" \
    --host "${LLM_HOST}" --port "${LLM_PORT}" \
    --context-length "${ROBOT_LLM_CONTEXT_LENGTH:-4096}" \
    --max-new-tokens "${ROBOT_LLM_MAX_TOKENS:-96}" &
  LLM_PID=$!
  for _attempt in $(seq 1 90); do
    if ! kill -0 "${LLM_PID}" 2>/dev/null; then
      echo "错误：Qwen 常驻进程在初始化期间退出。" >&2
      exit 1
    fi
    if "${PYTHON_BIN}" -c \
      "from urllib.request import urlopen; urlopen('http://${LLM_HOST}:${LLM_PORT}/health', timeout=1).read()" \
      2>/dev/null; then
      break
    fi
    sleep 1
  done
  if ! "${PYTHON_BIN}" -c \
    "from urllib.request import urlopen; urlopen('http://${LLM_HOST}:${LLM_PORT}/health', timeout=1).read()" \
    2>/dev/null; then
    echo "错误：Qwen 在 90 秒内未完成初始化。" >&2
    exit 1
  fi
  export ROBOT_LLM_ENDPOINT="http://${LLM_HOST}:${LLM_PORT}"
fi

export PYTHONPATH="${SCRIPT_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
export ROBOT_YOLO_MODEL="${MODEL_PATH}"
export ROBOT_DETECTOR_PLUGIN='rknn_yolov8_detector:detect'
export ROBOT_INFERENCE_HOST="${ROBOT_INFERENCE_HOST:-127.0.0.1}"
export ROBOT_INFERENCE_PORT="${ROBOT_INFERENCE_PORT:-9100}"
export ROBOT_LLM_MODEL="${ROBOT_LLM_MODEL:-qwen2.5-3b-instruct-w8a8-rk3588}"
export ROBOT_LLM_MAX_TOKENS="${ROBOT_LLM_MAX_TOKENS:-96}"

echo "YOLO 模型：${ROBOT_YOLO_MODEL}"
echo "Qwen 文本服务：${ROBOT_LLM_ENDPOINT:-${ROBOT_VLM_ENDPOINT}}"
echo "推理网关：http://${ROBOT_INFERENCE_HOST}:${ROBOT_INFERENCE_PORT}"
"${PYTHON_BIN}" "${SCRIPT_DIR}/brain_inference_server.py"
