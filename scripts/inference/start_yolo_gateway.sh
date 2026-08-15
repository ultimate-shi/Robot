#!/usr/bin/env bash
# 使用方法：bash scripts/inference/start_yolo_gateway.sh。
# 启动 YOLO RKNN 检测插件和机器人宿主机推理网关。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
MODEL_PATH="${ROBOT_YOLO_MODEL:-${REPO_DIR}/model/yolo/yolov8n.rknn}"
DEFAULT_PYTHON="${REPO_DIR}/model/python-venv/bin/python"
if [[ ! -x "${DEFAULT_PYTHON}" ]]; then
  DEFAULT_PYTHON=python3
fi
PYTHON_BIN="${ROBOT_AI_PYTHON:-${DEFAULT_PYTHON}}"

if [[ ! -s "${MODEL_PATH}" ]]; then
  echo "错误：YOLO 模型不存在或为空：${MODEL_PATH}" >&2
  echo "请先运行 scripts/inference/download_rk3588_models.sh yolo" >&2
  exit 1
fi

"${PYTHON_BIN}" -c 'import cv2, numpy, rknnlite.api' 2>/dev/null || {
  echo "错误：缺少 OpenCV、NumPy 或 RKNNLite2。" >&2
  exit 1
}
"${PYTHON_BIN}" -c 'import fastapi, uvicorn' 2>/dev/null || {
  echo "错误：缺少 FastAPI/uvicorn，请安装：python3 -m pip install fastapi uvicorn" >&2
  exit 1
}

export PYTHONPATH="${SCRIPT_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
export ROBOT_YOLO_MODEL="${MODEL_PATH}"
export ROBOT_DETECTOR_PLUGIN='rknn_yolov8_detector:detect'
export ROBOT_INFERENCE_HOST="${ROBOT_INFERENCE_HOST:-127.0.0.1}"
export ROBOT_INFERENCE_PORT="${ROBOT_INFERENCE_PORT:-9100}"

echo "YOLO 模型：${ROBOT_YOLO_MODEL}"
echo "推理网关：http://${ROBOT_INFERENCE_HOST}:${ROBOT_INFERENCE_PORT}"
exec "${PYTHON_BIN}" "${SCRIPT_DIR}/brain_inference_server.py"
