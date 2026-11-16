#!/usr/bin/env bash
# 使用方法：bash scripts/inference/download_rk3588_models.sh [all|yolo|qwen]；直接下载 RK3588 转换模型。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
MODEL_DIR="${REPO_DIR}/model"
TARGET="${1:-all}"
QWEN_DIR="${MODEL_DIR}/qwen2.5-3b-instruct"
QWEN_NAME='Qwen2.5-3B-Instruct-rk3588-w8a8-opt-0-hybrid-ratio-0.5.rkllm'
QWEN_URL="https://huggingface.co/c01zaut/Qwen2.5-3B-Instruct-rk3588-1.1.1/resolve/main/${QWEN_NAME}?download=true"
QWEN_BYTES=3738346748
QWEN_SHA256='35dbbed39a2d3cec79a14fe252cd14801f81842769488b9867a9c42f472c7485'
RUNTIME_URL='https://raw.githubusercontent.com/airockchip/rknn-llm/release-v1.2.1/rkllm-runtime/Linux/librkllm_api/aarch64/librkllmrt.so'
RUNTIME_SHA256='a7e6f87f07bbb08058cad4871cc74e8069a054fe4f6259b43c29a4738b0affdd'

if [[ "${TARGET}" != "all" && "${TARGET}" != "yolo" && "${TARGET}" != "qwen" ]]; then
  echo "用法：$0 [all|yolo|qwen]" >&2
  exit 2
fi
mkdir -p "${MODEL_DIR}/yolo" "${QWEN_DIR}/lib"

if [[ "${TARGET}" == "all" || "${TARGET}" == "yolo" ]]; then
  curl -fL --retry 5 --continue-at - \
    'https://downloads.mixtile.com/doc-files/yolov8/rk3588/yolov8n.rknn' \
    -o "${MODEL_DIR}/yolo/yolov8n.rknn"
fi

if [[ "${TARGET}" == "all" || "${TARGET}" == "qwen" ]]; then
  CURRENT_MODEL_BYTES=0
  if [[ -f "${QWEN_DIR}/${QWEN_NAME}" ]]; then
    CURRENT_MODEL_BYTES="$(stat -c '%s' "${QWEN_DIR}/${QWEN_NAME}")"
  fi
  REMAINING_BYTES=$((QWEN_BYTES - CURRENT_MODEL_BYTES))
  if (( REMAINING_BYTES < 0 )); then
    REMAINING_BYTES="${QWEN_BYTES}"
  fi
  AVAILABLE_STORAGE_KB="$(df -Pk "${QWEN_DIR}" | awk 'NR==2 {print $4}')"
  REQUIRED_STORAGE_KB=$((REMAINING_BYTES / 1024 + 524288))
  if (( AVAILABLE_STORAGE_KB < REQUIRED_STORAGE_KB )); then
    echo "错误：剩余模型下载与校验预留空间不足。" >&2
    exit 1
  fi
  if [[ "${CURRENT_MODEL_BYTES}" == "${QWEN_BYTES}" ]] && \
      printf '%s  %s\n' "${QWEN_SHA256}" \
        "${QWEN_DIR}/${QWEN_NAME}" | sha256sum -c - >/dev/null 2>&1; then
    echo "Qwen 模型已完整下载，跳过。"
  elif (( CURRENT_MODEL_BYTES < QWEN_BYTES )); then
    curl -fL --retry 5 --continue-at - "${QWEN_URL}" \
      -o "${QWEN_DIR}/${QWEN_NAME}"
  else
    curl -fL --retry 5 "${QWEN_URL}" \
      -o "${QWEN_DIR}/${QWEN_NAME}"
  fi
  if ! printf '%s  %s\n' "${RUNTIME_SHA256}" \
      "${QWEN_DIR}/lib/librkllmrt.so" | sha256sum -c - >/dev/null 2>&1; then
    curl -fL --retry 5 "${RUNTIME_URL}" \
      -o "${QWEN_DIR}/lib/librkllmrt.so"
  fi
  if [[ "$(stat -c '%s' "${QWEN_DIR}/${QWEN_NAME}")" != "${QWEN_BYTES}" ]]; then
    echo "错误：Qwen 文件大小与仓库记录不一致。" >&2
    exit 1
  fi
  printf '%s  %s\n' "${QWEN_SHA256}" \
    "${QWEN_DIR}/${QWEN_NAME}" | sha256sum -c -
  printf '%s  %s\n' "${RUNTIME_SHA256}" \
    "${QWEN_DIR}/lib/librkllmrt.so" | sha256sum -c -
  {
    echo "${QWEN_SHA256}  qwen2.5-3b-instruct/${QWEN_NAME}"
    echo "${RUNTIME_SHA256}  qwen2.5-3b-instruct/lib/librkllmrt.so"
  } > "${QWEN_DIR}/SHA256SUMS"
  echo "Qwen SHA-256：${QWEN_SHA256}"
fi
