#!/usr/bin/env bash
# 使用方法：bash scripts/inference/download_rk3588_models.sh [all|yolo|qwen]。
# 下载能够直接在 RK3588 上加载的预转换模型。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
MODEL_DIR="${REPO_DIR}/model"
TARGET="${1:-all}"

mkdir -p "${MODEL_DIR}/yolo" "${MODEL_DIR}/qwen2.5-vl"

if [[ "${TARGET}" == "all" || "${TARGET}" == "yolo" ]]; then
  curl -fL --retry 5 --continue-at - \
    'https://downloads.mixtile.com/doc-files/yolov8/rk3588/yolov8n.rknn' \
    -o "${MODEL_DIR}/yolo/yolov8n.rknn"
fi

if [[ "${TARGET}" == "all" || "${TARGET}" == "qwen" ]]; then
  echo "Qwen 模型由 Sync 公共分享页提供，需要浏览器执行页面解密。" >&2
  echo "语言模型：https://ln5.sync.com/dl/4450f65a0#5xun8r2j-qeg5z6aw-k33ph6s7-eb7dgfbv" >&2
  echo "视觉模型：https://ln5.sync.com/dl/68063c720#jadpjqg3-wgyca793-dfs8tv3u-qp2sgsnq" >&2
  echo "请将两个文件保存到：${MODEL_DIR}/qwen2.5-vl" >&2
fi
