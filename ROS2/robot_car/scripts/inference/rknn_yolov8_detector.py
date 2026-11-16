#!/usr/bin/env python3
"""用法：由 brain_inference_server.py 动态加载的 YOLOv8n RKNN 检测插件。"""

import os
from pathlib import Path
import threading

import cv2
import numpy as np


MODEL_SIZE = 640
NMS_THRESHOLD = 0.45
ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = Path(os.environ.get(
    'ROBOT_YOLO_MODEL', ROOT / 'model/yolo/yolov8n.rknn'))

CLASSES = (
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
    'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
    'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
    'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon',
    'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
    'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant',
    'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
    'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush',
)

LABELS_ZH = {
    'person': '人', 'bicycle': '自行车', 'car': '汽车', 'motorcycle': '摩托车',
    'bus': '公交车', 'truck': '卡车', 'boat': '船', 'traffic light': '交通灯',
    'bench': '长椅', 'bird': '鸟', 'cat': '猫', 'dog': '狗', 'backpack': '背包',
    'umbrella': '雨伞', 'handbag': '手提包', 'suitcase': '行李箱',
    'bottle': '瓶子', 'cup': '杯子', 'bowl': '碗', 'banana': '香蕉',
    'apple': '苹果', 'orange': '橙子', 'chair': '椅子', 'couch': '沙发',
    'potted plant': '盆栽', 'bed': '床', 'dining table': '餐桌',
    'toilet': '马桶', 'tv': '电视', 'laptop': '笔记本电脑', 'mouse': '鼠标',
    'remote': '遥控器', 'keyboard': '键盘', 'cell phone': '手机',
    'microwave': '微波炉', 'oven': '烤箱', 'sink': '水槽',
    'refrigerator': '冰箱', 'book': '书', 'clock': '时钟', 'vase': '花瓶',
    'scissors': '剪刀', 'teddy bear': '玩具熊',
}

_MODEL = None
_INIT_LOCK = threading.Lock()


def _get_model():
    """延迟加载模型，导入插件时不会无条件占用 NPU。"""
    global _MODEL
    # RKNNLite2 导入会修改 Python logging 映射，必须晚于 Uvicorn 日志初始化。
    from rknnlite.api import RKNNLite
    if _MODEL is not None:
        return _MODEL
    with _INIT_LOCK:
        if _MODEL is not None:
            return _MODEL
        if not MODEL_PATH.is_file():
            raise FileNotFoundError(f'YOLO RKNN 模型不存在: {MODEL_PATH}')
        model = RKNNLite()
        result = model.load_rknn(str(MODEL_PATH))
        if result != 0:
            raise RuntimeError(f'加载 YOLO RKNN 模型失败，错误码 {result}')
        result = model.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        if result != 0:
            model.release()
            raise RuntimeError(f'初始化 RKNN Runtime 失败，错误码 {result}')
        _MODEL = model
    return _MODEL


def _letterbox(image):
    """等比例缩放并使用黑色填充，与 Rockchip 官方示例保持一致。"""
    height, width = image.shape[:2]
    scale = min(MODEL_SIZE / width, MODEL_SIZE / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    resized = cv2.resize(image, (resized_width, resized_height))
    left = (MODEL_SIZE - resized_width) // 2
    top = (MODEL_SIZE - resized_height) // 2
    canvas = np.zeros((MODEL_SIZE, MODEL_SIZE, 3), dtype=np.uint8)
    canvas[top:top + resized_height, left:left + resized_width] = resized
    return canvas, scale, left, top


def _dfl(position):
    batch, channels, height, width = position.shape
    bins = channels // 4
    values = position.reshape(batch, 4, bins, height, width).astype(np.float32)
    values -= np.max(values, axis=2, keepdims=True)
    weights = np.exp(values)
    weights /= np.sum(weights, axis=2, keepdims=True)
    indexes = np.arange(bins, dtype=np.float32).reshape(1, 1, bins, 1, 1)
    return np.sum(weights * indexes, axis=2)


def _box_process(position):
    grid_height, grid_width = position.shape[2:4]
    column, row = np.meshgrid(np.arange(grid_width), np.arange(grid_height))
    grid = np.concatenate((
        column.reshape(1, 1, grid_height, grid_width),
        row.reshape(1, 1, grid_height, grid_width),
    ), axis=1)
    stride = np.array(
        [MODEL_SIZE // grid_width, MODEL_SIZE // grid_height],
        dtype=np.float32).reshape(1, 2, 1, 1)
    distance = _dfl(position)
    top_left = grid + 0.5 - distance[:, :2]
    bottom_right = grid + 0.5 + distance[:, 2:]
    return np.concatenate((top_left * stride, bottom_right * stride), axis=1)


def _flatten(value):
    return value.transpose(0, 2, 3, 1).reshape(-1, value.shape[1])


def _nms(boxes, scores):
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        index = order[0]
        keep.append(index)
        xx1 = np.maximum(x1[index], x1[order[1:]])
        yy1 = np.maximum(y1[index], y1[order[1:]])
        xx2 = np.minimum(x2[index], x2[order[1:]])
        yy2 = np.minimum(y2[index], y2[order[1:]])
        intersection = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[index] + areas[order[1:]] - intersection
        overlap = intersection / np.maximum(union, 1e-6)
        order = order[np.where(overlap <= NMS_THRESHOLD)[0] + 1]
    return np.asarray(keep, dtype=np.int64)


def _post_process(outputs, minimum_confidence):
    """解析 Rockchip 优化版 YOLOv8 的三尺度输出。"""
    if outputs is None:
        raise RuntimeError('RKNN 推理没有返回输出')
    if len(outputs) < 6 or len(outputs) % 3 != 0:
        raise RuntimeError(f'YOLO 输出数量不符合 Rockchip 优化模型: {len(outputs)}')
    pair_per_branch = len(outputs) // 3
    boxes = []
    probabilities = []
    for branch in range(3):
        offset = branch * pair_per_branch
        boxes.append(_flatten(_box_process(outputs[offset])))
        probabilities.append(_flatten(outputs[offset + 1]))
    boxes = np.concatenate(boxes)
    probabilities = np.concatenate(probabilities)
    class_ids = np.argmax(probabilities, axis=1)
    scores = probabilities[np.arange(probabilities.shape[0]), class_ids]
    selected = scores >= minimum_confidence
    boxes, class_ids, scores = boxes[selected], class_ids[selected], scores[selected]
    kept_boxes, kept_classes, kept_scores = [], [], []
    for class_id in np.unique(class_ids):
        selected = class_ids == class_id
        indexes = _nms(boxes[selected], scores[selected])
        kept_boxes.append(boxes[selected][indexes])
        kept_classes.append(class_ids[selected][indexes])
        kept_scores.append(scores[selected][indexes])
    if not kept_boxes:
        return np.empty((0, 4)), np.empty(0, dtype=int), np.empty(0)
    return (
        np.concatenate(kept_boxes),
        np.concatenate(kept_classes),
        np.concatenate(kept_scores),
    )


def detect(jpeg_bytes, min_confidence):
    """检测一张 JPEG，返回推理网关约定的目标列表。"""
    encoded = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('无法解码输入 JPEG')
    model_input, scale, left, top = _letterbox(image)
    model_input = cv2.cvtColor(model_input, cv2.COLOR_BGR2RGB)
    # RKNNLite2 板端接口要求显式提供批次维度，输入保持 NHWC uint8。
    outputs = _get_model().inference(inputs=[model_input[np.newaxis, ...]])
    boxes, class_ids, scores = _post_process(outputs, float(min_confidence))
    height, width = image.shape[:2]
    results = []
    for box, class_id, score in zip(boxes, class_ids, scores):
        x1 = float(np.clip((box[0] - left) / scale, 0, width - 1))
        y1 = float(np.clip((box[1] - top) / scale, 0, height - 1))
        x2 = float(np.clip((box[2] - left) / scale, 0, width - 1))
        y2 = float(np.clip((box[3] - top) / scale, 0, height - 1))
        if x2 <= x1 or y2 <= y1:
            continue
        class_name = CLASSES[int(class_id)]
        results.append({
            'class_name': class_name,
            'label_zh': LABELS_ZH.get(class_name, class_name),
            'confidence': float(score),
            'bbox': [x1, y1, x2, y2],
        })
    return results
