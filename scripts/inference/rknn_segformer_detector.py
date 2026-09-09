#!/usr/bin/env python3
"""使用方法：由 brain_inference_server.py 动态加载 SegFormer-B0 RKNN 分割插件。

输入为 JPEG 字节，输出与原图同尺寸的 ADE20K 类别图和置信度图 PNG。
"""

import base64
import os
from pathlib import Path
import threading

import cv2
import numpy as np


MODEL_SIZE = 512
ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = Path(os.environ.get(
    'ROBOT_SEGFORMER_MODEL',
    ROOT / 'model/segformer/segformer-b0-ade20k-fp16.rknn'))
MODEL_NAME = 'segformer-b0-ade20k-fp16-rk3588'
TRAVERSABLE_CLASSES = {3: 'floor', 28: 'rug'}
ADE20K_CLASSES = (
    'wall', 'building', 'sky', 'floor', 'tree', 'ceiling', 'road', 'bed',
    'windowpane', 'grass', 'cabinet', 'sidewalk', 'person', 'earth', 'door',
    'table', 'mountain', 'plant', 'curtain', 'chair', 'car', 'water',
    'painting', 'sofa', 'shelf', 'house', 'sea', 'mirror', 'rug', 'field',
    'armchair', 'seat', 'fence', 'desk', 'rock', 'wardrobe', 'lamp',
    'bathtub', 'railing', 'cushion', 'base', 'box', 'column', 'signboard',
    'chest of drawers', 'counter', 'sand', 'sink', 'skyscraper', 'fireplace',
    'refrigerator', 'grandstand', 'path', 'stairs', 'runway', 'case',
    'pool table', 'pillow', 'screen door', 'stairway', 'river', 'bridge',
    'bookcase', 'blind', 'coffee table', 'toilet', 'flower', 'book', 'hill',
    'bench', 'countertop', 'stove', 'palm', 'kitchen island', 'computer',
    'swivel chair', 'boat', 'bar', 'arcade machine', 'hovel', 'bus', 'towel',
    'light', 'truck', 'tower', 'chandelier', 'awning', 'streetlight', 'booth',
    'television receiver', 'airplane', 'dirt track', 'apparel', 'pole', 'land',
    'bannister', 'escalator', 'ottoman', 'bottle', 'buffet', 'poster', 'stage',
    'van', 'ship', 'fountain', 'conveyer belt', 'canopy', 'washer', 'plaything',
    'swimming pool', 'stool', 'barrel', 'basket', 'waterfall', 'tent', 'bag',
    'minibike', 'cradle', 'oven', 'ball', 'food', 'step', 'tank', 'trade name',
    'microwave', 'pot', 'animal', 'bicycle', 'lake', 'dishwasher', 'screen',
    'blanket', 'sculpture', 'hood', 'sconce', 'vase', 'traffic light', 'tray',
    'ashcan', 'fan', 'pier', 'crt screen', 'plate', 'monitor', 'bulletin board',
    'shower', 'radiator', 'glass', 'clock', 'flag',
)
IMAGE_MEAN = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
IMAGE_STD = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)

_MODEL = None
_INIT_LOCK = threading.Lock()


def _get_model():
    """延迟加载 RKNN，网关导入阶段不占用 NPU。"""
    global _MODEL
    if _MODEL is not None:
        return _MODEL
    with _INIT_LOCK:
        if _MODEL is not None:
            return _MODEL
        if not MODEL_PATH.is_file():
            raise FileNotFoundError(f'SegFormer RKNN 模型不存在: {MODEL_PATH}')
        from rknnlite.api import RKNNLite
        model = RKNNLite()
        result = model.load_rknn(str(MODEL_PATH))
        if result != 0:
            raise RuntimeError(f'加载 SegFormer RKNN 失败，错误码 {result}')
        result = model.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        if result != 0:
            model.release()
            raise RuntimeError(f'初始化 SegFormer RKNN Runtime 失败，错误码 {result}')
        _MODEL = model
    return _MODEL


def _preprocess(image):
    """按 Hugging Face SegFormer 图像处理规范生成 NCHW float32。"""
    resized = cv2.resize(image, (MODEL_SIZE, MODEL_SIZE),
                         interpolation=cv2.INTER_LINEAR)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    normalized = (rgb - IMAGE_MEAN) / IMAGE_STD
    return normalized.transpose(2, 0, 1)[np.newaxis, ...]


def _decode_logits(outputs, image_shape):
    """把 [1,C,H,W] logits 解码为原图大小的类别与 0..255 置信度图。"""
    if not outputs:
        raise RuntimeError('SegFormer 推理没有返回输出')
    logits = np.asarray(outputs[0], dtype=np.float32)
    if logits.ndim != 4 or logits.shape[0] != 1:
        raise RuntimeError(f'SegFormer 输出形状无效: {logits.shape}')
    if logits.shape[3] > logits.shape[1] and logits.shape[3] > logits.shape[2]:
        logits = logits.transpose(0, 3, 1, 2)
    if logits.shape[1] < 2:
        raise RuntimeError(f'SegFormer 类别维度无效: {logits.shape}')
    logits = logits[0]
    class_map = np.argmax(logits, axis=0).astype(np.uint8)
    maximum = np.max(logits, axis=0)
    denominator = np.sum(np.exp(logits - maximum[None, ...]), axis=0)
    confidence = np.clip(1.0 / np.maximum(denominator, 1e-6), 0.0, 1.0)
    height, width = image_shape
    class_map = cv2.resize(class_map, (width, height),
                           interpolation=cv2.INTER_NEAREST)
    confidence = cv2.resize(confidence, (width, height),
                            interpolation=cv2.INTER_LINEAR)
    return class_map, np.rint(confidence * 255.0).astype(np.uint8)


def _png_base64(image):
    success, encoded = cv2.imencode('.png', image)
    if not success:
        raise RuntimeError('SegFormer PNG 编码失败')
    return base64.b64encode(encoded).decode('ascii')


def segment(jpeg_bytes):
    """分割一张 JPEG，返回网关可直接序列化的紧凑结果。"""
    encoded = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('无法解码 SegFormer 输入 JPEG')
    outputs = _get_model().inference(inputs=[_preprocess(image)])
    class_map, confidence = _decode_logits(outputs, image.shape[:2])
    return {
        'model': MODEL_NAME,
        'width': int(image.shape[1]),
        'height': int(image.shape[0]),
        'mask_png_base64': _png_base64(class_map),
        'confidence_png_base64': _png_base64(confidence),
        'traversable_classes': {
            str(key): value for key, value in TRAVERSABLE_CLASSES.items()},
        'class_names': {
            str(index): name for index, name in enumerate(ADE20K_CLASSES)},
    }
