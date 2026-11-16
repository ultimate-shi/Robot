"""使用方法：pytest 运行本文件，验证 SegFormer 后处理无需加载真实 NPU 模型。"""

from pathlib import Path
import sys

import cv2
import numpy as np
import pytest


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from rknn_segformer_detector import (  # noqa: E402
    ADE20K_CLASSES, _decode_logits, _get_model, _png_base64, _preprocess)


def test_ade20k_label_table_has_expected_navigation_classes():
    assert len(ADE20K_CLASSES) == 150
    assert ADE20K_CLASSES[3] == 'floor'
    assert ADE20K_CLASSES[28] == 'rug'


def test_preprocess_produces_static_normalized_nchw():
    image = np.full((240, 320, 3), 127, dtype=np.uint8)
    value = _preprocess(image)
    assert value.shape == (1, 3, 512, 512)
    assert value.dtype == np.float32
    assert np.isfinite(value).all()


def test_decode_logits_restores_original_size_and_confidence():
    logits = np.zeros((1, 4, 2, 2), dtype=np.float32)
    logits[:, 3, :, :] = 8.0
    mask, confidence = _decode_logits([logits], (4, 6))
    assert mask.shape == (4, 6)
    assert confidence.shape == (4, 6)
    assert np.all(mask == 3)
    assert np.all(confidence > 250)


def test_png_payload_is_lossless_for_class_ids():
    import base64

    source = np.asarray([[0, 3], [28, 149]], dtype=np.uint8)
    encoded = base64.b64decode(_png_base64(source))
    decoded = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert np.array_equal(decoded, source)


def test_invalid_output_shape_is_rejected():
    with pytest.raises(RuntimeError, match='输出形状无效'):
        _decode_logits([np.zeros((1, 2, 2), dtype=np.float32)], (2, 2))


def test_missing_model_fails_before_importing_runtime(monkeypatch, tmp_path):
    import rknn_segformer_detector as module

    monkeypatch.setattr(module, 'MODEL_PATH', tmp_path / 'missing.rknn')
    monkeypatch.setattr(module, '_MODEL', None)
    with pytest.raises(FileNotFoundError, match='模型不存在'):
        _get_model()
