#!/usr/bin/env python3
"""使用方法：在 x86_64 Linux 运行本脚本导出 SegFormer-B0 ONNX/RKNN。

依赖 torch、transformers、onnx 和 RKNN-Toolkit2 2.3.x。
板端只复制生成的 FP16 RKNN 文件。
"""

import argparse
import hashlib
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description='转换 SegFormer-B0 为 RKNN')
    parser.add_argument(
        '--model-id', default='nvidia/segformer-b0-finetuned-ade-512-512')
    parser.add_argument('--output-directory', default='model/segformer')
    return parser.parse_args()


def main():
    import torch
    from rknn.api import RKNN
    from transformers import SegformerForSemanticSegmentation

    args = parse_args()
    output = Path(args.output_directory).resolve()
    output.mkdir(parents=True, exist_ok=True)
    onnx_path = output / 'segformer-b0-ade20k.onnx'
    rknn_path = output / 'segformer-b0-ade20k-fp16.rknn'
    model = SegformerForSemanticSegmentation.from_pretrained(
        args.model_id).eval()

    class LogitsOnly(torch.nn.Module):
        def __init__(self, wrapped):
            super().__init__()
            self.wrapped = wrapped

        def forward(self, pixel_values):
            return self.wrapped(pixel_values=pixel_values).logits

    sample = torch.zeros((1, 3, 512, 512), dtype=torch.float32)
    torch.onnx.export(
        LogitsOnly(model), sample, str(onnx_path), opset_version=17,
        input_names=['pixel_values'], output_names=['logits'],
        dynamic_axes=None, do_constant_folding=True)

    converter = RKNN(verbose=True)
    try:
        if converter.config(target_platform='rk3588') != 0:
            raise RuntimeError('RKNN config 失败')
        if converter.load_onnx(model=str(onnx_path)) != 0:
            raise RuntimeError('RKNN 加载 ONNX 失败')
        if converter.build(do_quantization=False) != 0:
            raise RuntimeError('RKNN FP16 构建失败')
        if converter.export_rknn(str(rknn_path)) != 0:
            raise RuntimeError('RKNN 导出失败')
    finally:
        converter.release()
    checksum = hashlib.sha256(rknn_path.read_bytes()).hexdigest()
    (output / 'SHA256SUMS').write_text(
        f'{checksum}  {rknn_path.name}\n', encoding='utf-8')
    print(f'已生成：{rknn_path}')


if __name__ == '__main__':
    main()
