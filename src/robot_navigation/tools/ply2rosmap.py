#!/usr/bin/env python3
"""使用方法：python3 ply2rosmap.py input.ply output_prefix --resolution 0.05 生成 PGM/YAML。"""

import argparse
import os

import cv2
import numpy as np
from plyfile import PlyData


def convert(ply_path, output_prefix, resolution, height_min, height_max):
    """把指定高度范围的 PLY 点投影为 Nav2 占据栅格。"""
    plydata = PlyData.read(ply_path)
    points = np.column_stack([
        plydata['vertex']['x'], plydata['vertex']['y'],
        plydata['vertex']['z']])
    points = points[(points[:, 2] > height_min) & (points[:, 2] < height_max)]
    if not len(points):
        raise ValueError('指定高度范围内没有点')
    xy = points[:, :2]
    min_x, min_y = np.min(xy, axis=0)
    max_x, max_y = np.max(xy, axis=0)
    width = max(1, int(np.ceil((max_x - min_x) / resolution)))
    height = max(1, int(np.ceil((max_y - min_y) / resolution)))
    image = np.full((height, width), 254, dtype=np.uint8)
    cols = np.clip(((xy[:, 0] - min_x) / resolution).astype(int), 0, width - 1)
    rows = np.clip(((xy[:, 1] - min_y) / resolution).astype(int), 0, height - 1)
    image[height - 1 - rows, cols] = 0
    pgm_path = output_prefix + '.pgm'
    yaml_path = output_prefix + '.yaml'
    os.makedirs(os.path.dirname(os.path.abspath(output_prefix)), exist_ok=True)
    if not cv2.imwrite(pgm_path, image):
        raise OSError(f'无法写入 {pgm_path}')
    with open(yaml_path, 'w', encoding='utf-8') as stream:
        stream.write(
            f'image: {os.path.basename(pgm_path)}\n'
            f'resolution: {resolution}\n'
            f'origin: [{min_x}, {min_y}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def main():
    parser = argparse.ArgumentParser(description='PLY 转 Nav2 PGM/YAML')
    parser.add_argument('ply_path')
    parser.add_argument('output_prefix')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--height-min', type=float, default=0.1)
    parser.add_argument('--height-max', type=float, default=1.5)
    args = parser.parse_args()
    convert(args.ply_path, args.output_prefix, args.resolution,
            args.height_min, args.height_max)


if __name__ == '__main__':
    main()
