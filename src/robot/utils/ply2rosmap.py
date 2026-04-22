import numpy as np
import cv2
from plyfile import PlyData

# ===================== 配置参数 =====================
PLY_PATH = "/home/shijiahao/Downloads/studyroom.ply"
PGM_PATH = "/home/shijiahao/Downloads/studyroom.pgm"
YAML_PATH = "/home/shijiahao/Downloads/studyroom.yaml"
RESOLUTION = 0.05
HEIGHT_MIN = 0.1   # 过滤地面
HEIGHT_MAX = 1.5   # 过滤天花板
# ====================================================

# 1. 读取点云
plydata = PlyData.read(PLY_PATH)
x = plydata['vertex']['x']
y = plydata['vertex']['y']
z = plydata['vertex']['z']
points = np.column_stack([x, y, z])

# 2. 高度过滤（只留墙壁）
filtered_points = points[(points[:, 2] > HEIGHT_MIN) & (points[:, 2] < HEIGHT_MAX)]
xy = filtered_points[:, :2]

# 3. 原始坐标范围（保留点云原点，不修改）
min_x, min_y = np.min(xy, axis=0)
max_x, max_y = np.max(xy, axis=0)

# 4. 地图尺寸
width = int(np.ceil((max_x - min_x) / RESOLUTION))
height = int(np.ceil((max_y - min_y) / RESOLUTION))

# 5. 空白地图
map_img = 255 * np.ones((height, width), dtype=np.uint8)

# 6. 绘制障碍物
for (x_p, y_p) in xy:
    px = int((x_p - min_x) / RESOLUTION)
    py = int((y_p - min_y) / RESOLUTION)
    if 0 <= px < width and 0 <= py < height:
        map_img[py, px] = 0

# 🔥 核心修复：垂直翻转图像，解决轴对称问题！
map_img = cv2.flip(map_img, 0)

# 7. 轻微膨胀
kernel = np.ones((1, 1), np.uint8)
map_img = cv2.dilate(map_img, kernel, iterations=1)

# 8. 保存
cv2.imwrite(PGM_PATH, map_img)

# 9. YAML：完全保留点云原始原点
yaml_content = f"""image: studyroom.pgm
resolution: {RESOLUTION}
origin: [{min_x:.6f}, {min_y:.6f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

with open(YAML_PATH, "w") as f:
    f.write(yaml_content)

print("✅ 生成成功！轴对称已修复，3D点云与2D地图完美对齐")
print(f"📍 原始原点：({min_x:.3f}, {min_y:.3f})")