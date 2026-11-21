"""模拟点自动确定栅格范围；在电脑本地运行，不需要ROS。"""

import math
from pathlib import Path

import numpy as np

# 教学假设：同一固定坐标系、z轴向上、地面z=0；参数不用于真机。
points = [
    (0.25, 0.15, 0.5),
    (0.25, 0.15, 1.2),
    (0.25, 0.15, 0.0),
    (0.45, 0.25, 0.8),
]
ground_z = 0.0
min_height = 0.2
max_height = 1.5
resolution = 0.1

# 先保存筛选结果，地图范围和占用标记使用同一批点。
selected_points = []
for point_x, point_y, point_z in points:
    point_height = point_z - ground_z
    if min_height <= point_height <= max_height:
        selected_points.append((point_x, point_y, point_z))

if not selected_points:
    raise SystemExit("没有符合高度条件的点，停止生成地图；不能据此认定场地空闲。")

origin_x = min(point[0] for point in selected_points)
origin_y = min(point[1] for point in selected_points)
max_x = max(point[0] for point in selected_points)
max_y = max(point[1] for point in selected_points)

# +1容纳最大坐标所在的格子，编号与后面写入时采用相同算法。
width = math.floor((max_x - origin_x) / resolution) + 1
height = math.floor((max_y - origin_y) / resolution) + 1
grid = np.full((height, width), -1, dtype=np.int8)

for point_x, point_y, point_z in selected_points:
    column = math.floor((point_x - origin_x) / resolution)
    row = math.floor((point_y - origin_y) / resolution)
    if 0 <= row < height and 0 <= column < width:
        grid[row, column] = 100

print(f"栅格左下角：({origin_x}, {origin_y}) 米")
print(f"地图大小：{height} 行、{width} 列；分辨率：{resolution} 米/格")
print("数组从第0行开始打印（不是地图图片）：")
print(grid)

# 图片与地图数组分开保存；翻转只适配图片从上到下的行顺序。
image = np.full(grid.shape, 205, dtype=np.uint8)
image[grid == 100] = 0
image[grid == 0] = 254
image = np.flipud(image)

# 无论从哪个终端目录启动，都写到本脚本旁边的固定教学输出目录。
output_dir = Path(__file__).resolve().parent / "outputs" / "simulated_map"
output_dir.mkdir(parents=True, exist_ok=True)
pgm_path = output_dir / "map.pgm"
yaml_path = output_dir / "map.yaml"

with pgm_path.open("w", encoding="ascii") as file:
    file.write("P2\n")
    file.write(f"{width} {height}\n")
    file.write("255\n")
    np.savetxt(file, image, fmt="%d")

# 使用本次生成栅格的实际参数，不另外填写另一套位置和尺寸。
yaml_path.write_text(
    f"image: map.pgm\n"
    f"mode: trinary\n"
    f"resolution: {resolution}\n"
    f"origin: [{origin_x}, {origin_y}, 0.0]\n"
    f"negate: 0\n"
    f"occupied_thresh: 0.65\n"
    f"free_thresh: 0.196\n",
    encoding="utf-8",
)
print(f"已保存图片：{pgm_path}")
print(f"已保存配置：{yaml_path}")
