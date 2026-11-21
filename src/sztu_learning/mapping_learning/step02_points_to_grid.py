"""在电脑本地运行：用模拟点练习生成占据栅格，不连接机器人。"""

import math

import numpy as np

# 教学假设：所有点在同一坐标系内，z轴向上，地面z为0。
points = [
    (0.25, 0.15, 0.5),   # 合适高度的障碍物点
    (0.25, 0.15, 1.2),   # 同一平面位置的另一个障碍物点
    (0.25, 0.15, 0.0),   # 地面点：跳过，不清除之前的占用
    (0.45, 0.25, 0.8),   # 另一个位置的障碍物点
]
ground_z = 0.0
min_height = 0.2
max_height = 1.5

origin_x = 0.0
origin_y = 0.0
resolution = 0.1
width = 6   # 列数
height = 4  # 行数，不是点的离地高度

grid = np.full((height, width), -1, dtype=np.int8)

for point_x, point_y, point_z in points:
    point_height = point_z - ground_z
    if not min_height <= point_height <= max_height:
        continue

    column = math.floor((point_x - origin_x) / resolution)
    row = math.floor((point_y - origin_y) / resolution)

    # 范围外的点不写入地图，避免越界或负索引访问。
    if 0 <= row < height and 0 <= column < width:
        grid[row, column] = 100

print("数组按第0行到第3行打印，每行按第0列到第5列打印：")
print(grid)
