"""电脑离线：用成功地图的同源变换处理第一帧，不覆盖地图。"""
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

previous = Path(__file__).resolve().parent.parent / "previous_learning"
archive = previous / "maps/room_7x3_20261120_run01"
map_dir = previous / "maps/room_7x3_20261120_map01"

with (archive / "frames.csv").open() as file:
    first_row = next(csv.DictReader(file))
with np.load(archive / first_row["file"], allow_pickle=False) as frame:
    xyz = frame["xyz"].astype(float)
    imu_position = frame["position"].astype(float)
    imu_quaternion = frame["quaternion_xyzw"].astype(float)

# 只能配合同一次建图档案使用，不适用于另一次FAST-LIO运行。
transform = json.loads((map_dir / "transform.json").read_text())
rotation = np.array(transform["rotation_target_from_source"], dtype=float)
translation = np.array(transform["translation_m"], dtype=float)

# 每行一个点：等价于对每个列向量p计算R@p+t。
points_map = xyz @ rotation.T + translation

# 此处body指FAST-LIO采用的IMU参考系，不是雷达或机器人几何中心。
offset = np.array(transform["extrinsic_T_m"], dtype=float)
imu_rotation = Rotation.from_quat(imu_quaternion)
lidar_position = imu_position + imu_rotation.apply(offset)
lidar_position_map = lidar_position @ rotation.T + translation

print("转换前：camera_init中的前三个点（米）")
print(xyz[:3])
print("转换后：learning_map中的前三个点（米）")
print(points_map[:3])
print("转换前后点数：", len(xyz), len(points_map))
print("雷达原点在IMU参考系中的安装偏移（米）：", offset)
print("雷达原点在camera_init中的位置（米）：", lidar_position)
print("雷达原点在learning_map中的位置（米）：", lidar_position_map)
print("没有筛选点，也没有生成或覆盖地图文件。")
