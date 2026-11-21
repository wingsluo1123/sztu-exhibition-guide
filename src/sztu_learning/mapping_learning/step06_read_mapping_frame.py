"""电脑离线：读取成功建图档案的第一组点云与body位姿，不修改输入。"""
import csv
from pathlib import Path

import numpy as np

archive = (Path(__file__).resolve().parent.parent
           / "previous_learning/maps/room_7x3_20261120_run01")
with (archive / "frames.csv").open() as file:
    first_row = next(csv.DictReader(file))

with np.load(archive / first_row["file"], allow_pickle=False) as frame:
    xyz = frame["xyz"]
    position = frame["position"]
    quaternion = frame["quaternion_xyzw"]
    print("点云形状：", xyz.shape)
    print("前两个点（camera_init坐标，米）：\n", xyz[:2])
    print("body原点位置（camera_init坐标，米）：", position)
    print("body朝向四元数（x,y,z,w）：", quaternion)
