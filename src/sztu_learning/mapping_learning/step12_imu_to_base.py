"""离线用同一次记录、同一时间的位姿与TF演示换算，不发布消息。"""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

previous = Path(__file__).resolve().parent.parent / "previous_learning"
archive = previous / "maps/localization_20261120_run02"
tf_dir = previous / "runtime_logs/continuous_base_tf_20261120"
metadata = json.loads((tf_dir / "manifest.json").read_text())
assert hashlib.sha256((archive / "manifest.json").read_bytes()).hexdigest() == metadata["archive_manifest_sha256"]
assert hashlib.sha256((tf_dir / "base_transforms.npz").read_bytes()).hexdigest() == metadata["output_sha256"]
with (archive / "frames.csv").open() as file:
    row = next(csv.DictReader(file))
with np.load(archive / row["file"], allow_pickle=False) as frame:
    odom_from_imu = np.eye(4)
    odom_from_imu[:3, :3] = Rotation.from_quat(frame["quaternion_xyzw"]).as_matrix()
    odom_from_imu[:3, 3] = frame["position"]
with np.load(tf_dir / "base_transforms.npz", allow_pickle=False) as saved:
    stamp = np.array([int(row["sec"]), int(row["nanosec"])])
    indexes = np.flatnonzero(np.all(saved["stamps"] == stamp, axis=1))
    if len(indexes) != 1:
        raise ValueError("缺少唯一的同时间TF")
    base_from_imu = saved["base_from_imu"][indexes[0]]

# A_from_B表示：将B坐标转换为A坐标。
odom_from_base = odom_from_imu @ np.linalg.inv(base_from_imu)
if not np.allclose(odom_from_base @ base_from_imu, odom_from_imu):
    raise ValueError("换算后无法恢复IMU位姿")
print("同时间戳：", stamp)
print("IMU在该趟camera_init中的位置（米）：", odom_from_imu[:3, 3])
print("base_link在同一camera_init中的位置（米）：", odom_from_base[:3, 3])
print("往返关系校验通过；未调平、未接入地图或实时导航。")
