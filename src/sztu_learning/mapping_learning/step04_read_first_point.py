"""电脑离线练习：只读取旧课程PCD的第一个点，不修改源文件。"""

from pathlib import Path

import numpy as np

pcd_path = (
    Path(__file__).resolve().parent.parent
    / "previous_learning/maps/lesson51_turned_cloud/registered_scans_xyz.pcd"
)

with pcd_path.open("rb") as file:
    header = {}
    # 限制头部读取范围，避免错误文件导致无限读取。
    for _ in range(64):
        line = file.readline(1024)
        if not line or not line.endswith(b"\n"):
            raise ValueError("PCD文件头不完整或单行过长")
        text = line.decode("ascii").strip()
        if not text or text.startswith("#"):
            continue
        key, value = text.split(maxsplit=1)
        header[key] = value
        if key == "DATA":
            break
    else:
        raise ValueError("未找到PCD数据起点")

    # 仅支持这一份已知来源、以小端float32写出的XYZ文件。
    expected = {
        "FIELDS": "x y z", "SIZE": "4 4 4", "TYPE": "F F F",
        "COUNT": "1 1 1", "HEIGHT": "1", "DATA": "binary",
    }
    for key, value in expected.items():
        if header.get(key) != value:
            raise ValueError(f"不支持的PCD结构：{key}={header.get(key)}")
    count = int(header["POINTS"])
    if count <= 0 or int(header["WIDTH"]) != count:
        raise ValueError("点数不一致或没有点")
    if pcd_path.stat().st_size - file.tell() != count * 12:
        raise ValueError("点数据长度与文件头不符")

    payload = file.read(12)
    if len(payload) != 12:
        raise ValueError("第一个点的数据不完整")
    xyz = np.frombuffer(payload, dtype="<f4").reshape(-1, 3)
    if not np.isfinite(xyz).all():
        raise ValueError("第一个点包含无效数值")

print(f"读取文件：{pcd_path}")
print(f"文件共有 {count} 个点，本次只读取第一个点：")
print(xyz)
print("这是文件中的坐标，不直接等于离地高度。")
