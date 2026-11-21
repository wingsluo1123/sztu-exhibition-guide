"""只读旧点云，统计近似调平后的z分布，不自动认定地面。"""

from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

pcd_path = (Path(__file__).resolve().parent.parent
            / "previous_learning/maps/lesson51_turned_cloud/registered_scans_xyz.pcd")
bin_size = 0.05  # 统计区间宽度5厘米，不是地图分辨率或高度筛选范围
batch_points = 100000
counts = {}
valid_count = 0
invalid_count = 0
candidate_samples = []
candidate_count = 0
context_samples = []

with pcd_path.open("rb") as file:
    header = {}
    for _ in range(64):
        line = file.readline(1024)
        if not line or not line.endswith(b"\n"):
            raise ValueError("PCD头部不完整或过长")
        text = line.decode("ascii").strip()
        if not text or text.startswith("#"):
            continue
        key, value = text.split(maxsplit=1)
        header[key] = value
        if key == "DATA":
            break
    else:
        raise ValueError("未找到数据起点")
    expected = {"FIELDS": "x y z", "SIZE": "4 4 4", "TYPE": "F F F",
                "COUNT": "1 1 1", "HEIGHT": "1", "DATA": "binary"}
    if any(header.get(key) != value for key, value in expected.items()):
        raise ValueError("只支持本课已知来源的小端float32 XYZ文件")
    point_count = int(header["POINTS"])
    if point_count <= 0 or int(header["WIDTH"]) != point_count:
        raise ValueError("点数不一致或为空")
    if pcd_path.stat().st_size - file.tell() != point_count * 12:
        raise ValueError("数据长度不符")

    while True:
        payload = file.read(12 * batch_points)
        if not payload:
            break
        xyz = np.frombuffer(payload, dtype="<f4").reshape(-1, 3)
        valid = np.isfinite(xyz).all(axis=1)
        invalid_count += int((~valid).sum())
        z = xyz[valid, 2].astype(np.float64)
        valid_count += len(z)
        context_samples.append(xyz[valid][::100].copy())

        # 观察窗口仅用于分析地面候选，不是障碍物高度筛选。
        candidate = xyz[valid][(z >= -1.05) & (z < -0.95)]
        candidate_count += len(candidate)
        # 绘图每100个候选点取1个，避免绘制数百万点；统计仍使用全部点。
        candidate_samples.append(candidate[::100, :2].copy())

        # 以z=0为分组基准，负坐标也向下取整；不改变点坐标。
        bins = np.floor(z / bin_size)
        indexes, batch_counts = np.unique(bins, return_counts=True)
        for index, count in zip(indexes, batch_counts):
            key = int(index)
            counts[key] = counts.get(key, 0) + int(count)

output_dir = Path(__file__).resolve().parent / "outputs" / "height_histogram"
output_dir.mkdir(parents=True, exist_ok=True)
csv_path = output_dir / "z_histogram.csv"
with csv_path.open("w", encoding="utf-8") as file:
    file.write("z_lower_m,z_upper_m,point_count\n")
    for index in sorted(counts):
        file.write(f"{index * bin_size:.2f},{(index + 1) * bin_size:.2f},{counts[index]}\n")

print(f"有效点：{valid_count}；非有限坐标点：{invalid_count}")
print("点数最多的8个z区间（左闭右开；不是地面判定）：")
for index in sorted(counts, key=counts.get, reverse=True)[:8]:
    print(f"[{index * bin_size:.2f}, {(index + 1) * bin_size:.2f}) 米：{counts[index]} 点")
print(f"完整统计：{csv_path}")

if candidate_count:
    xy = np.concatenate(candidate_samples, axis=0)
    fig, ax = plt.subplots(figsize=(9, 7), constrained_layout=True)
    ax.scatter(xy[:, 0], xy[:, 1], s=0.5, alpha=0.3)
    ax.set(xlabel="X (m)", ylabel="Y (m)",
           title="Candidate slice: -1.05 <= Z < -0.95 m (not a ground decision)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.25)
    preview_path = output_dir / "candidate_xy.png"
    fig.savefig(preview_path, dpi=150)
    plt.close(fig)
    print(f"候选区间共{candidate_count}点，绘图抽样{len(xy)}点：{preview_path}")
else:
    print("候选区间没有点，不生成预览。")

if valid_count:
    context = np.concatenate(context_samples, axis=0)
    fig, axes = plt.subplots(1, 2, figsize=(12, 6), constrained_layout=True)
    for ax, coordinate, label in zip(axes, (0, 1), ("X", "Y")):
        ax.scatter(context[:, coordinate], context[:, 2], s=0.3, alpha=0.12)
        ax.axhspan(-1.05, -0.95, color="orange", alpha=0.4,
                   label="Candidate Z band (not confirmed ground)")
        ax.set(xlabel=f"{label} (m)", ylabel="Z (m)", title=f"{label}-Z side view")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=7)
    side_path = output_dir / "candidate_side_views.png"
    fig.savefig(side_path, dpi=150)
    plt.close(fig)
    print(f"点云侧视图：{side_path}")
