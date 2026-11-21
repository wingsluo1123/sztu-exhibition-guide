"""离线演示轮廓旋转和平移；假设位姿，不表示真实机器人或可通行性。"""
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

lesson = Path(__file__).resolve().parent
map_dir = lesson / "reproduction/generated_map"
config = yaml.safe_load((map_dir / "map.yaml").read_text())
outline = yaml.safe_load((lesson / "outputs/outline_example.yaml").read_text())
corners = np.array(outline["footprint_example_m"])
grid = np.load(map_dir / "occupancy.npy", allow_pickle=False)
resolution = config["resolution"]
ox, oy, _ = config["origin"]
extent = [ox, ox + grid.shape[1] * resolution,
          oy, oy + grid.shape[0] * resolution]

# 为看清姿态变换，故意在同一假设位置分别画0度与90度；不是实时位置。
position = np.array([0.0, 0.0])
fig, axes = plt.subplots(1, 2, figsize=(11, 5), constrained_layout=True)
for ax, yaw in zip(axes, [0.0, np.pi / 2]):
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)],
                         [np.sin(yaw), np.cos(yaw)]])
    corners_map = corners @ rotation.T + position
    pixels = np.full(grid.shape, 205, dtype=np.uint8)
    pixels[grid == 0] = 254
    pixels[grid == 100] = 0
    # imshow支持origin=lower，直接使用地图行顺序，无需翻转。
    ax.imshow(pixels, cmap="gray", vmin=0, vmax=255,
              origin="lower", extent=extent, interpolation="nearest")
    ax.add_patch(Polygon(corners_map, fill=False, edgecolor="red", linewidth=2))
    ax.plot(*position, "bo", markersize=4)
    ax.arrow(*position, .35 * np.cos(yaw), .35 * np.sin(yaw),
             width=.008, color="blue", length_includes_head=True)
    ax.set(xlim=(-1, 1), ylim=(-1, 1), xlabel="Map X (m)", ylabel="Map Y (m)",
           title=f"Assumed pose (0,0), yaw={np.degrees(yaw):.0f} deg")
    ax.set_aspect("equal")
fig.suptitle("Outline transform example only; not a clearance or live-pose check")
output = lesson / "outputs/outline_on_map.png"
fig.savefig(output, dpi=150)
plt.close(fig)
print(output)
