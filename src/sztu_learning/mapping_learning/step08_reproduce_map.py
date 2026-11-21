"""电脑离线：在独立目录调用原成功建图实现，并对照固定地图。"""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import yaml

lesson = Path(__file__).resolve().parent
previous = lesson.parent / "previous_learning"
original_map = previous / "maps/room_7x3_20261120_map01"
work = lesson / "reproduction"
result = work / "generated_map"
if work.exists():
    raise SystemExit(f"复现目录已经存在，拒绝覆盖：{work}")

# 复制已确认与原地图记录一致的实现；不在旧目录编译或写日志。
manifest = json.loads((original_map / "manifest.json").read_text())
names = ["build_room_map.py", "build_rviz_map.py", "build_rviz_map_v2.py",
         "map_voxel_rays.cpp", "fastlio_extrinsics_candidate.yaml", "room_map_config.yaml"]
for name in names:
    actual = hashlib.sha256((previous / name).read_bytes()).hexdigest()
    if actual != manifest["source_sha256"][name]:
        raise ValueError(f"原实现与成功记录不一致：{name}")

work.mkdir()
for name in names:
    shutil.copy2(previous / name, work / name)
(work / "runtime_logs").mkdir()
(work / "inputs").mkdir()
# 链接只供读取；成功档案本身不复制、不改写。
(work / "inputs/run").symlink_to(previous / "maps/room_7x3_20261120_run01",
                                 target_is_directory=True)
shutil.copy2(previous / "runtime_logs/room_capture_20261120/room_reference.json",
             work / "inputs/room_reference.json")

config = yaml.safe_load((previous / "room_map_config.yaml").read_text())
# 只改变文件位置，建图数值参数保持原样。
config["source"] = "inputs/run"
config["output"] = "generated_map"
config["up_hint_record"] = "inputs/room_reference.json"
config_path = work / "reproduce_config.yaml"
config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")

subprocess.run([sys.executable, str(work / "build_room_map.py"),
                "--config", str(config_path)], check=True)

# 比较地图含义，不要求包含新路径的清单或记录文件逐字节相同。
original_grid = np.load(original_map / "occupancy.npy", allow_pickle=False)
new_grid = np.load(result / "occupancy.npy", allow_pickle=False)
old_yaml = yaml.safe_load((original_map / "map.yaml").read_text())
new_yaml = yaml.safe_load((result / "map.yaml").read_text())
report = {
    "same_shape": original_grid.shape == new_grid.shape,
    "same_grid": bool(np.array_equal(original_grid, new_grid)),
    "same_map_yaml": old_yaml == new_yaml,
    "original_map": str(original_map),
    "reproduced_map": str(result),
}
if report["same_shape"]:
    report["different_cells"] = int(np.count_nonzero(original_grid != new_grid))
(work / "comparison.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps(report, ensure_ascii=False, indent=2))
if not all(report[key] for key in ("same_shape", "same_grid", "same_map_yaml")):
    raise SystemExit("复现结果存在差异，需检查，不能认定完全复现。")
