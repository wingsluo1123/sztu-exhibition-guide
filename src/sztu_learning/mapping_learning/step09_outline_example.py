"""历史姿态下的近似轮廓示例，不写入导航运行配置。"""
import json
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

lesson = Path(__file__).resolve().parent
previous = lesson.parent / "previous_learning"
probe_path = previous / "runtime_logs/online_20261120/input_probe01.json"
text = json.loads(probe_path.read_text())["lookup_latest"]
# 历史探测记录以ROS对象的文本表示保存，限定读取其平移和四元数。
match = re.search(
    r"translation=geometry_msgs.msg.Vector3\(x=([^,]+), y=([^,]+), z=([^\)]+)\), "
    r"rotation=geometry_msgs.msg.Quaternion\(x=([^,]+), y=([^,]+), z=([^,]+), w=([^\)]+)\)", text)
if match is None or "frame_id='base_link'" not in text or "child_frame_id='lidar_imu_chest_front'" not in text:
    raise ValueError("历史记录的结构或坐标系不符合预期")
numbers = np.array([float(value) for value in match.groups()])
imu_position_base = numbers[:3]
imu_rotation_base = Rotation.from_quat(numbers[3:])
transform = json.loads((previous / "maps/room_7x3_20261120_map01/transform.json").read_text())
lidar_offset_imu = np.array(transform["extrinsic_T_m"])
lidar_position_base = imu_position_base + imu_rotation_base.apply(lidar_offset_imu)
dx, dy = lidar_position_base[:2]

measurements = yaml.safe_load((lesson / "robot_outline_measurements.yaml").read_text())
front, rear = measurements["front_distance"], measurements["rear_distance"]
left, right = measurements["left_distance"], measurements["right_distance"]
footprint = [[dx + front, dy + left], [dx + front, dy - right],
             [dx - rear, dy - right], [dx - rear, dy + left]]
report = {
    "status": "historical_pose_example_not_for_deployment",
    "assumptions": ["人工测量参考点近似雷达原点投影", "外形边沿机器人前后左右方向",
                    "历史TF不代表当前测量姿态", "尚未确认覆盖摆臂范围"],
    "tf_source": str(probe_path),
    "lidar_position_in_base_m": lidar_position_base.tolist(),
    "footprint_example_m": np.asarray(footprint).tolist(),
}
output = lesson / "outputs/outline_example.yaml"
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(yaml.safe_dump(report, allow_unicode=True, sort_keys=False))
print("历史姿态下雷达在base_link中的位置（米）：", lidar_position_base)
print("近似轮廓示例，非实机配置：\n", np.round(footprint, 4))
print("保存：", output)
