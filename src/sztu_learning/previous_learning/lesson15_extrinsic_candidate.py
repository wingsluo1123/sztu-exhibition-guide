"""根据2026-11-19读取的静态TF计算候选外参；不发布、不写配置。"""

import numpy as np
from scipy.spatial.transform import Rotation

# 读取方向：IMU原点及坐标轴在雷达坐标系中。
translation = np.array([
    -0.011192456057564526, 0.004586773994111502, 0.01567766096742688,
])
quaternion = [
    0.4946908610094775, -0.5060007528510458,
    -0.49874186085435784, 0.5005004958832802,
]

rotation = Rotation.from_quat(quaternion).as_matrix()
inverse_rotation = rotation.T
inverse_translation = -inverse_rotation @ translation

print('# 候选：雷达在IMU中；尚未验证标定精度及与录包的对应关系')
print('extrinsic_T:', inverse_translation.tolist())
print('extrinsic_R:', inverse_rotation.reshape(9).tolist())

# 验证两次坐标转换互相抵消，仅检查数值求逆，不验证标定。
assert np.allclose(inverse_rotation @ rotation, np.eye(3))
assert np.allclose(inverse_rotation @ translation + inverse_translation, 0)
