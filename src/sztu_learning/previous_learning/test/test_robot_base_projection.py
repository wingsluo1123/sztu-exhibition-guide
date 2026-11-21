"""Robot forward and lever arm must follow dynamic base-to-IMU kinematics."""
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_base_projection import project_base


def test_rotated_imu_y_is_left_but_base_forward_is_x():
    transform = np.eye(4)
    transform[:3, 3] = [.12, .03, .4]
    result = project_base([0, 0, 0], np.eye(3), np.zeros(3), [.12, .03, .4], [0, 0, 0, 1], transform)
    np.testing.assert_allclose(result, [0, 0, 0], atol=1e-12)
    assert abs(np.pi / 2 - result[2]) > 1.


def test_dynamic_waist_and_mounted_sensor_cancel_in_robot_pose():
    map_rotation = Rotation.from_euler('xyz', [.04, -.06, .3]).as_matrix()
    desired = np.eye(4)
    desired[:3, :3] = Rotation.from_euler('xyz', [.02, -.03, .8]).as_matrix()
    desired[:3, 3] = [2., -1., .7]
    for waist in [-.7, 0., .9]:
        kinematic = np.eye(4)
        kinematic[:3, :3] = Rotation.from_euler('xyz', [.2, -.3, waist]).as_matrix()
        kinematic[:3, 3] = [.2, -.05, .35]
        level_from_imu = desired @ kinematic
        camera_rotation = map_rotation.T @ level_from_imu[:3, :3]
        camera_position = map_rotation.T @ (level_from_imu[:3, 3] - [0, 0, .8])
        result = project_base([.3, -.2, .4], map_rotation, [0, 0, .8], camera_position,
                              Rotation.from_matrix(camera_rotation).as_quat(), kinematic)
        correction = Rotation.from_euler('z', .4).as_matrix()
        expected_position = correction @ desired[:3, 3] + [.3, -.2, 0]
        expected_forward = correction @ desired[:3, 0]
        np.testing.assert_allclose(result, [*expected_position[:2], np.arctan2(expected_forward[1], expected_forward[0])], atol=1e-12)


def test_invalid_kinematic_transform_is_rejected():
    with pytest.raises(ValueError):
        project_base([0, 0, 0], np.eye(3), np.zeros(3), [0, 0, 0], [0, 0, 0, 1], np.zeros((4, 4)))
