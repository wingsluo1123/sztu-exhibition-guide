from pathlib import Path
import sys

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from online_bootstrap import OnlineBootstrap
from online_sensor_adapter import adapt, OUTPUT_DTYPE


def raw_cloud():
    dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'timestamp'],
                      'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'],
                      'offsets': [0, 4, 8, 16, 20, 24], 'itemsize': 32})
    payload = bytearray(144)
    points = np.ndarray((2, 2), dtype=dtype, buffer=payload, strides=(72, 32))
    points['x'] = [[1., 2.], [3., 4.]]
    points['timestamp'] = [[0., 1000000.], [40000000., 80000000.]]
    points['ring'] = [[0, 1], [94, 95]]
    message = PointCloud2(height=2, width=2, point_step=32, row_step=72, data=bytes(payload))
    message.header.frame_id = 'lidar_chest_front'
    message.header.stamp.sec = 123
    for name in dtype.names:
        datatype = PointField.UINT16 if name == 'ring' else PointField.FLOAT64 if name == 'timestamp' else PointField.FLOAT32
        message.fields.append(PointField(name=name, offset=dtype.fields[name][1], datatype=datatype, count=1))
    return message


def test_adapter_preserves_stamps_xyz_and_handles_padded_rows():
    original = raw_cloud()
    converted = adapt(original)
    points = np.frombuffer(converted.data, dtype=OUTPUT_DTYPE)
    np.testing.assert_allclose(points['x'], [1, 2, 3, 4])
    np.testing.assert_allclose(points['time'], [0, .001, .04, .08])
    assert list(points['ring']) == [0, 1, 94, 95]
    assert converted.header == original.header
    assert converted.row_step == 96


@pytest.mark.parametrize('fault', ['length', 'datatype', 'frame', 'time', 'ring'])
def test_adapter_rejects_changed_sensor_contract(fault):
    message = raw_cloud()
    if fault == 'length':
        message.data = bytes(143)
    elif fault == 'datatype':
        message.fields[0].datatype = PointField.FLOAT64
    elif fault == 'frame':
        message.header.frame_id = 'base_link'
    else:
        import struct
        payload = bytearray(message.data)
        struct.pack_into('<d' if fault == 'time' else '<H', payload, 24 if fault == 'time' else 20,
                         1795167723000000000. if fault == 'time' else 96)
        message.data = bytes(payload)
    with pytest.raises(ValueError):
        adapt(message)


def bootstrap(gyro=0., shift=0.):
    config = {'floor_search_height_m': [-1.4, -.6], 'floor_seed': 57,
              'floor_inlier_m': .05, 'obstacle_height_m': [.25, 1.85]}
    candidate = OnlineBootstrap(config, imu_samples=4, cloud_frames=3)
    grid = np.arange(-3., 3., .08)
    horizontal = np.array(np.meshgrid(grid, grid)).reshape(2, -1).T
    points = np.column_stack([horizontal, np.full(len(horizontal), -.9)])
    for sample in range(4):
        candidate.add_imu(float(sample), [0., 0., 9.81], [gyro, 0., 0.])
    for sample in range(3):
        candidate.add_frame(float(sample), points, [sample * shift, 0., 0.], [0., 0., 0., 1.])
    return candidate


def test_current_startup_estimates_floor_without_old_frame():
    result = bootstrap().solve()
    np.testing.assert_allclose(result['rotation'], [[0, 1, 0], [-1, 0, 0], [0, 0, 1]], atol=1e-10)
    np.testing.assert_allclose(result['translation'], [0, 0, .9], atol=1e-10)
    assert result['frames'] == 3


@pytest.mark.parametrize('gyro,shift', [(.1, 0.), (0., .1)])
def test_moving_startup_is_rejected(gyro, shift):
    with pytest.raises(ValueError, match='运动'):
        bootstrap(gyro, shift).solve()


def test_display_freshness_rejects_backlog_and_uncertain_clock():
    from online_display_bridge import fresh, clock_offset
    offset = clock_offset(99.98, 100.02, 100.6)
    assert fresh(101.6, .02, 101.05, offset, .02)
    assert not fresh(100.6, .02, 101.05, offset, .02)
    assert fresh(100., .05, 110.1, 10., .02)
    assert not fresh(100., .05, 111., 10., .02)
    assert not fresh(100., .05, 110.1, 10., .2)
    assert not fresh(102., .05, 110.1, 10., .02)
    assert not fresh(100., .6, 110.1, 10., .02)


def test_display_shutdown_cleans_ssh_after_ros_context_has_closed():
    from types import SimpleNamespace
    from online_display_bridge import ComputerDisplay
    calls = []
    process = SimpleNamespace(terminate=lambda: calls.append('terminate'), wait=lambda timeout: calls.append('wait'))
    node = SimpleNamespace(context=SimpleNamespace(ok=lambda: False), process=process)
    ComputerDisplay.close(node)
    assert calls == ['terminate', 'wait']
