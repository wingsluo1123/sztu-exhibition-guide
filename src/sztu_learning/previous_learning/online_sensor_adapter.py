"""Vectorized live sensor adapter; keeps raw stamps and exposes bounded input diagnostics."""
import json
import time
from array import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


OUTPUT_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'time', 'ring'],
                         'formats': ['<f4', '<f4', '<f4', '<f4', '<f4', '<u2'],
                         'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24})


def adapt(message):
    if message.is_bigendian or message.header.frame_id != 'lidar_chest_front':
        raise ValueError('雷达字节序或坐标系改变')
    fields = {field.name: field for field in message.fields}
    expected = {'x': (7, '<f4'), 'y': (7, '<f4'), 'z': (7, '<f4'),
                'intensity': (7, '<f4'), 'ring': (4, '<u2'), 'timestamp': (8, '<f8')}
    for name, (datatype, format_name) in expected.items():
        if name not in fields or fields[name].datatype != datatype or fields[name].count != 1:
            raise ValueError('雷达字段格式改变：' + name)
        if fields[name].offset < 0 or fields[name].offset + np.dtype(format_name).itemsize > message.point_step:
            raise ValueError('雷达字段越界')
    if message.row_step < message.width * message.point_step or len(message.data) != message.row_step * message.height:
        raise ValueError('雷达数据长度无效')
    dtype = np.dtype({'names': list(expected), 'formats': [value[1] for value in expected.values()],
                      'offsets': [fields[name].offset for name in expected], 'itemsize': message.point_step})
    raw = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                     strides=(message.row_step, message.point_step)).reshape(-1)
    if not len(raw):
        raise ValueError('空雷达帧')
    if any(not np.isfinite(raw[name]).all() for name in expected):
        raise ValueError('雷达帧含无效值')
    seconds = raw['timestamp'] * 1e-9
    if seconds.min() < 0 or seconds.max() > .15 or raw['ring'].max() >= 96:
        raise ValueError('点时间或扫描线超出已核对的候选格式')
    converted = np.zeros(len(raw), dtype=OUTPUT_DTYPE)
    for name in ['x', 'y', 'z', 'intensity', 'ring']:
        converted[name] = raw[name]
    converted['time'] = seconds
    output = PointCloud2(header=message.header, height=1, width=len(converted),
                         is_bigendian=False, point_step=24, row_step=24 * len(converted), is_dense=True)
    output.fields = [PointField(name=name, offset=OUTPUT_DTYPE.fields[name][1],
                               datatype=PointField.UINT16 if name == 'ring' else PointField.FLOAT32, count=1)
                     for name in OUTPUT_DTYPE.names]
    output.data = array('B', converted.tobytes())
    return output


class LiveSensorAdapter(Node):
    def __init__(self):
        super().__init__('learning_live_sensor_adapter')
        self.publisher = self.create_publisher(PointCloud2, '/learning/online/candidate', qos_profile_sensor_data)
        self.status_publisher = self.create_publisher(String, '/learning/online/adapter_status', 1)
        self.last_stamp, self.accepted, self.rejected = None, 0, 0
        self.last_reason, self.last_age, self.last_duration = 'waiting_for_sensor', None, None
        self.create_subscription(PointCloud2, '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud', self.receive, qos_profile_sensor_data)
        self.create_timer(1., self.status)

    def receive(self, message):
        started = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        try:
            if not -.05 <= age <= .4 or self.last_stamp is not None and stamp <= self.last_stamp:
                raise ValueError('雷达时间戳过期、超前或未递增')
            converted = adapt(message)
            self.publisher.publish(converted)
            self.last_stamp, self.accepted, self.last_reason = stamp, self.accepted + 1, 'accepted'
        except ValueError as error:
            self.rejected += 1
            self.last_reason = str(error)
        self.last_age, self.last_duration = age, time.monotonic() - started

    def status(self):
        self.status_publisher.publish(String(data=json.dumps({'accepted': self.accepted, 'rejected': self.rejected,
                                                             'last_reason': self.last_reason, 'input_age_s': self.last_age,
                                                             'conversion_wall_s': self.last_duration,
                                                             'point_time_interpretation': 'Candidate relative ns; nominal calibration retained'})))


def main():
    rclpy.init()
    node = LiveSensorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
