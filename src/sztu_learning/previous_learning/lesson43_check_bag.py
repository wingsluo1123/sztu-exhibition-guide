"""只读检查移动录包；数据非空、时序正常不等于地图或运动已验证。"""

import json
from pathlib import Path
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def inspect_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    report = {'bag': str(path), 'topics': {}, 'tf_pairs': {}}
    previous = {}
    while reader.has_next():
        topic, data, recorded_ns = reader.read_next()
        message = deserialize_message(data, get_message(types[topic]))
        item = report['topics'].setdefault(topic, {
            'type': types[topic], 'count': 0,
            'recorded_min_ns': recorded_ns, 'recorded_max_ns': recorded_ns,
        })
        item['count'] += 1
        item['recorded_min_ns'] = min(item['recorded_min_ns'], recorded_ns)
        item['recorded_max_ns'] = max(item['recorded_max_ns'], recorded_ns)
        if hasattr(message, 'header'):
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            item.setdefault('frames', [])
            if message.header.frame_id not in item['frames']:
                item['frames'].append(message.header.frame_id)
            item['header_min_ns'] = min(item.get('header_min_ns', stamp), stamp)
            item['header_max_ns'] = max(item.get('header_max_ns', stamp), stamp)
            item.setdefault('backward_header_steps', 0)
            item.setdefault('duplicate_header_steps', 0)
            item.setdefault('max_header_gap_sec', 0.0)
            if topic in previous:
                delta = stamp - previous[topic]
                item['backward_header_steps'] += int(delta < 0)
                item['duplicate_header_steps'] += int(delta == 0)
                item['max_header_gap_sec'] = max(item['max_header_gap_sec'], delta / 1e9)
            previous[topic] = stamp
        if hasattr(message, 'point_step'):
            count = message.width * message.height
            item['min_points'] = min(item.get('min_points', count), count)
            item['max_points'] = max(item.get('max_points', count), count)
            item.setdefault('fields', [
                {'name': field.name, 'offset': field.offset, 'datatype': field.datatype, 'count': field.count}
                for field in message.fields
            ])
        if hasattr(message, 'transforms'):
            pairs = report['tf_pairs'].setdefault(topic, {})
            for transform in message.transforms:
                key = transform.header.frame_id + ' -> ' + transform.child_frame_id
                pairs[key] = {
                    'translation': [getattr(transform.transform.translation, axis) for axis in 'xyz'],
                    'rotation_xyzw': [getattr(transform.transform.rotation, axis) for axis in 'xyzw'],
                }
    return report


if __name__ == '__main__':
    print(json.dumps(inspect_bag(Path(sys.argv[1])), ensure_ascii=False, indent=2))
