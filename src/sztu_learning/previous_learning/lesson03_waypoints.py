"""教学假设：同一 demo_map 坐标系，位置单位为米，不可用于真机导航。"""

import math

robot_position = {'x': 1.0, 'y': 0.0}

waypoints = [
    {'name': '入口', 'x': 0.0, 'y': 0.0, 'speech': '欢迎来到展厅，接下来开始参观。'},
    {'name': '机器人展区', 'x': 2.0, 'y': 1.0, 'speech': '这里展示机器人相关的教学与研究成果。'},
    {'name': '成果展示区', 'x': 4.0, 'y': 1.0, 'speech': '这里展示项目成果，本次参观到此结束。'},
]

for waypoint in waypoints:
    distance = math.hypot(
        waypoint['x'] - robot_position['x'],
        waypoint['y'] - robot_position['y'],
    )
    print(waypoint['name'])
    print(f"位置：x={waypoint['x']} m，y={waypoint['y']} m（demo_map，教学假设）")
    print(f'距机器人假设位置的直线距离：{distance:.2f} m')
    print(waypoint['speech'])
