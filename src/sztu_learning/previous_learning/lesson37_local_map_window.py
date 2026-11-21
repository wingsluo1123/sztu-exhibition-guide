"""复现源码的局部窗口边界判定；不运行FAST-LIO或生成地图。"""

det_range = 300.0  # 当前源码默认mapping.det_range，单位米。
move_threshold = 1.5  # 源码MOV_THRESHOLD常量，无单位。

for cube_side_length in (200.0, 1000.0):
    # 假设雷达位于立方体窗口正中心，沿每个轴到边界都是半条边长。
    distance_to_edge = cube_side_length / 2.0
    trigger_distance = move_threshold * det_range
    need_move = distance_to_edge <= trigger_distance
    print(f'窗口边长={cube_side_length:.1f} m，中心到边界={distance_to_edge:.1f} m')
    print(f'移动触发距离={trigger_distance:.1f} m，需要移动窗口={need_move}')
