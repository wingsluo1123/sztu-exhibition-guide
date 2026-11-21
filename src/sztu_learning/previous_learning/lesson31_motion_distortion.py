"""一维匀速平移的假设例子，用于理解运动去畸变；不是实际点云算法。"""

speed = 1.0  # 假设雷达沿x正方向移动，单位m/s。
frame_duration = 0.1  # 假设本帧起点到终点为0.1秒。
# 每项为（相对帧起点的采样时间，采样时静止目标在雷达中的x坐标）。
samples = [(0.0, 10.0), (0.1, 9.9)]

for point_time, measured_x in samples:
    remaining_distance = speed * (frame_duration - point_time)
    x_at_frame_end = measured_x - remaining_distance
    print(f't={point_time:.1f} s: {measured_x:.3f} m -> {x_at_frame_end:.3f} m')
