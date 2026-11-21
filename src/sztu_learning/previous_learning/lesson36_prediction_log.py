"""读取离线实验的匹配前状态日志；内部预测值不代表有效融合定位。"""

import math
from pathlib import Path

log_path = Path(__file__).parent / 'runtime_logs' / 'lesson34' / 'mat_pre.txt'
rows = [list(map(float, line.split())) for line in log_path.read_text().splitlines() if line.strip()]
if not rows or any(len(row) != 25 for row in rows):
    raise RuntimeError('需要本次源码格式的非空25列匹配前状态日志。')

all_finite = all(math.isfinite(value) for row in rows for value in row)
positions = [row[4:7] for row in rows]
distances = [math.sqrt(x*x + y*y + z*z) for x, y, z in positions]

print(f'匹配前状态记录：{len(rows)}条')
print(f'全部记录的25列数值均有限：{all_finite}')
print(f'首条内部位置：{positions[0]} m')
print(f'末条内部位置：{positions[-1]} m')
print(f'所记录位置距算法原点的最大距离：{max(distances):.6f} m')
print('以上是内部预测状态，不证明实际运动、融合定位或地图正确。')
