"""将已知单位的帧内时间偏移换算为秒；不猜测真机单位或计时起点。"""

import math


def offset_to_seconds(value, unit):
    seconds_per_unit = {'s': 1.0, 'ms': 1e-3, 'us': 1e-6, 'ns': 1e-9}
    if unit not in seconds_per_unit:
        raise ValueError('必须明确指定单位：s、ms、us或ns。')
    if not math.isfinite(value) or value < 0:
        raise ValueError('本函数要求有限、非负的帧内时间偏移。')
    return value * seconds_per_unit[unit]


if __name__ == '__main__':
    # 假设数据：已知相对本帧起点的偏移为80000000纳秒，不来自机器人。
    offset = 80_000_000.0
    time_seconds = offset_to_seconds(offset, 'ns')
    print(f'假设偏移：{offset} ns -> {time_seconds} s')
