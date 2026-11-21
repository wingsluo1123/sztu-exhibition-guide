"""假设逐点偏移相对于header且单位纳秒，演示时间重建；不证明真机协议。"""

header_sec = 100
header_nanosec = 900_000_000
offset_nanosec = 150_000_000

header_time_ns = header_sec * 1_000_000_000 + header_nanosec
point_time_ns = header_time_ns + offset_nanosec
point_sec, point_nanosec = divmod(point_time_ns, 1_000_000_000)

print(f'header: {header_sec}.{header_nanosec:09d} s')
print(f'offset: {offset_nanosec} ns')
print(f'point:  {point_sec}.{point_nanosec:09d} s')
