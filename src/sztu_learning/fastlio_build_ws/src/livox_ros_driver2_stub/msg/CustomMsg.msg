# Livox CustomMsg —— 接口占位，不是真的 Livox 驱动
#
# FAST-LIO 只读 header / point_num / points 三项，但这里按 Livox 官方定义
# 写全，避免哪天换成真驱动时对不上。
std_msgs/Header header
uint64 timebase         # 本帧第一个点的绝对时间戳，纳秒
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
