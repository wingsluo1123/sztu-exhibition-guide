# Livox CustomPoint —— 接口占位，不是真的 Livox 驱动
#
# 字段和 Livox 官方 livox_ros_driver2 一致。FAST-LIO 的 avia 分支会读
# x/y/z/line/tag/reflectivity/offset_time，全都在这儿。
uint32 offset_time      # 相对 CustomMsg.timebase 的偏移，纳秒
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
