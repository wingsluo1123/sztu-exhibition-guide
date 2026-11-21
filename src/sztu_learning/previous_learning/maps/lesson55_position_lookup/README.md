# 第79步：用消息时间戳查同源轨迹位置

电脑离线执行新写的lesson55_position_at_stamp.py，未重跑FAST-LIO、读取实时话题或连接机器人。
输入positions.csv为maps/lesson50_turned_positions/positions.csv，1428条位置，单位米，
参考系learning_level_turned_candidate，表示FAST-LIO输出body参考点的位置，不是已算出的雷达起点。
查找时间来自maps/lesson47_move_run01/collection.json保存的first_cloud_stamp、last_cloud_stamp，
是原运行收到的配准点云header.stamp，不是电脑当前时间，也不是单点发射时刻。

load_positions使用csv.DictReader读取已保存CSV，按整数(sec,nanosec)元组建字典。
每个键对应一行XYZ；相同时间戳重复出现则拒绝，以免悄悄覆盖。
position_at_stamp只接受完全相同键，缺失时报错，不选最近点、不假定两行之间静止。
整数秒/纳秒分开保留，没有拼成大浮点秒数，不引入时间舍入。
本课核心是时间关联，不涉及算法滤波或新位置估计。

实际两次查询：
- (1795101514,309956073) → (0.0067331999,-0.0150276356,0.0052161600)米。
- (1795101657,9885072) → (-0.4685836202,0.6810966037,0.0538516606)米。

输出positions_at_stamps.json只存查询结果、坐标系、数据来源与范围说明。
同秒不同纳秒/不同秒同纳秒、键缺失拒绝、重复行拒绝控制例通过；
保存结果独立逐行对应源CSV唯一行，源轨迹哈希未变。记录runtime_logs/lesson55/output_check.json。

这不等于恢复每束激光的测量起点：仍缺雷达安装偏移与朝向转换、逐点/扫描关联和相应时间语义。
输出点云时间戳是配准帧的参考时间，不等于这一帧所有原始激光同时发射。
本课没有给累计PCD恢复点的来源，没有清空真实网格、做射线跟踪或发布导航地图。
缺失时间戳的插值及不同时钟/协议核对留到实际需要时处理。
命令（/home/sztu/sztu）：`python3 src/sztu_learning/lesson55_position_at_stamp.py`；结果已存在时拒绝覆盖。
