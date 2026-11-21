# 小场地新地图：离线候选

来源：bags/room_7x3_20261120_154812，由现场遥控新采集；初次失败包未使用。
同次FAST-LIO档案：maps/room_7x3_20261120_run01，1182对点云/body位姿。
重新拟合本趟地板并建立坐标和边界，未套上一场地地图变换或点位。
地图map.pgm+map.yaml，5cm/格、295×270，frame_id=learning_map。
同名frame_id只是当前独立查看约定，不意味着与旧场地原点相同；不能混合点位或同时发布。
轨迹仅为历史估计，不是实时机器人；完整扫描范围包含桌椅后方。
查看：bash /home/sztu/sztu/src/sztu_learning/view_room_map.bash
RViz已实际显示，/map与GetMap重载每格相同；未知和局部墙面粗线仍存在。
这不是精度、物理标定或自主导航验收。详见../../小场地重新建图操作.md。
