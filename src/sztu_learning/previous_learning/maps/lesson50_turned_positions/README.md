# 第74步：轨迹绕竖直Z轴转180°

用户确认希望起点在下方、整条路线转180°，不是仅倒转图片显示。
在电脑离线运行学习目录新写的lesson50_turn_heading.py。
输入maps/lesson49_level_positions/positions.csv及同目录transform.json。
输出positions.csv为1428条时间戳和新XYZ，目标参考系learning_level_turned_candidate，单位米。
这里只保存位置，没有朝向、点云、协方差或ROS TF。

在第73步水平坐标基础上使用heading_rotation=diag(-1,-1,1)：
x_turned=-x_level，y_turned=-y_level，z_turned=z_level。
这是绕Z轴180°的旋转，右手性及向上方向不变；只改变一个水平轴符号会成为镜像反射。
原点不动，地板高度仍未知，朝向选择不自动代表机器人前方或北方。

transform.json记录从原camera_init一步到本轮目标坐标系的组合矩阵：
R_combined = R_heading @ R_level。原始camera_init点后续可直接乘该组合矩阵，
不能先使用组合矩阵再重复执行第73步旋转。对第73步已转换的位置仅应用R_heading。
调用此前lesson49_level_positions.transform_position执行单点矩阵乘法，未调用旧项目转换代码。

trajectory.png已查看，起点在下方、远端转弯在上方，符合用户示意。
核对1428行、时间戳不变、XY同时取反、Z不变、距离不变、组合变换一致、源文件未改通过；
记录在runtime_logs/lesson50/output_check.json。固定旋转不修复漂移，不证明地图准确度。
原lesson47/49结果保留，点云后续须采用匹配的组合变换才能和本轮位置共同使用。
