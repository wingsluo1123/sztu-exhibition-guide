# 第75步：点云与轨迹使用相同坐标系

电脑离线处理，功能代码为本学习目录新写的lesson51_transform_cloud.py。
调用命令：`OPENBLAS_NUM_THREADS=1 python3 src/sztu_learning/lesson51_transform_cloud.py`
（从/home/sztu/sztu执行；目录已存在时会拒绝覆盖。）

输入是maps/lesson47_move_run01/registered_scans_xyz.pcd：同一趟FAST-LIO保存的累计配准扫描，
参考系camera_init，单位米，binary little-endian float32 XYZ，每点12字节。
不是原始雷达坐标的传感器扫描，不能给其他参考系的点云直接套用本次矩阵。
变换来自maps/lesson50_turned_positions/transform.json，是水平化和绕Z半圈的组合矩阵。
列向量约定p_target=R_combined@p_camera_init，平移为零、原点不变。
输出参考系learning_level_turned_candidate；没有发布同名ROS TF。

输出registered_scans_xyz.pcd含18359162个点，按源顺序一一转换、无删点/去重/降采样。
每批100000点只是控制内存用量，不改变空间选择或扫描内容。
数据按float64计算旋转，再按float32写回，因此存在数值舍入。
PCD头保持原XYZ布局及点数；该PCD头没有坐标系名称，参考系以本说明及transform.json/manifest.json为准。
positions.csv和transform.json从lesson50逐字节复制，与点云同源、同一最终变换，位置含1428条。
位置文件没有朝向，不能当作完整位姿。点云和轨迹不能再重复应用同一变换。

cloud_and_positions.png已查看：左为XY投影（包含所有抽样高度），右为Z朝上的3D预览。
蓝色点云、红色轨迹；预览约抽取15万个点，3D坐标视界按1%～99%分位限定，保存PCD包含全部点。
此图为可视化，不是占据栅格地图，蓝点不全是障碍物，也没有由缺少蓝点推断可通行区域。

runtime_logs/lesson51/output_check.json记录全体按序点逐坐标公式、逆变换、有限性、长度、
同源轨迹变换与原文件哈希核对结果；check_output.py为验证专用，不作为本课主代码。
逐坐标最大写回舍入差约1.65e-6米，只描述float32数值保存，不是地图真实精度。
变换保持候选向上方向与所选朝向，尚未标定地板零高度、修复漂移、转换朝向/协方差或建立导航地图。
未重跑FAST-LIO、启动ROS或连接/控制机器人。源PCD、原轨迹和前课结果保留。
