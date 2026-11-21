# 同一轨迹的三个投影

来源：maps/lesson47_move_run01/trajectory.csv，1428条估计位姿。
由lesson48_view_projections.py在电脑离线绘制，未重新运行FAST-LIO。
三图均使用camera_init原坐标，分别选XY、XZ、YZ两列画图，比例尺各轴单位为米，未做坐标变换。
本次初始IMU约2秒400条记录的平均加速度方向约(0.99785,-0.02585,0.06031)，
结合当前算法初始化没有强制Z向上，支持camera_init的X轴近似竖直、YZ近似地面投影的解释。
IMU方向位于初始IMU坐标系，不能未经旋转直接用作任意时刻的camera_init精确重力方向。
此图只比较投影，不是校正后的导航平面地图；未生成map坐标系、占据栅格或可行驶区域。
方向检查见runtime_logs/lesson48/axis_check.json。正式导航前仍需明确水平坐标变换，并一致处理地图与位姿。
