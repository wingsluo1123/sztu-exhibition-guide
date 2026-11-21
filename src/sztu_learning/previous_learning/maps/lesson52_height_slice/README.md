# 第76步：用坐标Z范围选出候选高度切片

电脑离线运行学习目录新写的lesson52_height_slice.py，未连接机器人或运行ROS。
输入maps/lesson51_turned_cloud/registered_scans_xyz.pcd及其同源位置/变换。
输入和输出参考系均为learning_level_turned_candidate，单位米。
配置lesson52_slice_config.json，选取0.0<=Z<=1.0，包含两端边界。
**Z相对当前坐标原点，不是离地高度；本课没有完成地板高度标定。**

范围是用于演示布尔筛选和观察墙面等结构的教学选择，不来自机器人尺寸、正式净空或避障要求。
初步查看源点云Z分布后选取这个原点上方的区间观察，不把该区间当作自动求得的地面或障碍物分割。
范围外的矮物体、高处结构和其他点不在此切片中。不能根据切片空白判断可通行。

输出height_slice_xyz.pcd含1592393点，源有18359162点。
保留源顺序及重复点，保留点的XYZ二进制数值不变；只选点、不旋转、不平移、不压扁Z。
PCD头按新点数写WIDTH/POINTS，XYZ仍为little-endian float32，12字节每点。
positions.csv和transform.json从输入目录逐字节复制；slice_config.json为本次参数副本。
selection.json记录本次实际点数与选择操作，不等于定位或导航验收。
源文件完整保留，本目录产物未用于控制或导航。

height_slice_comparison.png已查看：左侧全部高度，右侧选定Z区间；红线是同源轨迹。
两图按源点云每100点取样的XY分量1%～99%分位取相同显示界限，预览各自约抽样15万点。
显示限界不改变PCD；切片仍含当前视界之外的点。
界面留白可来自高度筛除、遮挡或未观测，不是自由空间标记。

边界/重复/空结果/非法范围控制例、保存点逐行逐字节子序列、源文件哈希、配置和轨迹拷贝核对通过。
记录runtime_logs/lesson52/output_check.json及同目录check_output.py，验证脚本不作为本轮主课。
未实现地面分割、去噪、占据/自由空间栅格、机器人净空或矮障碍检测。
命令（工作目录/home/sztu/sztu）：`OPENBLAS_NUM_THREADS=1 python3 src/sztu_learning/lesson52_height_slice.py`。
既有输出目录存在时程序拒绝覆盖；两张图显示范围随后由save_preview重新生成，PCD未改。
