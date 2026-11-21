# 第77步：将高度切片投影到二维格子

电脑离线运行本学习目录新写的lesson53_point_grid.py。
输入maps/lesson52_height_slice/height_slice_xyz.pcd的全部1592393点，只使用XY求格子位置。
参考系仍为learning_level_turned_candidate，单位米；切片Z范围仍是相对坐标原点的0～1米。
本课不改变源点坐标，不新增坐标变换或ROS TF，不启动导航。

每格边长0.1米（10厘米），见lesson53_grid_config.json；该参数为教学显示选择，不代表真实精度或机器人净空。
网格左下角origin_xy=(-32.9,-26.3)米，在原有参考系中表达。
它是覆盖全部切片点的栅格边界位置，不是重新设置机器人/地图坐标原点。
范围由源点最小XY向下对齐格子边界求得，少量远点也保留，因此完整范围明显大于主要房间。
cell_xy=floor((xy-origin_xy)/resolution)，每点依次得到column_x、row_y。
数组按observed[row_y,column_x]存储，行数378、列数409，行增加对应坐标Y增加。
点按浮点表示和floor分到下边界包含、上边界不包含的格子；边界行为有具体控制例，未加任意坐标容差。

point_hits.npy为378×409的NumPy布尔数组：
- True：至少一个高度切片点落入该格子；不保证该点是真实静态障碍。
- False：未确定，没有作为自由/可通行格子。

共有5012个True格子，149590个False格子；多点同格只标True，不进行计数或概率融合。
grid.json记录参考系、分辨率、覆盖区域左下角、尺寸、行列顺序与状态含义。
positions.csv、transform.json、slice_config.json从输入逐字节复制，grid_config.json为本次配置。
位置用于叠加显示，未把红色轨迹沿途的格子自动标为自由。

point_grid.png已查看：蓝色格子有点落入，灰色未确定，红线为同源估计轨迹。
左侧完整保存范围，右侧按源切片XY抽样的1%～99%分位放大显示，保存数组包含全范围。
渲染origin='lower'保证行增加向上，不将NumPy数组默认图像方向误当地图方向。

已用负坐标、不对称行列、确切边界及边界前一个浮点值、重复/重排、非法输入控制例核对；
再对全部1592393个源点用独立Python math.floor逐点建立(row,column)集合，
与保存数组全部True位置集合完全相同，无漏格或多标格。源文件哈希及复制文件核对通过。
记录runtime_logs/lesson53/output_check.json与check_output.py，验证不作为本轮主课。

这不是ROS占据地图，未生成可加载导航的PGM/YAML、未推断自由空间、未膨胀障碍或进行路径规划。
高度切片可能漏掉低矮障碍，单点格子也可能来自噪点；固定水平化和里程计漂移限制仍存在。
代码的一千万格上限仅防止教学工具分配过大数组，不是物理通行判据。
命令（工作目录/home/sztu/sztu）：`OPENBLAS_NUM_THREADS=1 python3 src/sztu_learning/lesson53_point_grid.py`。
输出已存在时拒绝覆盖，原PCD和之前结果保留。
