# 第80步：同一次FAST-LIO运行，逐帧保存点云和位姿

运行地点为电脑离线，使用既有lesson43移动录包，ROS_DOMAIN_ID=42、ROS_LOCALHOST_ONLY=1。
不会要求现场再走一遍，也不发送速度。新功能为lesson56_collect_frames.py。
FAST-LIO仍为之前从参考项目复制并独立编译的学习副本，本轮不改算法和时间候选假设。
新启动脚本lesson56_moving_input.bash复用lesson47配置，只将ROS日志改到本课目录。
因此仍用lesson44话题；课次编号不要求话题更名。

```text
原始录包 → 可靠点云格式转换 + IMU → FAST-LIO
                                       ├─ 配准点云
                                       └─ body位姿
                                            ↓ 按完全相同输出时间戳配对
                                       一帧一个NPZ + frames.csv索引
```

已有lesson44_collect_mapping.py把所有点连续追加进一个PCD，轨迹另存CSV；
此方式可以看整片点云，但没有保存“哪一批点属于哪一帧”。
本课另写收集器，从算法输出开始保留每帧点云及同次运行的完整位姿，旧产物不覆盖。
XYZ提取/有限值处理复用已讲过的lesson44_collect_mapping.xyz_bytes，其他配对与档案逻辑新写。

两个输入为：
- /learning/lesson44/cloud_registered，sensor_msgs/msg/PointCloud2，frame_id=camera_init。
- /learning/lesson44/odometry，nav_msgs/msg/Odometry，frame_id=camera_init，child_frame_id=body。

FAST-LIO源码发布两者时均使用同一lidar_end_time转换成header.stamp，因此本课按整数
(sec,nanosec)精确匹配。该时间是算法帧参考时间，不是这一帧所有原始激光的共同发射时刻。
先到的消息放进self.clouds或self.poses字典，另一个到达后检查是否具备同一键。

```python
if stamp in self.clouds and stamp in self.poses:
    cloud = self.clouds.pop(stamp)
    odometry = self.poses.pop(stamp)
    self.save_frame(stamp, cloud, odometry)
```

pop取出并移除已配对缓存条目，self.save_frame是本课自写保存函数。
它提取有效XYZ、body位置XYZ、朝向四元数xyzw以及整数时间戳，
用np.savez放进同一个.npz文件。NPZ是NumPy保存多个命名数组的容器，本课没有启用压缩。
点坐标单位米、处于camera_init；朝向描述body在该参考系中的方向，没有被忽略或改为默认值。
每个文件名带秒和九位纳秒，frames.csv记录时间、相对文件路径、点数。
程序拒绝同类重复时间戳，超过100条未配对消息停止检查，不自动错配最近时间。
退出时必须无未配对且至少一帧完成，否则保留错误报告并报失败。

输出maps/lesson56_paired_run01是**新一趟运行**，只使用本趟位姿和点云。
FAST-LIO回放可能产生不同数值，不能把它接到旧地图/轨迹/旧展点上继续算而不处理坐标一致性。
尚未把本课新输出水平化、套用旧格子或计算雷达起点；后续必须明确同源坐标及外参变换。
档案保留扫描归属，是恢复观测关系的一步，不等于恢复每束原始激光的瞬时起点。
需要更精确的射线时仍须考虑原始逐点时间、运动补偿和外参，不能不加说明把整帧点都当同一时刻测得。

实际编排、参数/二进制与输入快照、进程日志在runtime_logs/lesson56，属于助手处理的运行和验证记录。
教学重点是“同一个时间戳→同帧配对→一起保存”，不逐行讲编排。
完整手动启动步骤沿用lesson47_offline_run.md，只将算法脚本换为lesson56_moving_input.bash，
收集器换为`python3 lesson56_collect_frames.py maps/某个不存在的新目录`，不与旧链路同时启动。
本课已有运行结果时无需再执行一次，运行产物说明见maps/lesson56_paired_run01/README.md。
