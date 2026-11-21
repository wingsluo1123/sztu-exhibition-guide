# 第一次遥控移动采集记录

2026-11-19，学习者确认会使用厂商遥控器、在机器人旁边可以操作。
本轮由学习者遥控机器人；电脑助手只停止冲突的旧项目服务并录制原始数据，没有发送速度指令。

## 实际采集

- 机器人：192.168.224.19，用户 agi。传感器驱动沿用机器人正在运行的厂商驱动。
- 时间：机器人本地23:18:33至23:20:57，录包消息跨度143.097872902秒。
- 现场反馈：已走完、已停稳；估计总路程约18米、转弯约90度，最后回到起点附近。
  这些是学习者现场估计，不是独立测量真值；不据此假定精确闭合或编造路线形状。
- 采集前实测旧项目sztu-bridge为active，内含navigation_manager；按项目实走约定执行systemctl --user stop sztu-bridge。
- 结束后sztu-bridge仍inactive，没有自动重启；这不改变其下次开机自启动配置。
- 未停止厂商传感器或走跑控制器，未运行学习版FAST-LIO，未调用未采购的厂商SLAM/导航功能。

机器人原始目录：

```text
/home/agi/sztu_learning/bags/lesson43_move_20261119_231833
```

电脑复制目标：

```text
/home/sztu/sztu/src/sztu_learning/bags/lesson43_move_20261119_231833
```

## 录制内容与方式

| 话题 | 消息数 | 用途 |
|---|---:|---|
| /aima/hal/sensor/lidar_chest_front/lidar_pointcloud | 1431 | 原始点云 |
| /aima/hal/sensor/lidar_chest_front/imu | 28607 | 雷达IMU |
| /tf | 6823 | 运行时动态坐标关系 |
| /tf_static | 5 | 固定坐标关系，消息内可包含多条变换 |

录制只接收并保存消息，不计算地图或控制机器人。
使用sqlite3和显式QoS配置lesson43_record_qos.yaml；点云/IMU选BEST_EFFORT、VOLATILE，避免取历史传感器消息。
静态TF使用RELIABLE、TRANSIENT_LOCAL，以接收录制器启动前已发布且发布者仍保留的固定变换。
机器人上保存的该次QoS、命令及录制日志位于：

```text
/home/agi/sztu_learning/capture_runs/lesson43_move_20261119_231833
```

录制进程设置最长180秒，实际在学习者回复已完成后用SIGINT结束，数据库已生成metadata.yaml，ros2 bag info可读。
没有更改机器人的系统时间、驱动配置、授权或全局ROS安装。

## 初检范围

lesson43_check_bag.py逐条反序列化录包，统计消息、坐标系、时间戳与TF父子关系。
先在旧包上核对42帧点云/647条IMU的已知计数，并确认没有误报TF，再检查新包。
本次点云和IMU均无header时间倒退或相邻重复，点云最大header间隔0.110776秒，IMU最大0.081001369秒。
最大间隔只描述录到的消息，不直接证明驱动丢帧原因；IMU存在间隔，不能声称完整无丢失。
点云每帧25389～25906点，frame为lidar_chest_front，IMU frame为lidar_imu_chest_front。
静态TF记录22组父子关系，动态TF31组；按父子对保留最后数值，不证明发布者唯一或全部变换一致。
这些检查不验证实际轨迹、时间协议、外参精度或地图质量。
FAST-LIO本身没有回环纠正；后续可观察首尾估计位置差，但实际终点只是“起点附近”，
未经独立测量的实际终点偏差混在首尾差中，不能把首尾差全部当作算法漂移。

runtime_logs/lesson43保存本轮预检、服务状态、录制起止、数据检查与复制校验记录。
电脑复制已完成，数据库与metadata.yaml的SHA-256分别和机器人原件相同。
数据库SHA-256为5750aa5068868c3526c38e8f4376e010c8e49da2861f90a6d217eb521a4a0b52。
本次是移动录包采集，不是自主导航完成。下一步用这段数据运行学习版离线算法，观察轨迹并保存累积结果。
