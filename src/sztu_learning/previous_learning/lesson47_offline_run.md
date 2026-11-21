# 完整移动包离线运行：功能与启动方式

在电脑运行，ROS domain42、仅本机通信，不发运动指令。机器人不需要重新走一遍。

```text
播放器的原始点云 → lesson46_reliable_relay.py → FAST-LIO
播放器的IMU ────────────────────────────────→ FAST-LIO
FAST-LIO的配准点云与位姿 → lesson44_collect_mapping.py → PCD与CSV
```

FAST-LIO是此前从参考项目复制、独立编译的学习副本，不是自写算法；
本次增加可选的可靠点云订阅，默认false；学习版算法已重新编译。
逐点时间按候选ns解释、外参和局部地图参数仍沿用之前配置，未据此宣称地图准确。

## 本次关键配置

`fastlio_offline_reception.yaml`：

```yaml
laser_mapping:
  ros__parameters:
    common:
      reliable_lidar_input: true
```

`laser_mapping`匹配节点名称，`ros__parameters`下面是ROS参数，
嵌套的common与reliable_lidar_input合起来对应参数`common.reliable_lidar_input`。
此参数为本轮给学习副本新增，不是原版FAST-LIO通用参数；不改计算算法。
`true`让PointCloud2输入请求RELIABLE；上游lesson46发布端已经提供RELIABLE。
原始点云侧也由可靠播放器向lesson46的可靠订阅端发布。
不能把这一设置直接用于只提供BEST_EFFORT的上游。

`lesson47_moving_input.bash`沿用lesson44启动方式，多加载这一份参数文件。
仍使用lesson44话题名称；新脚本名不要求另起一套话题。不要同时运行旧脚本。

## 手动启动参考

以下是可复现的独立终端步骤，不要求为了学习再重跑一遍；助手本次用验证编排启动了同一组功能程序。
输出文件不可覆盖，选择不存在的新目录。按1至4启动，先准备接收端，最后播放数据。

【电脑：本地终端1】

```bash
bash /home/sztu/sztu/src/sztu_learning/lesson47_moving_input.bash
```

【电脑：本地终端2、3、4，各自先执行环境设置】

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/home/sztu/sztu/src/sztu_learning/runtime_logs/lesson47/ros_logs
cd /home/sztu/sztu/src/sztu_learning
```

终端2启动转换：

```bash
python3 lesson46_reliable_relay.py
```

终端3启动保存（这里的目录只是供以后手动运行选择，必须不存在）：

```bash
python3 lesson44_collect_mapping.py maps/lesson47_manual_run01
```

终端4播放：

```bash
ros2 bag play bags/lesson43_move_20261119_231833 \
  --rate 0.5 --delay 2 --disable-keyboard-controls \
  --qos-profile-overrides-path lesson44_play_qos.yaml \
  --topics /aima/hal/sensor/lidar_chest_front/lidar_pointcloud /aima/hal/sensor/lidar_chest_front/imu \
  --remap /aima/hal/sensor/lidar_chest_front/imu:=/learning/lesson44/imu \
  --wait-for-all-acked 5000
```

播放器只回放点云与IMU；不循环、不回放TF、不发布/clock。
确认前面三个接收/处理节点已启动再播放；delay2只延迟播放，不保证节点就绪。
播放结束后等待输出停止，再在保存终端Ctrl+C，使其关闭文件并完成PCD组装；
随后退出转换和算法终端。最终数据为累计扫描点PCD和轨迹CSV，不是导航验收通过的地图。

本次实际运行的可追溯配置、日志、接收核对和产物检查位于runtime_logs/lesson47，
实际输出另存maps/lesson47_move_run01，保留lesson44历史结果。
