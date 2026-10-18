# 2026-10-13 18:27 新建地图（待现场验收）

本目录保留新一趟建图结果，不替换 `maps/testroom`，不复用旧地图点位。
原始录制保存在机器人 `agi@192.168.43.5`：
`/home/agi/sztu_lio_runs/20261013_182750_live/odom`。

## 数据与生成参数

- 录制时长 344.137 秒，95,502 条消息，原始数据库 1,540,984,832 字节。
- `/Odometry`、`/cloud_registered` 各 3,432 帧，约 10 Hz；全部点云匹配到里程计。
- 轨迹长度 30.90 米，最大单帧位移 0.091 米，沿重力方向起伏 0.18 米。
- 使用已录制的注册点云直接投影，未重新回放 FAST-LIO。
- 重力取开头静止段 IMU；地面拟合厚度约 11 毫米，地面法向与重力差 1.58°。
- 分辨率 0.05 米；离地高度 0.25–1.80 米；最大范围 15 米；使用每一帧。
- 图幅 373 × 335 格（18.65 × 16.75 米），原点 `[-5.5470, -9.4808, 0]`。
- 占据 23.70 平方米，空闲 74.84 平方米，未知 213.84 平方米。

生成命令（在机器人 `~/sztu`，已加载 ROS Humble 环境）：

```bash
python3 tools/lio_report.py /home/agi/sztu_lio_runs/20261013_182750_live/odom \
  --src-bag /home/agi/sztu_lio_runs/20261013_182750_live/odom --no-loop --live
python3 tools/gridmap.py /home/agi/sztu_lio_runs/20261013_182750_live \
  -o /home/agi/sztu_lio_runs/20261013_182750_live/map
```

生成代码版本为 `075691a-dirty`，当时机器人脚本 SHA-256：

```text
a201aca74d1c6b8d8a8139409d6fef0d5d4954c157391b1b7ca269dd833574aa  tools/gridmap.py
a1b2a85a4f94e00770aa040f803c0a55cfe87f5836a57f2cee457569c862aaf2  tools/lio_report.py
```

## 验证与限制

机器人地图服务器已加载本图并激活；开发机 RViz 已收到 373 × 335 地图。
独立开发机订阅校验全部 124,955 格、分辨率、尺寸和原点，均与本目录 PGM/YAML 相符。
本地文件与机器人原文件 SHA-256 相同：

```text
8b4a22dc1f65786e073cec9398e7a5e98cf7b8b064b0a15cb3da64a9a85c3d34  map.pgm
fd48077303827bbd4eddfa2a1b60547bbcd63968890dcf43f877870b244c4469  map.yaml
```

`map.png` 叠加了轨迹，供查看；导航使用 `map.pgm`。原始 `bag_metadata.yaml`、
`gridmap_build.log`、`map_quality_report.txt` 随图保留。
外墙轮廓可辨，但室内仍有较密集的占据区域，尚需与现场家具、人员及真实通道核对；
不能凭地面拟合或里程计检查通过就认定地图可用于导航。
未确认起终点重合，因此未评估闭合误差；FAST-LIO 不提供回环纠正。

原始 YAML 保留生成器内容以保持字节一致。其重力注释不是本项目 AMCL 导航的启动指令，
AMCL 使用方法以 `docs/autonomous_navigation.md` 为准；其中 `odom_topic` 字段由生成器
写为点云输入 `/cloud_registered`，实际位姿来源为 `/Odometry`。
