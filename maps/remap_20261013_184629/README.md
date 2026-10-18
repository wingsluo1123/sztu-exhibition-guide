# 2026-10-13 18:46 建图结果（已选用，待导航验收）

2026-10-14 操作者确认选用此图并命名为“展厅图”，最终落地地图入口为 `maps/exhibition/map.yaml`。
展厅副本与本目录 PGM/YAML 字节一致；教室测试将使用独立录制的新图。
地图独立保留，旧 `maps/testroom` 与配套点位保留作历史记录；新图讲解点位需重新标定。
原始数据仍在机器人 `agi@192.168.43.5`：
`/home/agi/sztu_lio_runs/20261013_184629_live/odom`。

## 生成与检查

- 录制约 7.4 分钟，123,486 条消息，原始数据库 1,998,192,640 字节。
- 4,420 帧注册点云全部匹配到 `/Odometry`，约 10 Hz。
- 轨迹长度 51.15 米；最大单帧位移 0.097 米；沿重力方向起伏 0.39 米。
- 直接使用已录制的注册点云，未重新回放 FAST-LIO。
- 重力来自开头静止段 IMU；拟合地面厚度约 13 毫米，法向与重力差 1.16°。
- 栅格分辨率 0.05 米；离地高度带 0.25–1.80 米；最大范围 15 米；使用每一帧。
- 图幅 488 × 532 格（24.4 × 26.6 米），原点 `[-14.4467, -9.3313, 0]`。
- 占据 33.19 平方米，空闲 150.43 平方米，未知 465.42 平方米。

在机器人 `~/sztu` 加载 ROS Humble 后执行：

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 tools/lio_report.py /home/agi/sztu_lio_runs/20261013_184629_live/odom \
  --src-bag /home/agi/sztu_lio_runs/20261013_184629_live/odom --no-loop --live
python3 tools/gridmap.py /home/agi/sztu_lio_runs/20261013_184629_live \
  -o /home/agi/sztu_lio_runs/20261013_184629_live/map
```

生成代码版本 `075691a-dirty`，当时机器人脚本 SHA-256：

```text
a201aca74d1c6b8d8a8139409d6fef0d5d4954c157391b1b7ca269dd833574aa  tools/gridmap.py
a1b2a85a4f94e00770aa040f803c0a55cfe87f5836a57f2cee457569c862aaf2  tools/lio_report.py
```

## 保存与展示

机器人地图服务器已加载并激活；开发机 RViz 已加载 488 × 532 地图。
独立订阅校验全部 259,616 格以及尺寸、分辨率、原点和坐标系，与保存文件相符。
本地与机器人文件 SHA-256 一致：

```text
484d274b7c7999bcafbab66b0fecd6a14326e48433e87de9bed053ffa7fd7fa8  map.pgm
8d68e9212c8ecaa9233082a41dc60f05bf31e822f290a83075251251bab0ff32  map.yaml
```

导航使用 `map.pgm`；`map.png` 叠有轨迹，供查看。
原始 `bag_metadata.yaml`、`gridmap_build.log`、`map_quality_report.txt` 随图保存。
外墙、内部结构与通道轮廓可辨，但边缘存在射线状杂点；实际通道和占据区仍需现场核对。
未确认起终点重合，未评估闭合误差；FAST-LIO 不提供回环纠正。
上述检查不代表地图已通过导航验收。

原始 YAML 保持生成器内容，其重力注释不作为 AMCL 导航的启动指令；
AMCL 使用方式以 `docs/autonomous_navigation.md` 为准。
生成器的 `odom_topic` 字段写为点云输入 `/cloud_registered`，实际位姿来自 `/Odometry`。
