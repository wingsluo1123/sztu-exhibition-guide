# 已在RViz实际加载的二维候选地图

来源：lesson56_paired_run01的同次FAST-LIO点云/位姿，未混入其他运行。
地图：map.pgm + map.yaml；5cm/格，268列×333行，frame_id=learning_map。
数据与过程：summary.json、transform.json、trajectory.csv、keyframes.json、manifest.json、evidence/。
地图保存的数组行0对应最小Y，PGM图片已作一次上下翻转；真实map_server重载逐格一致。
map_preview.png上的橙线是历史body轨迹，不改变地图，也不代表现在机器人位置。

在电脑运行：

```bash
bash /home/sztu/sztu/src/sztu_learning/view_learning_map.bash
```

使用ROS域42、仅本机通信；已有这套显示服务时不要重复运行。
关闭RViz或在启动终端Ctrl+C结束本次显示。
具体参数、证据、来源和限制见[地图生成与RViz验收.md](../../地图生成与RViz验收.md)。

本次通过地图保存、真实/map重载及RViz显示验收。未知区域保持未知，空闲区域保守。
当前0.25～1.85m高度带和扫描参考射线是候选模型，未通过完整机体通行/导航验收。
