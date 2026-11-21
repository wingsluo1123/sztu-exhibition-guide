# 优化后的离线查看候选图

同源输入：lesson56_paired_run01。冻结lesson57的坐标、地板和边界，使用全部1428帧。
新旧trajectory.csv、transform.json字节相同，不是新的FAST-LIO运行。
改变的是观测帧选择和弱命中冲突融合，未实施回环、墙面强制补线或轨迹清空。
查看：bash /home/sztu/sztu/src/sztu_learning/view_learning_map.bash
当前已经在RViz显示；关闭后再启动，避免重复/map服务。
完整对照、墙体问题、证据和限制见 ../../地图优化与墙体问题.md。
本图只是离线候选图，不是已通过实机导航验收的正式地图。
