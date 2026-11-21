# 学习目录入口

- `mapping_learning/`：当前从三维点云开始的建图学习，笔记为《开始建图学习笔记.md》。
- `previous_learning/`：此前课程代码、配置、笔记、地图、录包、验证程序及记录。
- `fastlio_build_ws/`：保留原位置的FAST-LIO编译工作区；旧课程目录通过同名链接使用它。
- `package.xml`、`setup.py`、`setup.cfg`、`resource/`、`sztu_learning/`：ROS包基础文件，保留原位置。

旧课程的代码、配置和数据一同移动，程序按自身目录查找的相对关系保留。
旧笔记中的历史命令若指向 `src/sztu_learning/lessonXX...`，现在应使用
`src/sztu_learning/previous_learning/lessonXX...`；运行位置仍按原笔记的电脑/机器人说明判断。
历史验收记录及地图清单保留原文，不批量改写其中的历史路径与哈希。
本次仅整理电脑工作目录，机器人上已部署的学习程序位置不变。

根目录保留以下五个常用查看入口，原命令可继续使用：

```bash
bash src/sztu_learning/view_learning_map.bash
bash src/sztu_learning/view_room_map.bash
bash src/sztu_learning/view_localization_sample.bash
bash src/sztu_learning/view_continuous_localization.bash
bash src/sztu_learning/view_online_localization.bash
```

各入口用途沿用旧课程，不要同时启动多个定位/回放入口。
旧代码相关测试：`bash tools/run_tests.sh src/sztu_learning/previous_learning/test`。

2026-11-20整理验证：34项相关测试通过；80帧离线定位回放到达最后一帧；
原地图查看入口成功加载地图，收到的全部270×295栅格与保存值一致。
记录见 `previous_learning/runtime_logs/reorganization_*`。
