# FAST-LIO 独立学习编译目录

这里是从本机参考项目复制的源码快照，不是已经完成的自写 SLAM。
复制时保留了参考源码当前的修改；没有宣称它是未经修改的上游版本。
只复制构建需要的目录和文件，没有复制原项目的 build/install、历史日志或地图。
`source_snapshot.json` 记录复制来源、当时的 Git 状态和逐文件 SHA-256。
`livox_ros_driver2_stub` 仅提供消息接口，不包含驱动。

```text
src/       学习用源码副本
build/     编译中间结果
install/   本次构建的安装产物和环境脚本
log/       构建日志
```

所有命令在【电脑：本地终端】运行。新开终端，只加载 Humble，
不要先加载 `/home/sztu/sztu/install`，以免混入原项目同名包。

```bash
source /opt/ros/humble/setup.bash
cd /home/sztu/sztu/src/sztu_learning/fastlio_build_ws
export MAKEFLAGS=-j1
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon --log-base log build --base-paths src --executor sequential --packages-up-to fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

两个并行度变量限制单个包内部的构建并行度；`--executor sequential` 限制包间并行。
`--packages-up-to fast_lio` 选择本包和当前源码目录里它所依赖的包，
不会自动下载或安装缺失的外部依赖。
`--base-paths src` 将包发现范围限定为这里的源码目录。
根目录的 `COLCON_IGNORE` 是为了防止外层工作空间递归发现这里的重复包；
本地构建显式从 `src` 开始发现，仍可构建这两个包。

构建成功后，仅查询本次安装的包和程序，不启动算法：

```bash
source install/local_setup.bash
ros2 pkg prefix fast_lio
ros2 pkg executables fast_lio
```

预期 prefix 指向本目录的 `install/fast_lio`，程序列表包含 `fast_lio fastlio_mapping`。
编译成功不能证明时间协议、参数或建图质量正确。尚未运行建图。

## 2026-11-20 学习副本的后续修改

上面的“尚未运行”是首次构建记录；后续移动包运行记录见学习笔记。
本次仅在学习副本增加`common.reliable_lidar_input`参数：默认`false`保持原PointCloud2
输入的BEST_EFFORT，设为`true`时使用RELIABLE，队列深度仍为5；Livox分支不受该参数影响。
它改变输入订阅接口，不改变滤波、匹配或坐标计算。使用它要求上游提供可靠发布。
退出时增加输入点云回调计数与未处理队列数量，用于核对离线接收；计数上限为源码MAXN。

原源码和原二进制已保存到`../runtime_logs/lesson47/before/`，旧哈希见`before.json`；
原`source_snapshot.json`继续描述首次复制来源，不冒充本次修改后的源码清单。
本次编译日志为`../runtime_logs/lesson47/build.log`，运行快照、实际参数和二进制哈希
见该目录的`run_report.json`与`inputs/`。
