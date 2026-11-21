# livox_ros_driver2（接口占位包）

**这个包不是 Livox 的驱动。** 它只有两个消息定义，一行代码都没有。

## 为什么它存在

FAST-LIO2 的 `CMakeLists.txt` 里有一行：

```cmake
find_package(livox_ros_driver2 REQUIRED)
```

因为它支持 Livox 的 AVIA / HORIZON / MID360，那几款雷达用的是 Livox 私有的
`CustomMsg`，不是标准 `sensor_msgs/PointCloud2`。

我们用的是 X2 胸前那颗 96 线雷达，发的是标准 `PointCloud2`，走的是
`lidar_type: 2`（velodyne 分支），**永远不会执行 AVIA 那条代码路径**。
但 `REQUIRED` 是编译期的事，不管运行时走不走，这个依赖必须能被满足。

## 为什么不改 FAST_LIO 的源码

把这个依赖删掉需要动 4 个文件（`CMakeLists.txt`、`src/preprocess.h`、
`src/preprocess.cpp`、`src/laserMapping.cpp`），改动本身不难。没走这条路，
是因为：

- FAST_LIO 是 **GPLv2**。让它的源码保持字节不变，我们既不用维护 fork，
  也不用随本仓库分发它的任何代码或补丁，许可证边界干净。
- 上游哪天更新，我们不会有冲突要解。
- 编译期依赖的问题，用编译期的包来解，比改别人的源码更小。

## 为什么不装真的 Livox 驱动

真的 `livox_ros_driver2` 依赖 Livox-SDK2（另一个 cmake 项目），而且它的
`build.sh` 会覆写自己的 `package.xml`。为了一条我们用不到的代码路径拉进这些
东西，不划算。

## 如果哪天真要接 Livox 雷达

把这个目录删掉，按 Livox 官方文档装真的 `livox_ros_driver2`。两者包名相同，
**不能同时存在于一个 workspace**。`tools/setup_fastlio.sh` 会先检查系统里
有没有真的驱动，有就不再用这个占位包。

## 目录名和包名不一致是故意的

目录叫 `livox_ros_driver2_stub`，`package.xml` 里的 `<name>` 是
`livox_ros_driver2`。colcon 按 `package.xml` 认包名，所以这样能工作；
目录名带 `_stub` 是为了让人一眼看出它不是真货。
