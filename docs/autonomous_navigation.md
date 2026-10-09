# 自主导航启动

当前导航使用自研 FAST-LIO + Nav2 + AMCL；厂商 SLAM 不参与。

## 启动顺序

1. 停掉会抢速度的旧服务：

   ```bash
   systemctl --user stop sztu-bridge
   ```

2. 终端一启动实时里程计和点云（保持运行）：

   ```bash
   cd ~/sztu
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   tools/run_lio_live.sh --bridge --map-frame maps/testroom.yaml
   ```

3. 终端二启动已知地图导航（默认 `live:=false`，不会发硬件指令）：

   ```bash
   cd ~/sztu
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch sztu_navigation autonomous.launch.py \
     map:=/home/agi/sztu/maps/testroom.yaml \
     live:=false
   ```

4. 在 RViz 设置一次 `/initialpose`，必须是机器人当前在 `testroom` 地图中的真实位置和朝向。等 `/amcl_pose` 协方差稳定后再发导航目标或启动讲解点位。

## 真机放行条件

胸前 E1R 只有约 120° 水平视场；在没有确认侧后方传感器前，不要把 `live` 改为 `true`。安全门默认要求新鲜点云、里程计、AMCL 位姿和 `map → sztu_nav_base` TF，任一失效就输出零速度。

完成实测并确认传感器覆盖整个机器人扫掠范围后，才可由现场人员执行：

```bash
ros2 launch sztu_navigation autonomous.launch.py \
  map:=/home/agi/sztu/maps/testroom.yaml \
  live:=true surround_coverage_verified:=true
```

仍需现场人员在急停旁边先做静止、低速、单点和动态障碍测试。`Spin`、`BackUp` 恢复动作没有启用；遇到人或传感器断流会等待并重新规划。

## 验证

开发机仿真（隔离域 `42`）已验证：地图定位、动态障碍绕行、障碍移开后重新规划、传感器断流停车和 Nav2 action 取消。测试命令：

```bash
ROS_DOMAIN_ID=42 tools/run_tests.sh
ROS_DOMAIN_ID=42 python3 tools/test_nav2_runtime.py
```

2026-10-09 软件测试：704 通过、2 跳过。最后一次仿真到点误差 0.291 m，绕行横向偏移 1.537 m，距测试障碍最小中心净空 1.180 m。随后验证了阻断路线等待、障碍移除后重新规划、传感器断流期间拒绝新鲜速度指令。这些结果仅来自合成地图、合成全向点云和理想速度响应。

速度上限 0.25 m/s、0.7 rad/s。规划半径 0.85 m 包含保守裕量；真实机器人半径 0.40 m、额外裕量 0.15 m、滑行预留 0.25 m 仍需现场测量。AMCL 的位置方差上限 0.09 m²、航向方差上限 0.07 rad² 是初始门槛，低协方差不能单独证明定位正确，必须核对地图中的墙和机器人真实位置。

真机已安装 Nav2 1.1.18 并编译导航和讲解包，默认 `live=false` 节点构造通过；开发机 Nav2 为 1.1.20。真机未做自主运动、制动距离、现有地图 AMCL 匹配和侧后方覆盖验证。完整的人流环境自主导航尚未验收。
