# 学习定位接入导航：接口约定

本文件是待实施方案，不是运行入口，不代表导航已接通。

保留现有地图匹配定位，不并行启动AMCL；不直接调用旧autonomous.launch.py开启第二定位来源。

## 数据和坐标

- 固定地图：本轮复现generated_map，地图参考系learning_map。
- 地图定位：/learning/localization/robot_base_pose，PoseStamped，base_link原点的平面投影。
- 有效性：/learning/localization/status；必须检查valid、地图身份、消息新鲜度，以及同时间机体TF是否可用。
- 连续里程计：需要从同次FAST-LIO输出、当前启动变换及同时间机体运动学得到同一参考点的连续位姿，不能直接将IMU位姿更名为base位姿。
- 计划建立私有learning_nav_odom→learning_nav_base连续TF，以及learning_map→learning_nav_odom地图校正TF。两者乘积须与定位输出一致。
- 两套位姿必须对应同一时刻和同一参考点；不能混用历史机器人姿态或将时间戳改成当前时间掩盖延迟。
- 定位失效或输入过期时，不以旧位姿继续声明可导航；控制禁行状态需要显式传递。TF本身没有有效状态字段。

## 现有接口尚不能直接复用的部分

- 现有安全网关依赖/amcl_pose、AMCL更新服务和/sztu/nav_odom；接入学习定位需要明确适配有效性，不能伪造协方差冒充AMCL。
- 旧Nav2全局配置allow_unknown=true且track_unknown_space=false。第一次已知空闲区域内的短距离试验应保留未知、禁止穿过未知，不修改地图像素。
- 暂定矩形轮廓不能与现有不同robot_radius参数不一致地各算各的；代价地图、规划碰撞检查、停止门槛需统一几何并确认摆臂覆盖。
- 地图接入与规划先在无执行器输出模式验证；随后才核查实时障碍物、单一控制源、停止链路并进行现场运动。

## 当前完成范围

step11仅验证平面坐标连接的数学关系，未实现在线适配、启动Nav2或发运动指令。
