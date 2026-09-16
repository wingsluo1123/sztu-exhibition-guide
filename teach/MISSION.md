# Mission: 能独立修改和扩展这套 ROS2 展厅讲解机器人源码

## Why
我已经会 Python、也知道这个项目是干什么的（SZTU 展厅讲解员机器人），但读代码还停留在"跟着别人讲解走、自己不敢动手改"的阶段。我想达到能自己改这个项目、加新功能的程度——比如改导航逻辑、加一条交互指令、修 bug——而不是每次都要别人带着。

## Success looks like
- 能独立定位并修复项目里的一个真实 bug（例如 `waypoints.yaml` 顶层 key 与 `guide_manager.load_config` 读取不一致）
- 能照着现有模式给 `bridge_server` 新增一条命令路由，并说清它会走快速路径还是慢速路径
- 能说出每个包对外提供/依赖的 ROS 接口（topic/service），以及它们如何串联成一次讲解

## Constraints
- 已有 Python 基础，语法层不重复讲，聚焦代码架构、接口、设计取舍
- 学习分散在多次会话，每次少量、要能记得住（配合间隔复习/检索练习）
- 实际运行环境是 Ubuntu 22.04 + ROS 2 Humble，本机（Windows）不一定能跑 ROS；讲解以读码 + 推理为主

## Out of scope
- 机器人硬件调试、SLAM 算法原理、AimDK 底层实现
- 从头系统学 Python / ROS 2 入门（只在有缺口时补，不整套重讲）
- HarmonyOS 客户端（Robot_APP）的 ArkTS 前端开发
