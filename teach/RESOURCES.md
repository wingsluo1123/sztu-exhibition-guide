# 本项目（ROS 2 Humble + rclpy）Resources

> 说明：本项目的"知识"分两层——通用 ROS 2 / rclpy 架构知识 + 本项目专属的 AimDK 接口。通用层有官方文档，专属层目前是 gap。

## Knowledge

- [ROS 2 Humble 官方文档](https://docs.ros.org/en/humble/index.html)
  权威入口。查任何 ROS 2 概念/API 都先来这里。

- [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
  rclpy 节点骨架（Node + create_publisher/create_subscription + spin）的官方最小范例。本项目每个节点都是这套骨架的扩展。使用场景：理解/复刻某个节点的 pub/sub 结构。

- [Writing a simple service and client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
  rclpy 的 create_client + call_async 模式。本项目 tts/emotion/gesture/slam 全是 service client 封装。使用场景：理解为什么它们要 wait_for_service + spin_until_future_complete。

- [About Quality of Service settings](https://docs.ros.org/en/ros2_documentation/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
  QoS（reliability/durability/history/depth）的官方说明。本项目里 BEST_EFFORT vs RELIABLE、TRANSIENT_LOCAL 的取舍都源自这里。使用场景：看懂 slam_manager 和 gesture_controller 里显式写 QoS 的动机。

- [Colcon Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
  colcon 构建/install 空间、setup.py entry_points、ament_python 包结构。使用场景：理解"改完代码为什么要 colcon build + source"、launch 文件为什么必须放包内 data_files。

## Wisdom (Communities)

- [ROS Answers](https://answers.ros.org/)
  ROS 官方问答站，质量高、有维护者出没。使用场景：报错排查、架构疑问。本项目的 AimDK 专属问题大概率搜不到，通用 rclpy 问题可查。

## Gaps

- **AimDK 接口文档（aimdk_msgs 的 .msg/.srv 定义）**：本项目依赖的 PlayTts / PlayEmoji / SetMcPresetMotion / GetStoredMapByName 等服务的字段语义，没有公开文档。目前只能靠源码里的 request 字段 + 注释反推。需要：机器人厂商 SDK 文档，或本机 aimdk_msgs 的接口定义文件。
