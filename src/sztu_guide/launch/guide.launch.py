#!/usr/bin/env python3
"""Launch 文件：启动讲解系统的各个子系统（**不含 guide_manager**）

**这个 launch 和 `ros2 run sztu_guide guide_manager` 是二选一，不要同时跑。**
guide_manager 在进程内自己 new 了 SlamManager / NavigationManager /
TtsController / EmotionController / GestureController 全套（见
guide_manager.py:58-71）。两边一起起，上面每个节点都会有两个实例，其中
**两个 NavigationManager 会用同一个名字 sdk_sztu_nav、同一个优先级 45
去注册 AimDK 走跑输入源，并且同时下发速度** —— 这是实走时最危险的一种
重复，机器人会收到两路互相打架的速度指令。

所以实走只用一条命令：
    ros2 run sztu_guide guide_manager
空跑（不下发速度，只看日志）：
    ros2 run sztu_guide guide_manager --ros-args -p dry_run:=true
（实测过：不带节点名的 -p 会透进进程内自建的每个子节点，包括
 navigation_manager。）

这个 launch 留给"只想起子系统、自己写上层逻辑"的场景。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明参数
    language_arg = DeclareLaunchArgument(
        'language',
        default_value='zh_CN',
        description='Language for the tour (zh_CN or en_US)'
    )
    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='true 时导航照常算控制量但不下发速度，机器人不动'
    )

    # SLAM 管理器
    slam_node = Node(
        package='sztu_slam',
        executable='slam_manager',
        name='slam_manager',
        output='screen'
    )

    # 位姿监控
    pose_monitor_node = Node(
        package='sztu_slam',
        executable='pose_monitor',
        name='pose_monitor',
        output='screen'
    )

    # 导航管理器
    navigation_node = Node(
        package='sztu_navigation',
        executable='navigation_manager',
        name='navigation_manager',
        output='screen',
        parameters=[{'dry_run': LaunchConfiguration('dry_run')}]
    )

    # TTS 控制器
    tts_node = Node(
        package='sztu_interaction',
        executable='tts_controller',
        name='tts_controller',
        output='screen'
    )

    # 表情控制器
    emotion_node = Node(
        package='sztu_interaction',
        executable='emotion_controller',
        name='emotion_controller',
        output='screen'
    )

    # 手势控制器
    gesture_node = Node(
        package='sztu_interaction',
        executable='gesture_controller',
        name='gesture_controller',
        output='screen'
    )

    # 讲解员核心不在这里起：guide_manager 的 main() 是交互式菜单（要在终端
    # 里按 1/2 选语言），由 launch 托管的话拿不到 stdin。而且它自建全套子系统
    # ——见本文件开头，两边同时起会出现两个 NavigationManager 抢同一个走跑
    # 输入源。实走请只用 ros2 run sztu_guide guide_manager。

    return LaunchDescription([
        language_arg,
        dry_run_arg,
        slam_node,
        pose_monitor_node,
        navigation_node,
        tts_node,
        emotion_node,
        gesture_node,
    ])
