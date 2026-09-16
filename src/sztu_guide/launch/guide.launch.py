#!/usr/bin/env python3
"""
Launch 文件：启动完整讲解系统
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明参数
    language_arg = DeclareLaunchArgument(
        'language',
        default_value='zh_CN',
        description='Language for the tour (zh_CN or en_US)'
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
        output='screen'
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
    
    # 讲解员核心（稍后手动启动）
    # guide_node = Node(
    #     package='sztu_guide',
    #     executable='guide_manager',
    #     name='guide_manager',
    #     output='screen',
    #     parameters=[{'language': LaunchConfiguration('language')}]
    # )
    
    return LaunchDescription([
        language_arg,
        slam_node,
        pose_monitor_node,
        navigation_node,
        tts_node,
        emotion_node,
        gesture_node,
    ])
