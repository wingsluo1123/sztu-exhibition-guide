#!/usr/bin/env python3
"""
Launch file: remote-control mode for the HarmonyOS northbound client.

Starts SLAM + navigation + the northbound bridge server. The bridge hosts the
TCP (and optional WebSocket) server the Robot_APP client connects to, and it
owns the interaction controllers (TTS / emotion / gesture) in-process. For that
reason the standalone tts/emotion/gesture nodes are intentionally NOT started
here — starting both would collide on node names.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port', default_value='8888',
        description='TCP port the northbound client connects to'
    )
    ws_enabled_arg = DeclareLaunchArgument(
        'ws_enabled', default_value='false',
        description='Enable the optional WebSocket server (requires the "websockets" package)'
    )
    ws_port_arg = DeclareLaunchArgument(
        'ws_port', default_value='8080',
        description='WebSocket port when ws_enabled is true'
    )

    slam_node = Node(
        package='sztu_slam', executable='slam_manager',
        name='slam_manager', output='screen'
    )
    pose_monitor_node = Node(
        package='sztu_slam', executable='pose_monitor',
        name='pose_monitor', output='screen'
    )
    navigation_node = Node(
        package='sztu_navigation', executable='navigation_manager',
        name='navigation_manager', output='screen'
    )

    bridge_node = Node(
        package='sztu_guide', executable='bridge_server',
        name='bridge_server', output='screen',
        parameters=[{
            'tcp_port': LaunchConfiguration('tcp_port'),
            'ws_enabled': LaunchConfiguration('ws_enabled'),
            'ws_port': LaunchConfiguration('ws_port'),
        }]
    )

    return LaunchDescription([
        tcp_port_arg,
        ws_enabled_arg,
        ws_port_arg,
        slam_node,
        pose_monitor_node,
        navigation_node,
        bridge_node,
    ])
