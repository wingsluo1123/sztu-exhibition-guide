#!/usr/bin/env python3
"""在 RViz 中显示 X2 模型。

用法:
    ros2 launch sztu_description display.launch.py
    ros2 launch sztu_description display.launch.py model:=X2-EDU
    ros2 launch sztu_description display.launch.py gui:=false rviz:=false

model 取值就是 urdf/ 下的文件名（不含 .urdf），由 install_urdf.sh 决定装了哪些。

注意根链接不一致: v1.3.0 的 x2_* 模型根链接是 base_link，v1.4.0 的
X2-* 是 pelvis。rviz/display.rviz 里的 Fixed Frame 按 v1.4.0 设为 pelvis，
用 v1.3.0 模型时在 RViz 左上角 Global Options -> Fixed Frame 改成 base_link。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('sztu_description')

    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')

    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', [model, '.urdf']])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])

    # 走 xacro 而不是直接读文件: 纯 urdf 也能过，以后换 xacro 不用改 launch
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='X2-Ultra',
            description='urdf/ 下的模型文件名，不含 .urdf 后缀',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='true 用 joint_state_publisher_gui 拖滑块，false 用无界面版',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='是否同时启动 RViz'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
