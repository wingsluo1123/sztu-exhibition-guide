"""Upstream rectifiers plus their explicitly named, rotated coordinate frames."""
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    clock = LaunchConfiguration('use_sim_time')
    reliable = LaunchConfiguration('reliable_input')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('reliable_input', default_value='true'),
        Node(package='py_examples', executable='pcl_rectifier', output='screen',
             parameters=[{'use_sim_time': clock, 'reliable_input': reliable}]),
        Node(package='py_examples', executable='imu_rectifier', output='screen',
             parameters=[{'use_sim_time': clock, 'reliable_input': reliable}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='lidar_rectified_tf',
             arguments=['--roll', str(math.pi/2), '--frame-id', 'lidar_chest_front',
                        '--child-frame-id', 'lidar_rectified']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='imu_rectified_tf',
             arguments=['--pitch', str(math.pi/2), '--frame-id', 'lidar_imu_chest_front',
                        '--child-frame-id', 'imu_rectified']),
    ])
