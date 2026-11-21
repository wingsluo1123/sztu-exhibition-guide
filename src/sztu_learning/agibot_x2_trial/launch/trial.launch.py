"""Perception/localization and optional Nav2; NEVER starts a motor bridge.

Nav2 output is isolated on /trial/nav_cmd_vel even on a live robot domain.
"""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]


def generate_launch_description():
    clock = LaunchConfiguration('use_sim_time')
    reliable = LaunchConfiguration('reliable_input')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('reliable_input', default_value='true'),
        DeclareLaunchArgument('nav2', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ROOT/'launch/sensors.launch.py')),
                                 launch_arguments={'use_sim_time': clock, 'reliable_input': reliable}.items()),
        ExecuteProcess(cmd=[sys.executable, str(ROOT/'scripts/kinematic_odom.py'),
                            '--ros-args', '-p', ['use_sim_time:=', clock]], output='screen'),
        Node(package='spark_fast_lio', executable='spark_lio_mapping', namespace='lio',
             parameters=[str(ROOT/'config/odometry.yaml'), {'use_sim_time': clock}],
             remappings=[('lidar','/lidar/points_rectified'), ('imu','/imu/rectified')], output='screen'),
        Node(package='lidar_localization_ros2', executable='lidar_localization_node',
             parameters=[str(ROOT/'config/localization.yaml'), {'use_sim_time': clock}],
             remappings=[('cloud','/lio/cloud_registered_lidar'), ('map','/pcd_map_input')], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='trial_localization_lifecycle', output='screen',
             parameters=[{'use_sim_time':clock, 'autostart':True,
                          'node_names':['lidar_localization'], 'bond_timeout':0.0}]),
        GroupAction(condition=IfCondition(LaunchConfiguration('nav2')), actions=[
            SetRemap(src='/cmd_vel', dst='/trial/nav_cmd_vel'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory('my_nav2_pointcloud_pkg'))/'launch/nav2_bringup.launch.py')),
                launch_arguments={'map':str(ROOT/'maps/mapping02/map.yaml'),
                                  'params_file':str(ROOT/'config/nav2.yaml'),
                                  'use_sim_time':clock}.items()),
        ]),
    ])
