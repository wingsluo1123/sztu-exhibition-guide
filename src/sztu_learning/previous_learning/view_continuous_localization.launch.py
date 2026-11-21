"""Fixed map, continuous tracker and optional bounded archive replay; computer only."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = Path(__file__).resolve().parent
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('replay', default_value='true'),
        DeclareLaunchArgument('archive', default_value=str(root / 'maps/localization_20261120_run02')),
        DeclareLaunchArgument('frame_config', default_value=str(root / 'runtime_logs/continuous_independent_accepted_20261120/frame_config.json')),
        DeclareLaunchArgument('seed_x', default_value='.3'),
        DeclareLaunchArgument('seed_y', default_value='-.2'),
        DeclareLaunchArgument('seed_yaw_degrees', default_value='-22.3'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root / 'view_learning_map.launch.py')),
                                 launch_arguments={'map_directory': str(root / 'maps/room_7x3_20261120_map01'),
                                                   'rviz': LaunchConfiguration('rviz'),
                                                   'rviz_config': str(root / 'continuous_localization.rviz')}.items()),
        ExecuteProcess(cmd=[sys.executable, str(root / 'continuous_localization_node.py'),
                            '--map-directory', str(root / 'maps/room_7x3_20261120_map01'),
                            '--frame-config', LaunchConfiguration('frame_config'), '--seed',
                            LaunchConfiguration('seed_x'), LaunchConfiguration('seed_y'),
                            LaunchConfiguration('seed_yaw_degrees'), '--surface-reference',
                            str(root / 'maps/room_7x3_20261120_surfaces02'),
                            '--base-transforms', str(root / 'runtime_logs/continuous_base_tf_20261120'),
                            '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(root / 'replay_localization_archive.py'),
                            '--archive', LaunchConfiguration('archive')],
                       condition=IfCondition(LaunchConfiguration('replay')), output='screen'),
    ])
