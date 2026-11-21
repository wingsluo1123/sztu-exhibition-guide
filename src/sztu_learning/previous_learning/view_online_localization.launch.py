"""Computer: fixed map plus read-only live robot display over SSH."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = Path(__file__).resolve().parent
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('robot', default_value='agi@192.168.224.19'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root / 'view_learning_map.launch.py')),
                                 launch_arguments={'map_directory': str(root / 'maps/room_7x3_20261120_map01'),
                                                   'rviz': LaunchConfiguration('rviz'),
                                                   'rviz_config': str(root / 'continuous_localization.rviz')}.items()),
        ExecuteProcess(cmd=[sys.executable, str(root / 'online_display_bridge.py'), '--robot', LaunchConfiguration('robot')], output='screen'),
    ])
