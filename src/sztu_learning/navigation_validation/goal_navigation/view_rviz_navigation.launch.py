"""Computer/domain42: exact saved map, live SSH display, optional one RViz goal."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = Path(__file__).resolve().parent
    map_directory = here.parents[1]/'previous_learning/maps/room_7x3_20261120_map01'
    rviz = Node(package='rviz2', executable='rviz2', name='navigation_validation_rviz',
                arguments=['-d', str(here/'goal_navigation.rviz')], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='agi@192.168.224.19'),
        Node(package='nav2_map_server', executable='map_server', name='navigation_validation_map_server',
             parameters=[dict(yaml_filename=str(map_directory/'map.yaml'), frame_id='learning_map',
                              topic_name='/learning/nav/saved_map', use_sim_time=False)], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='navigation_validation_map_lifecycle', parameters=[dict(autostart=True,
                 node_names=['navigation_validation_map_server'], use_sim_time=False)], output='screen'),
        ExecuteProcess(cmd=[sys.executable,str(here/'rviz_bridge.py'),'--robot',LaunchConfiguration('robot'),
                            '--forward-one-goal','--suggest-relative-goal','0.6','-0.4'], output='screen'),
        rviz,
        RegisterEventHandler(OnProcessExit(target_action=rviz, on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))])),
    ])
