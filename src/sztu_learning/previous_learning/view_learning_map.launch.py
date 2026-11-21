"""Computer/offline only: map server + lifecycle activation + saved path + RViz."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parent
    map_directory=LaunchConfiguration('map_directory')
    rviz=Node(package='rviz2',executable='rviz2',name='learning_map_view',output='screen',
              arguments=['-d',LaunchConfiguration('rviz_config')],condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('rviz_config',default_value=str(root/'learning_map.rviz')),
        DeclareLaunchArgument('map_directory',default_value=str(root/'maps/lesson58_rviz_map_v2')),
        Node(package='nav2_map_server',executable='map_server',name='learning_map_server',output='screen',
             parameters=[{'yaml_filename':PathJoinSubstitution([map_directory,'map.yaml']),'frame_id':'learning_map','topic_name':'/map','use_sim_time':False}]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='learning_map_lifecycle',output='screen',
             parameters=[{'autostart':True,'node_names':['learning_map_server'],'use_sim_time':False}]),
        rviz,
        RegisterEventHandler(OnProcessExit(target_action=rviz,on_exit=[EmitEvent(event=Shutdown(reason='Map viewer closed'))])),
        # An ExecuteProcess keeps this standalone learning directory independent
        # of the original project's installed Python packages.
        ExecuteProcess(
            cmd=[sys.executable,str(root/'map_trajectory_overlay.py'),'--map-directory',map_directory],output='screen'),
    ])
