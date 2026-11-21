"""Computer domain42: exact frozen map plus one-goal SSH relay to robot Nav2."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parent.parent
    map_dir=root.parent/'previous_learning/maps/room_7x3_20261120_map01'
    rviz=Node(package='rviz2',executable='rviz2',name='learning_nav2_rviz',
              arguments=['-d',str(root/'goal_navigation/goal_navigation.rviz')],output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('robot',default_value='agi@192.168.224.19'),
        DeclareLaunchArgument('remote_script',default_value='/home/agi/sztu_learning/navigation_validation/nav2_release08/goal_navigation/rviz_bridge.py'),
        DeclareLaunchArgument('restrict_trial_goal',default_value='false',
                             description='Show and enforce the old front-right trial selection area'),
        Node(package='nav2_map_server',executable='map_server',name='learning_nav2_view_map',
             parameters=[dict(yaml_filename=str(map_dir/'map.yaml'),frame_id='learning_map',
                              topic_name='/learning/nav/saved_map')],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='learning_nav2_view_lifecycle',
             parameters=[dict(autostart=True,node_names=['learning_nav2_view_map'])],output='screen'),
        ExecuteProcess(cmd=[sys.executable,str(root/'goal_navigation/rviz_bridge.py'),
            '--robot',LaunchConfiguration('robot'),'--remote-script',LaunchConfiguration('remote_script'),
            '--validate-map-directory',str(map_dir),
            '--forward-one-goal','--selection-mode',PythonExpression([
                "'trial' if '",LaunchConfiguration('restrict_trial_goal'),"'.lower() == 'true' else 'map'"])],output='screen'),
        rviz,RegisterEventHandler(OnProcessExit(target_action=rviz,on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))]))])
