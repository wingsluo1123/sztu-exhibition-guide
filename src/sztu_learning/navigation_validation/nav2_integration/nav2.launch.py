"""Namespaced official Nav2; outputs private Twist only, no motor node."""
import os
from pathlib import Path
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent/'goal_navigation'))
from parameters import build_parameters, NAMESPACE
from grid_route import GridRoute


def nodes(context):
    map_dir = Path(LaunchConfiguration('map_directory').perform(context)).resolve()
    GridRoute.load(map_dir, .40)  # Verify frozen map lineage, never regenerate.
    defaults = Path(get_package_share_directory('nav2_bringup'))/'params/nav2_params.yaml'
    cfg = build_parameters(defaults, map_dir, HERE/'navigate_once.xml')
    fd, file = tempfile.mkstemp(prefix='learning_nav2_', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        yaml.safe_dump(cfg, stream)
    names = ['map_server', 'planner_server', 'controller_server', 'bt_navigator']
    packages = ['nav2_map_server', 'nav2_planner', 'nav2_controller', 'nav2_bt_navigator']
    actions = [TimerAction(period=.75*index, actions=[Node(package=package, executable=name, name=name, namespace=NAMESPACE,
                    parameters=[file], output='screen',
                    remappings=[('/tf', '/learning_nav2/tf'),
                                ('/tf_static', '/learning_nav2/tf_static'),
                                ('cmd_vel', '/learning_nav2/cmd_vel_raw')])])
               for index,(name, package) in enumerate(zip(names, packages))]
    actions.append(TimerAction(period=3.,actions=[Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                        name='lifecycle_manager', namespace=NAMESPACE,
                        parameters=[dict(autostart=True, use_sim_time=False, node_names=names)],
                        output='screen')]))
    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda _: cleanup(file))])))
    return actions


def cleanup(file):
    Path(file).unlink(missing_ok=True)
    return []


def generate_launch_description():
    root = Path(os.environ.get('LEARNING_LOCALIZATION_ROOT',
                              str(HERE.parents[1]/'previous_learning')))
    return LaunchDescription([
        DeclareLaunchArgument('map_directory',
                              default_value=str(root/'maps/room_7x3_20261120_map01')),
        OpaqueFunction(function=nodes)])
