"""Known-map Nav2 bringup; hardware output requires explicit live mode."""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def merge_parameters(target, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            merge_parameters(target[key], value)
        else:
            target[key] = value


def build_nodes(context):
    package_share = get_package_share_directory('sztu_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')
    map_path = os.path.abspath(LaunchConfiguration('map').perform(context))
    if not os.path.isfile(map_path):
        raise RuntimeError(f'Map YAML missing: {map_path}')
    dds_profile = LaunchConfiguration('dds_profile').perform(context)
    if not os.path.isfile(dds_profile):
        raise RuntimeError(f'DDS profile missing: {dds_profile}')
    with open(os.path.join(nav2_share, 'params', 'nav2_params.yaml')) as stream:
        parameters = yaml.safe_load(stream)
    with open(os.path.join(package_share, 'config', 'nav2_params.yaml')) as stream:
        merge_parameters(parameters, yaml.safe_load(stream))
    parameters['map_server']['ros__parameters']['yaml_filename'] = map_path
    navigator = parameters['bt_navigator']['ros__parameters']
    navigator['default_nav_to_pose_bt_xml'] = os.path.join(
        package_share, 'config', 'navigate_wait.xml')
    navigator['default_nav_through_poses_bt_xml'] = os.path.join(
        package_share, 'config', 'navigate_poses_wait.xml')
    parameters['behavior_server']['ros__parameters'].update({
        'global_frame': 'sztu_nav_odom', 'robot_base_frame': 'sztu_nav_base'})
    parameters['controller_server']['ros__parameters']['odom_topic'] = '/sztu/nav_odom'
    parameters['controller_server']['ros__parameters']['progress_checker'].update({
        'required_movement_radius': 0.10, 'movement_time_allowance': 12.0})
    parameters['controller_server']['ros__parameters']['goal_checker_plugins'] = ['goal_checker']
    parameters['controller_server']['ros__parameters']['goal_checker'] = {
        'plugin': 'nav2_controller::SimpleGoalChecker', 'stateful': False,
        'xy_goal_tolerance': 0.30, 'yaw_goal_tolerance': 0.26}
    parameters['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan'].update({
        'topic': '/sztu/safety_scan',
        'observation_persistence': 0.0, 'expected_update_rate': 0.5,
        'inf_is_valid': False})
    parameters['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan'].update({
        'topic': '/sztu/safety_scan',
        'observation_persistence': 0.0, 'expected_update_rate': 0.5,
        'inf_is_valid': False})
    descriptor, params_path = tempfile.mkstemp(prefix='sztu_nav2_', suffix='.yaml')
    with os.fdopen(descriptor, 'w') as stream:
        yaml.safe_dump(parameters, stream)
    simulation = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    common = [params_path, {'use_sim_time': simulation}]
    processes = []
    lifecycle_groups = {'sztu_localization_lifecycle': [], 'sztu_nav_lifecycle': []}
    for name, package, executable in (
        ('map_server', 'nav2_map_server', 'map_server'),
        ('amcl', 'nav2_amcl', 'amcl'),
        ('planner_server', 'nav2_planner', 'planner_server'),
        ('controller_server', 'nav2_controller', 'controller_server'),
        ('behavior_server', 'nav2_behaviors', 'behavior_server'),
        ('bt_navigator', 'nav2_bt_navigator', 'bt_navigator'),
        ('velocity_smoother', 'nav2_velocity_smoother', 'velocity_smoother'),
    ):
        remappings = [('cmd_vel', '/cmd_vel_nav_raw')]
        if name == 'velocity_smoother':
            remappings = [('cmd_vel', '/cmd_vel_nav_raw'),
                          ('cmd_vel_smoothed', '/cmd_vel_nav')]
        if name == 'amcl':
            remappings.append(('initialpose', '/sztu/initialpose_retimed'))
            remappings.append(('map', '/sztu/map'))
        environment = {}
        if name == 'map_server':
            map_dds_profile = os.path.join(
                package_share, 'config', 'fastdds_map_wifi.xml')
            environment['FASTDDS_DEFAULT_PROFILES_FILE'] = map_dds_profile
            environment['FASTRTPS_DEFAULT_PROFILES_FILE'] = map_dds_profile
        processes.append(Node(package=package, executable=executable, name=name,
                              parameters=common, output='screen',
                              remappings=remappings, additional_env=environment))
        manager = ('sztu_localization_lifecycle' if name in ('map_server', 'amcl')
                   else 'sztu_nav_lifecycle')
        lifecycle_groups[manager].append(name)
    for manager, names in lifecycle_groups.items():
        processes.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name=manager, output='screen',
            parameters=[{'use_sim_time': simulation, 'autostart': True,
                         'node_names': names}]))
    processes.append(Node(
        package='sztu_navigation', executable='nav_sensor_adapter',
        parameters=[{'use_sim_time': simulation,
                     'cloud_topic': LaunchConfiguration('cloud_topic')}], output='screen'))
    processes.append(Node(
        package='sztu_navigation', executable='initialpose_retimer',
        parameters=[{'use_sim_time': simulation}], output='screen'))
    processes.append(Node(
        package='sztu_navigation', executable='safety_gateway', output='screen',
        parameters=[{'use_sim_time': simulation,
                     'live': ParameterValue(LaunchConfiguration('live'), value_type=bool),
                     'surround_coverage_verified': ParameterValue(
                         LaunchConfiguration('surround_coverage_verified'), value_type=bool),
                     'front_only_coverage': ParameterValue(
                         LaunchConfiguration('front_only_coverage'), value_type=bool),
                     'front_arc_coverage': ParameterValue(
                         LaunchConfiguration('front_arc_coverage'), value_type=bool),
                     'robot_radius': ParameterValue(
                         LaunchConfiguration('robot_radius'), value_type=float),
                     'stopping_margin': ParameterValue(
                         LaunchConfiguration('stopping_margin'), value_type=float)}]))
    processes.append(RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda unused: remove_parameters(params_path))])))
    return processes


def remove_parameters(path):
    if os.path.exists(path):
        os.unlink(path)
    return []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dds_profile',
            default_value=os.path.join(get_package_share_directory('sztu_slam'),
                                       'config', 'fastdds_large_cloud.xml'),
            description='Validated Fast DDS transport profile for navigation nodes'),
        SetEnvironmentVariable(
            'FASTDDS_DEFAULT_PROFILES_FILE',
            LaunchConfiguration('dds_profile')),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            LaunchConfiguration('dds_profile')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),
        DeclareLaunchArgument('map', description='Absolute path of the existing map YAML'),
        DeclareLaunchArgument('cloud_topic',
                              default_value='/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
                              description='Timestamped sensor cloud with a valid TF chain'),
        DeclareLaunchArgument('live', default_value='false',
                              description='Enable AimDK hardware output after commissioning'),
        DeclareLaunchArgument('surround_coverage_verified', default_value='false',
                              description='Requires validated sensors covering the whole footprint'),
        DeclareLaunchArgument('front_only_coverage', default_value='false',
                              description='Use a validated 120-degree front sensor; straight motion only'),
        DeclareLaunchArgument('front_arc_coverage', default_value='false',
                              description='Use a validated 120-degree front sensor; '
                                          'rotation and clamped arcs allowed'),
        DeclareLaunchArgument('robot_radius', default_value='0.30',
                              description='Footprint radius shared with the Nav2 costmaps'),
        DeclareLaunchArgument('stopping_margin', default_value='0.25',
                              description='Coasting distance added beyond the footprint radius'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use a published simulation clock'),
        OpaqueFunction(function=build_nodes),
    ])
