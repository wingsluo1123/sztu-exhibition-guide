"""Installed Humble Nav2 plugins, existing map and commissioning envelope."""
from copy import deepcopy
from pathlib import Path

import yaml

NAMESPACE = 'learning_nav2'


def build_parameters(defaults, map_directory, tree):
    source = yaml.safe_load(Path(defaults).read_text())
    result = {name: deepcopy(source[name]) for name in (
        'map_server', 'planner_server', 'controller_server', 'bt_navigator',
        'global_costmap', 'local_costmap')}
    result['map_server']['ros__parameters'].update(
        yaml_filename=str(Path(map_directory)/'map.yaml'),
        frame_id='learning_map', topic_name='/learning/nav/saved_map')
    result['planner_server']['ros__parameters'].update(
        expected_planner_frequency=1., planner_plugins=['GridBased'],
        GridBased=dict(plugin='nav2_navfn_planner/NavfnPlanner',
                       use_astar=True, allow_unknown=False, tolerance=.05))
    result['controller_server']['ros__parameters'] = dict(
        use_sim_time=False, controller_frequency=20.,
        min_x_velocity_threshold=.001, min_y_velocity_threshold=.001,
        min_theta_velocity_threshold=.001, failure_tolerance=0.,
        odom_topic='/learning_nav2/odom',
        progress_checker_plugin='progress_checker',
        progress_checker=dict(plugin='nav2_controller::SimpleProgressChecker',
                              required_movement_radius=.03, movement_time_allowance=15.),
        goal_checker_plugins=['goal_checker'],
        goal_checker=dict(plugin='nav2_controller::SimpleGoalChecker',
                          stateful=True, xy_goal_tolerance=.15, yaw_goal_tolerance=.10),
        controller_plugins=['FollowPath'],
        FollowPath=dict(
            plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            desired_linear_vel=.15, min_approach_linear_velocity=.15,
            approach_velocity_scaling_dist=.30,
            lookahead_dist=.35, min_lookahead_dist=.20, max_lookahead_dist=.50,
            use_velocity_scaled_lookahead_dist=False,
            use_rotate_to_heading=True, rotate_to_heading_min_angle=.15,
            rotate_to_heading_angular_vel=.70, max_angular_accel=1.,
            allow_reversing=False, use_collision_detection=True,
            max_allowed_time_to_collision_up_to_carrot=1.,
            use_regulated_linear_velocity_scaling=False,
            use_cost_regulated_linear_velocity_scaling=False,
            regulated_linear_scaling_min_speed=.05, transform_tolerance=.35))
    bt = result['bt_navigator']['ros__parameters']
    bt.update(global_frame='learning_map', robot_base_frame='learning_nav_base',
              odom_topic='/learning_nav2/odom',
              default_nav_to_pose_bt_xml=str(tree),
              default_nav_through_poses_bt_xml=str(Path(tree).with_name('navigate_poses_once.xml')),
              plugin_lib_names=['nav2_compute_path_to_pose_action_bt_node',
                                'nav2_compute_path_through_poses_action_bt_node',
                                'nav2_follow_path_action_bt_node'])
    # One ComputePath followed by one FollowPath: no periodic goal-checker reset,
    # no automatic retry, spin, backup or timeout-based mission completion.
    for name in ('global_costmap', 'local_costmap'):
        cfg = result[name][name]['ros__parameters']
        cfg.clear()
        cfg.update(use_sim_time=False, global_frame='learning_map',
                   robot_base_frame='learning_nav_base', transform_tolerance=.35,
                   update_frequency=5., publish_frequency=2.,
                   rolling_window=False, resolution=.05, robot_radius=.40,
                   track_unknown_space=True, always_send_full_costmap=True,
                   plugins=['static_layer', 'inflation_layer'],
                   static_layer=dict(plugin='nav2_costmap_2d::StaticLayer',
                                     map_topic='/learning/nav/saved_map',
                                     map_subscribe_transient_local=True),
                   inflation_layer=dict(plugin='nav2_costmap_2d::InflationLayer',
                                        inflation_radius=.55, cost_scaling_factor=5.))
    for name, cfg in result.items():
        if 'ros__parameters' in cfg:
            cfg['ros__parameters']['use_sim_time'] = False
    return {NAMESPACE: result}
