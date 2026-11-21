"""Read-only live Nav2 parameter/lifecycle/input probe; no action clients."""
import json
import sys
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String

sys.path.insert(0, '/home/agi/sztu_learning/navigation_validation/nav2_release08')
from navigation_observation import decode_observation
from planar_math import matrix, pose
from goal_navigation.input_readiness import MAP_HASH

rclpy.init()
n = Node('learning_nav2_readonly_preflight')
report = dict(epoch=time.time(), no_action_or_motor=True, parameters={}, lifecycle={})
observed = []
def receive(m):
    status, local, transform = decode_observation(m.data)
    if status['valid']:
        observed.append(dict(status=status, local=local,
                             map_pose=pose(transform @ matrix(local)).tolist(),
                             age=n.get_clock().now().nanoseconds*1e-9-status['stamp']))
n.create_subscription(String, '/learning/nav/observation', receive, 1)
def call(service, type_, request):
    c = n.create_client(type_, service)
    if not c.wait_for_service(timeout_sec=5.):
        raise RuntimeError('missing '+service)
    f = c.call_async(request)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5.)
    if not f.done():
        raise RuntimeError('timeout '+service)
    response=f.result(); n.destroy_client(c)
    return response
try:
    names=['FollowPath.plugin','FollowPath.max_angular_accel',
           'FollowPath.primary_controller','FollowPath.closed_loop','FollowPath.rotate_to_goal_heading',
           'FollowPath.min_approach_linear_velocity','FollowPath.approach_velocity_scaling_dist',
           'progress_checker.plugin','progress_checker.required_movement_angle',
           'FollowPath.desired_linear_vel','FollowPath.rotate_to_heading_angular_vel',
           'controller_frequency','goal_checker.xy_goal_tolerance',
           'goal_checker.yaw_goal_tolerance','goal_checker.stateful']
    values=call('/learning_nav2/controller_server/get_parameters',GetParameters,
                GetParameters.Request(names=names)).values
    for name,v in zip(names,values):
        report['parameters'][name] = (v.double_value if v.type==3 else
                                      v.string_value if v.type==4 else v.bool_value)
    for name in ['map_server','planner_server','controller_server','bt_navigator']:
        v=call('/learning_nav2/'+name+'/get_state',GetState,GetState.Request()).current_state
        report['lifecycle'][name]=dict(id=v.id,label=v.label)
    until=time.monotonic()+1.5
    while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
    report['observations']=observed[-10:]
    report['nodes']=n.get_node_names()
    report['command_publishers']=n.count_publishers('/learning_nav2/cmd_vel_raw')
    report['nav2_odom_publishers']=n.count_publishers('/learning_nav2/odom')
    p=report['parameters']
    assert p['FollowPath.max_angular_accel']==.50
    assert p['FollowPath.plugin']=='nav2_rotation_shim_controller::RotationShimController'
    assert p['FollowPath.primary_controller']=='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    assert p['FollowPath.closed_loop'] is False and p['FollowPath.rotate_to_goal_heading'] is True
    assert p['FollowPath.min_approach_linear_velocity']==.05
    assert p['FollowPath.approach_velocity_scaling_dist']==1.
    assert p['progress_checker.plugin']=='nav2_controller::PoseProgressChecker'
    assert p['progress_checker.required_movement_angle']==.10
    assert p['goal_checker.stateful'] is False
    assert p['FollowPath.desired_linear_vel']==.15
    assert p['FollowPath.rotate_to_heading_angular_vel']==.20
    assert p['goal_checker.xy_goal_tolerance']==.15 and p['goal_checker.yaw_goal_tolerance']==.10
    assert all(v['id']==3 for v in report['lifecycle'].values())
    assert report['command_publishers']==report['nav2_odom_publishers']==1
    assert len(observed)>=5 and all(-.05<=v['age']<=.35 and
        v['status']['map_manifest_sha256']==MAP_HASH for v in observed[-5:])
    assert not set(report['nodes']) & {'navigation_manager','sztu_safety_gateway',
        'learning_nav2_motor','learning_goal_motor','learning_straight_motor'}
    report['passed']=True
finally:
    print(json.dumps(report,indent=2),flush=True)
    n.destroy_node();rclpy.try_shutdown()
