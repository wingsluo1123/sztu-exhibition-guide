"""One official Nav2 goal with optional supervised AimDK output.

Default is diagnostic: no motor object, registration, or velocity publication.
Nav2 owns planning/tracking/heading. This adapter validates and forwards only.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path as RosPath
from rclpy.action import ActionClient

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT/'goal_navigation'))
from navigate import Observer, service_observations, wait_ready, readiness_snapshot
from grid_route import GridRoute, sha
from motion_policy import wrap
from nav2_integration.command_gate import command_reason
from nav2_integration.goal_selection import validate_goal, validate_route
from planar_math import pose as planar_pose
from rviz_bridge import goal_values


class Nav2Observer(Observer):
    def __init__(self):
        super().__init__()
        self.raw_values = None; self.raw_received = None
        self.raw_subscription = self.create_subscription(Twist, '/learning_nav2/cmd_vel_raw', self.raw, 1)
        self.create_subscription(RosPath, '/learning_nav2/plan', self.path_pub.publish, 1)
        self.navigator = ActionClient(self, NavigateToPose, '/learning_nav2/navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, '/learning_nav2/compute_path_to_pose')

    def raw(self, message):
        self.raw_values = [message.linear.x, message.linear.y, message.linear.z,
                           message.angular.x, message.angular.y, message.angular.z]
        self.raw_received = time.monotonic()

    def command_source_count(self):
        return self.count_publishers('/learning_nav2/cmd_vel_raw')


def future_result(node, future, seconds, interrupted):
    end = time.monotonic()+seconds
    while not future.done():
        service_observations(node)
        if interrupted.is_set(): raise RuntimeError('operator_interrupt')
        if time.monotonic() > end: raise TimeoutError('nav2_action_response_timeout')
        time.sleep(.004)
    return future.result()


def wait_action_servers(node, seconds, interrupted):
    """Lifecycle activation predates discovery in a newly started client."""
    end = time.monotonic()+seconds
    while True:
        if interrupted.is_set():
            raise RuntimeError('operator_interrupt_before_action')
        service_observations(node)
        if node.navigator.server_is_ready() and node.planner.server_is_ready():
            return
        if time.monotonic() >= end:
            raise RuntimeError('nav2_servers_not_ready')
        time.sleep(.01)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--rviz-goal', action='store_true')
    mode.add_argument('--relative-goal', type=float, nargs=2)
    p.add_argument('--goal-yaw-deg', type=float)
    p.add_argument('--approved-relative-goal', type=float, nargs=2, default=None,
                   help='Optional restricted trial area; default accepts any reachable map goal')
    execution = p.add_mutually_exclusive_group()
    execution.add_argument('--execute-one-attempt', action='store_true')
    execution.add_argument('--plan-only', action='store_true',
                           help='Compute and display path only; never request navigation or register a motor')
    p.add_argument('--max-distance', type=float, default=2.,
                   help='Approved physical travel budget in metres; ignored for plan-only')
    p.add_argument('--observe-seconds', type=float, default=5.)
    p.add_argument('--goal-wait-seconds', type=float, default=180.)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--map-directory', type=Path, default=Path(os.environ.get(
        'LEARNING_LOCALIZATION_ROOT', str(ROOT.parent/'previous_learning')))/'maps/room_7x3_20261120_map01')
    args = p.parse_args()
    if not math.isfinite(args.max_distance) or args.max_distance <= 0:
        raise ValueError('max-distance must be finite and positive')
    for values in (args.relative_goal, args.approved_relative_goal):
        if values is not None and not all(math.isfinite(v) for v in values):
            raise ValueError('relative goal must be finite')
    if args.goal_yaw_deg is not None and not math.isfinite(args.goal_yaw_deg):
        raise ValueError('goal yaw must be finite')
    if not 0 <= args.observe_seconds <= 30 or not 1 <= args.goal_wait_seconds <= 180:
        raise ValueError('invalid diagnostic/goal-wait window')
    if os.environ.get('ROS_DOMAIN_ID') != '0': raise RuntimeError('robot domain0 required')
    if subprocess.run(['systemctl','--user','is-active','--quiet','sztu-bridge']).returncode == 0:
        raise RuntimeError('sztu-bridge active')
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(actuation_requested=args.execute_one_attempt, motor_registered=False,
                  nonzero_commands=0, nav2_nonzero_samples=0,
                  controller='official Nav2 RotationShim + RegulatedPurePursuit',
                  planner='official Nav2 Navfn A*', dynamic_obstacles=False,
                  total_task_deadline=None, max_distance_m=None if args.plan_only else args.max_distance,
                  parameters={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
                  source_sha256={str(f.relative_to(ROOT)):sha(f) for f in
                                 [Path(__file__),HERE/'command_gate.py',HERE/'goal_selection.py',HERE/'parameters.py',
                                  HERE/'input_adapter.py',HERE/'navigate_once.xml',
                                  ROOT/'navigation_observation.py',ROOT/'navigation_localization.py',
                                  ROOT/'straight_trial.py',ROOT/'goal_navigation/navigate.py']})
    rclpy.init(); node = Nav2Observer(); motor = None; handle = None
    done = threading.Event(); interrupted = threading.Event(); watcher = None
    lock = threading.Lock(); control = dict(heartbeat=time.monotonic(), abort=None)
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, lambda *_:interrupted.set())
    events = (args.output/'events.jsonl').open('x')
    try:
        if not wait_ready(node,15.,1.): raise RuntimeError('preflight_not_ready')
        names = node.get_node_names()
        if names.count('learning_straight_trial') > 1 or set(names) & {
            'learning_straight_motor','learning_goal_motor','learning_nav2_motor',
            'navigation_manager','sztu_safety_gateway'}:
            raise RuntimeError('competing_control_node')
        grid = GridRoute.load(args.map_directory,.40)
        if grid.manifest_hash != node.status['map_manifest_sha256']:
            raise RuntimeError('map_lineage_mismatch')
        wait_action_servers(node, 10., interrupted)
        goal = PoseStamped(); goal.header.frame_id = 'learning_map'
        if args.rviz_goal:
            end = time.monotonic()+args.goal_wait_seconds
            while node.goal_message is None and time.monotonic() < end and not interrupted.is_set():
                node.announce(str(args.output.resolve()), 'waiting_for_goal', grid.manifest_hash)
                service_observations(node); time.sleep(.02)
            node.announce(str(args.output.resolve()), 'goal_wait_finished', grid.manifest_hash)
            if node.goal_message is None: raise RuntimeError('no_goal_received')
            goal = node.goal_message
        else:
            from planar_math import matrix
            xy = (matrix(node.map_pose)@np.r_[args.relative_goal,1.])[:2]
            goal.pose.position.x, goal.pose.position.y = map(float,xy)
            yaw = (math.radians(args.goal_yaw_deg) if args.goal_yaw_deg is not None
                   else node.map_pose[2]+math.atan2(args.relative_goal[1],args.relative_goal[0]))
            goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        goal_values(goal)
        if not node.navigation_ready(): raise RuntimeError('localization_not_ready_at_goal')
        initial_odom = node.odom
        frozen_map_odom = node.map_matrix.copy()
        goal.header.stamp = node.get_clock().now().to_msg()
        target = [goal.pose.position.x, goal.pose.position.y]
        goal_yaw = 2*math.atan2(goal.pose.orientation.z,goal.pose.orientation.w)
        report.update(goal=target, goal_yaw_rad=goal_yaw)
        validate_goal(grid, target, node.map_pose, args.approved_relative_goal)
        request = ComputePathToPose.Goal(); request.goal = goal; request.planner_id = 'GridBased'
        plan_handle = future_result(node,node.planner.send_goal_async(request),5.,interrupted)
        if not plan_handle.accepted: raise RuntimeError('nav2_plan_rejected')
        planned = future_result(node,plan_handle.get_result_async(),8.,interrupted)
        report['nav2_planning_status'] = planned.status
        if planned.status != GoalStatus.STATUS_SUCCEEDED: raise RuntimeError('nav2_plan_failed')
        route = [[v.pose.position.x,v.pose.position.y] for v in planned.result.path.poses]
        distance = validate_route(grid, route, target,
                                  None if args.plan_only else args.max_distance,
                                  start=node.map_pose[:2].tolist())
        node.path_pub.publish(planned.result.path); node.goal_pub.publish(goal)
        report.update(goal=target,goal_yaw_rad=goal_yaw,nav2_path=route,planned_distance_m=distance)
        report['preflight']=readiness_snapshot(node)
        (args.output/'plan.json').write_text(json.dumps(report,indent=2))
        if args.plan_only:
            report['result']='planned_only_no_navigation_or_motor'
            # Keep the publishers alive long enough for the SSH relay to collect
            # the transient path and goal. No NavigateToPose is requested here.
            end = time.monotonic()+.5
            while time.monotonic()<end and not interrupted.is_set():
                node.announce(str(args.output.resolve()), 'planned_only', grid.manifest_hash)
                service_observations(node); time.sleep(.01)
            return
        if args.execute_one_attempt:
            if not wait_ready(node,3.,.5): raise RuntimeError('registration_preflight_not_ready')
            if node.command_source_count() != 1: raise RuntimeError('nav2_command_publisher_count')
            from sztu_navigation.locomotion import LocomotionController
            motor = LocomotionController(source='sdk_learning_nav2',priority=45,command_timeout=.20,
                release_after=1.,forward_threshold=0.,lateral_threshold=0.,angular_threshold=0.,
                node_name='learning_nav2_motor')
            if not motor.register_input_source() or not motor.registration_confirmed:
                raise RuntimeError('motor_registration_unconfirmed')
            report['motor_registered']=True; report['registration_responses']=motor.registration_responses
            if not wait_ready(node,3.,.5): raise RuntimeError('post_registration_not_ready')
            if node.guard.arm(node.get_clock().now().nanoseconds,time.monotonic()):
                raise RuntimeError('mc_not_armable')
        if interrupted.is_set(): raise RuntimeError('operator_interrupt_before_action')
        if math.dist(initial_odom[:2],node.odom[:2])>.10 or abs(wrap(initial_odom[2]-node.odom[2]))>.15:
            raise RuntimeError('robot_moved_during_planning')
        node.raw_values = None; node.raw_received = None
        request = NavigateToPose.Goal(); request.pose = goal
        handle = future_result(node,node.navigator.send_goal_async(request),5.,interrupted)
        if not handle.accepted: raise RuntimeError('nav2_goal_rejected')
        result_future = handle.get_result_async()
        start = time.monotonic(); travel = 0.; previous = node.odom; fault = None
        report['started_epoch']=time.time()
        if motor:
            control['heartbeat']=time.monotonic()
            def watch():
                while not done.wait(.02):
                    with lock:
                        if time.monotonic()-control['heartbeat'] > .20:
                            control['abort']='control_heartbeat_timeout'
                        if control['abort']: motor.stop(); motor.tick()
            watcher=threading.Thread(target=watch,daemon=True);watcher.start()
        while rclpy.ok():
            cycle=time.monotonic(); service_observations(node); now=time.monotonic()
            if interrupted.is_set(): fault='operator_interrupt'
            if not node.navigation_ready(): fault='localization_or_mc_invalid'
            step=math.dist(previous[:2],node.odom[:2]); travel+=step; previous=node.odom
            if step>.15: fault='odometry_jump'
            if travel>args.max_distance: fault='travel_limit'
            correction=planar_pose(np.linalg.inv(frozen_map_odom)@node.map_matrix)
            if math.hypot(*correction[:2])>.30 or abs(correction[2])>.35:
                fault='localization_correction_changed'
            try:
                if not grid.allowed[grid.cell(node.map_pose[:2])]: fault='map_clearance_lost'
            except ValueError: fault='outside_map'
            values=node.raw_values
            if values is None:
                values=[0.]*6
                if now-start>3.: fault='nav2_no_command'
            else:
                reason=command_reason(values,node.raw_received,now)
                if reason: fault=reason
            if node.command_source_count()!=1: fault='nav2_command_publisher_count'
            if control['abort']: fault=control['abort']
            if fault:
                if motor: motor.stop()
                report['result']=fault
                events.write(json.dumps(dict(epoch=time.time(),fault=fault,
                    readiness=readiness_snapshot(node)))+'\n');events.flush()
                break
            if result_future.done():
                result=result_future.result(); report['nav2_action_status']=result.status
                report['result']='nav2_succeeded_pending_settle' if result.status==GoalStatus.STATUS_SUCCEEDED else 'nav2_action_failed'
                break
            if not motor and now-start>=args.observe_seconds:
                report['result']='diagnostic_nav2_commands_observed_no_motor';break
            with lock:
                if control['abort']: fault=control['abort']
                control['heartbeat']=now
                if motor:
                    if fault: motor.stop()
                    else: motor.set_velocity(values[0],0.,values[5])
                    motor.tick(); report['nonzero_commands']+=int(not fault and bool(values[0] or values[5]))
            report['nav2_nonzero_samples']+=int(bool(values[0] or values[5]))
            events.write(json.dumps(dict(epoch=time.time(),odom=node.odom,map_pose=node.map_pose.tolist(),
                raw_command=values,forwarded=bool(motor and not fault),travel_m=travel,fault=fault,
                readiness=readiness_snapshot(node)))+'\n');events.flush()
            if fault: report['result']=fault;break
            time.sleep(max(0.,.02-(time.monotonic()-cycle)))
        if motor: motor.stop()
        done.set()
        if watcher: watcher.join(timeout=1.); watcher=None
        if handle and not result_future.done(): future_result(node,handle.cancel_goal_async(),3.,threading.Event())
        until=time.monotonic()+2.5
        while time.monotonic()<until:
            service_observations(node)
            if motor: motor.tick()
            time.sleep(.014)
        report.update(final_map_pose=node.map_pose.tolist(),goal_distance_m=math.dist(node.map_pose[:2],target),
            goal_yaw_error_deg=math.degrees(wrap(goal_yaw-node.map_pose[2])),travel_m=travel,
            final_ready=node.navigation_ready())
        if report.get('result')=='nav2_succeeded_pending_settle':
            if not node.navigation_ready(): report['result']='final_localization_invalid'
            elif report['goal_distance_m']>.15: report['result']='final_position_outside_tolerance'
            elif abs(report['goal_yaw_error_deg'])>math.degrees(.10): report['result']='final_heading_outside_tolerance'
            else: report['result']='arrived'
        report['operator_stop_confirmation']='pending' if motor else 'no_actuation'
    except Exception as error:
        report['error']=repr(error)
        report.setdefault('result', 'rejected_before_navigation' if handle is None else 'navigation_error')
    finally:
        done.set()
        if watcher: watcher.join(timeout=1.)
        if motor: motor.stop(); motor.destroy_node()
        if handle:
            try: future_result(node,handle.cancel_goal_async(),2.,threading.Event())
            except Exception: pass
        (args.output/'wire.json').write_text(json.dumps(node.wire))
        report['ended_epoch']=time.time(); report['last_localization']=node.status
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        events.close(); node.destroy_node(); rclpy.try_shutdown()


if __name__=='__main__': main()
