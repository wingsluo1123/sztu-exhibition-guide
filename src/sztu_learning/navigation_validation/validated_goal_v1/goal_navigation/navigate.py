"""One supervised turn or static-map goal, diagnostic by default.

Uses existing FAST-LIO/localization and original AimDK controller. Does not
provide dynamic-obstacle avoidance. A mission is never replayed or retried.
"""
import argparse
from collections import deque
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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.qos import QoSProfile,DurabilityPolicy
from tf2_msgs.msg import TFMessage

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from straight_trial import TrialObserver
from planar_math import matrix,pose as planar_pose
from grid_route import GridRoute,sha
from motion_policy import MotionPolicy,wrap
from input_readiness import inputs_ready,MAX_SOURCE_AGE


class Observer(TrialObserver):
    def __init__(self):
        super().__init__()
        self.map_matrix=None;self.map_stamp=None;self.map_received=None
        self.goal_message=None
        self.create_subscription(TFMessage,'/learning/nav/tf',self.receive_tf,10)
        self.create_subscription(PoseStamped,'/learning/nav/goal',self.receive_goal,10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub=self.create_publisher(RosPath,'/learning/nav/planned_path',qos)
        self.goal_pub=self.create_publisher(PoseStamped,'/learning/nav/accepted_goal',qos)

    def receive_tf(self,message):
        transforms=message.transforms
        expected=[('learning_map','learning_nav_odom'),('learning_nav_odom','learning_nav_base')]
        if len(transforms)!=2:return
        if [(t.header.frame_id,t.child_frame_id) for t in transforms]!=expected:return
        stamps=[t.header.stamp.sec+t.header.stamp.nanosec*1e-9 for t in transforms]
        if stamps[0]!=stamps[1] or self.map_stamp is not None and stamps[0]<=self.map_stamp:return
        result=[]
        for t in transforms:
            p,q=t.transform.translation,t.transform.rotation
            if not all(math.isfinite(v) for v in [p.x,p.y,q.x,q.y,q.z,q.w]):return
            if abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.z*q.z+q.w*q.w-1)>.001:return
            result.append(matrix([p.x,p.y,2*math.atan2(q.z,q.w)]))
        self.map_matrix=result[0]
        self.map_pose=planar_pose(result[0]@result[1])
        self.map_stamp=stamps[0];self.map_received=time.monotonic()

    def receive_goal(self,message):
        p,q=message.pose.position,message.pose.orientation
        if (message.header.frame_id!='learning_map' or self.goal_message is not None
                or not all(math.isfinite(v) for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                or abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.z*q.z+q.w*q.w-1)>.001):return
        self.goal_message=message

    def navigation_ready(self):
        if self.map_matrix is None or self.odom is None:return False
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds*1e-9
        if self.guard.check(self.get_clock().now().nanoseconds,now):return False
        return inputs_ready(self.status,self.last_stamp,self.map_stamp,self.odom_received,
                            self.status_received,self.map_received,ros_now,now,self.invalid_odometry)

    def show_plan(self,route,goal_yaw):
        path=RosPath();path.header.frame_id='learning_map';path.header.stamp=self.get_clock().now().to_msg()
        for x,y in route:
            p=PoseStamped(header=path.header);p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.
            path.poses.append(p)
        self.path_pub.publish(path)
        target=PoseStamped(header=path.header);target.pose.position.x=float(route[-1][0]);target.pose.position.y=float(route[-1][1])
        target.pose.orientation.z=math.sin((goal_yaw or 0)/2);target.pose.orientation.w=math.cos((goal_yaw or 0)/2)
        self.goal_pub.publish(target)


def service_observations(node,budget=.006):
    """Drain a bounded batch; one callback per control tick starves status/TF.

    The velocity readback alone can exceed the control rate. Keep the age
    limits unchanged and process waiting observations before deciding readiness.
    """
    deadline=time.monotonic()+budget
    for _ in range(256):
        rclpy.spin_once(node,timeout_sec=0.)
        if time.monotonic()>=deadline:break


def readiness_snapshot(node):
    now=time.monotonic();epoch=node.get_clock().now().nanoseconds*1e-9
    return {'ready':node.navigation_ready(),'invalid_odometry':node.invalid_odometry,
            'mc_reason':node.guard.reason,'status':node.status,
            'odom_age':None if node.last_stamp is None else epoch-node.last_stamp,
            'map_age':None if node.map_stamp is None else epoch-node.map_stamp,
            'status_receive_age':None if node.status_received is None else now-node.status_received}


def wait_ready(node,seconds,stable_for):
    deadline=time.monotonic()+seconds;stable=None
    while time.monotonic()<deadline:
        service_observations(node)
        if node.navigation_ready():
            if stable is None:stable=time.monotonic()
            if time.monotonic()-stable>=stable_for:return True
        else:stable=None
        time.sleep(.004)
    return False


def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--turn-deg',type=float)
    mode.add_argument('--goal',type=float,nargs=2,metavar=('MAP_X','MAP_Y'))
    mode.add_argument('--relative-goal',type=float,nargs=2,metavar=('FORWARD','LEFT'))
    mode.add_argument('--rviz-goal',action='store_true')
    p.add_argument('--goal-yaw-deg',type=float)
    p.add_argument('--map-directory',type=Path,default=Path(os.environ.get('LEARNING_LOCALIZATION_ROOT',str(ROOT.parent/'previous_learning')))/'maps/room_7x3_20261120_map01')
    p.add_argument('--radius',type=float,default=.40)
    p.add_argument('--duration',type=float,default=30.)
    p.add_argument('--max-distance',type=float,default=2.)
    p.add_argument('--angular',type=float,default=.70)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--execute-one-attempt',action='store_true')
    p.add_argument('--observe-seconds',type=float,default=0.,help='diagnostic-only readiness observation')
    args=p.parse_args()
    if not 0<=args.observe_seconds<=60 or args.execute_one_attempt and args.observe_seconds:raise ValueError('observation is diagnostic only, up to 60 seconds')
    if args.turn_deg is not None and (not math.isfinite(args.turn_deg) or not 5<=abs(args.turn_deg)<=180):raise ValueError('turn outside 5..180 degrees')
    if not math.isfinite(args.radius) or args.radius<.4:raise ValueError('radius below provisional circumscribed envelope')
    if os.environ.get('ROS_DOMAIN_ID')!='0':raise RuntimeError('robot domain0 required')
    if subprocess.run(['systemctl','--user','is-active','--quiet','sztu-bridge']).returncode==0:raise RuntimeError('sztu-bridge active')
    args.output.mkdir(parents=True,exist_ok=False)
    report={'actuation_requested':args.execute_one_attempt,'nonzero_commands':0,'scope':'supervised static-map commissioning; no dynamic obstacle avoidance',
            'parameters':{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
            'source_sha256':{str(f.relative_to(ROOT)):sha(f) for f in [Path(__file__),Path(__file__).with_name('grid_route.py'),Path(__file__).with_name('motion_policy.py'),Path(__file__).with_name('input_readiness.py'),ROOT/'straight_trial.py']}}
    rclpy.init();node=Observer();motor=None;policy=None
    events=(args.output/'events.jsonl').open('x')
    lock=threading.Lock();done=threading.Event();watcher=None
    control={'heartbeat':time.monotonic(),'deadline':None,'abort':None}
    interrupted=threading.Event()
    signal.signal(signal.SIGTERM,lambda *_:interrupted.set())
    signal.signal(signal.SIGINT,lambda *_:interrupted.set())
    try:
        if not wait_ready(node,15,1.):raise RuntimeError('preflight_not_ready')
        names=node.get_node_names()
        if names.count('learning_straight_trial')>1 or set(names)&{'learning_straight_motor','learning_goal_motor','navigation_manager','sztu_safety_gateway'}:
            raise RuntimeError('competing_control_node')
        initial_map=node.map_pose.copy();initial_odom=node.odom
        frozen_map_odom=node.map_matrix.copy()
        turn_only=args.turn_deg is not None
        goal_yaw=math.radians(args.goal_yaw_deg) if args.goal_yaw_deg is not None else None
        if turn_only:
            route=[initial_map[:2].tolist()];goal_yaw=wrap(initial_map[2]+math.radians(args.turn_deg))
            grid=None
        else:
            grid=GridRoute.load(args.map_directory,args.radius)
            if grid.manifest_hash!=node.status['map_manifest_sha256']:raise RuntimeError('map/localization lineage mismatch')
            if args.rviz_goal:
                deadline=time.monotonic()+60
                while node.goal_message is None and time.monotonic()<deadline and not interrupted.is_set():
                    rclpy.spin_once(node,timeout_sec=.05)
                if node.goal_message is None:raise RuntimeError('no_goal_received')
                if not node.navigation_ready():raise RuntimeError('localization_not_ready_at_goal')
                initial_map=node.map_pose.copy();initial_odom=node.odom;frozen_map_odom=node.map_matrix.copy()
                m=node.goal_message;goal=[m.pose.position.x,m.pose.position.y]
                goal_yaw=2*math.atan2(m.pose.orientation.z,m.pose.orientation.w)
            elif args.relative_goal:
                delta=matrix(initial_map)@np.r_[args.relative_goal,1.]
                goal=delta[:2].tolist()
            else:goal=args.goal
            route=grid.plan(initial_map[:2],goal)
            route_length=sum(math.dist(a,b) for a,b in zip(route,route[1:]))
            if route_length>args.max_distance:raise RuntimeError('route exceeds approved travel budget')
        inv=np.linalg.inv(frozen_map_odom)
        odom_route=[(inv@np.r_[xy,1.])[:2].tolist() for xy in route]
        odom_yaw=wrap(goal_yaw-planar_pose(frozen_map_odom)[2]) if goal_yaw is not None else None
        policy=MotionPolicy(odom_route,goal_yaw=odom_yaw,turn_only=turn_only,duration=args.duration,
                            angular=args.angular,max_distance=args.max_distance,
                            yaw_tolerance=.20 if turn_only else .10,coast=.21 if turn_only else .26)
        plan={'map_route':route,'odom_route':odom_route,'map_goal_yaw':goal_yaw,'initial_map_pose':initial_map.tolist(),
              'map_from_odom_frozen':frozen_map_odom.tolist(),'radius_m':args.radius,'turn_only':turn_only,
              'map_manifest_sha256':node.status['map_manifest_sha256']}
        plan['maximum_source_age_s']=MAX_SOURCE_AGE
        plan['yaw_tolerance_rad']=policy.yaw_tolerance
        plan['forward_coast_compensation_m']=policy.coast
        (args.output/'plan.json').write_text(json.dumps(plan,indent=2));node.show_plan(route,goal_yaw)
        report['plan']=plan
        if interrupted.is_set():raise RuntimeError('operator_interrupt_before_registration')
        if not args.execute_one_attempt:
            deadline=time.monotonic()+args.observe_seconds
            report['observed_not_ready_samples']=0;report['observed_samples']=0
            while time.monotonic()<deadline and not interrupted.is_set():
                service_observations(node)
                sample=readiness_snapshot(node)
                report['observed_samples']+=1
                report['observed_not_ready_samples']+=int(not sample['ready'])
                events.write(json.dumps({'epoch':time.time(),'readiness':sample})+'\n');events.flush()
                time.sleep(.014)
            report['result']='diagnostic_plan_only_no_motor_registration';return
        from sztu_navigation.locomotion import LocomotionController
        motor=LocomotionController(source='sdk_learning_goal',priority=45,command_timeout=.20,release_after=1.,
                                  forward_threshold=0.,lateral_threshold=0.,angular_threshold=0.,node_name='learning_goal_motor')
        if not motor.register_input_source() or not motor.registration_confirmed:raise RuntimeError('registration_unconfirmed')
        report['registration_responses']=motor.registration_responses
        if not wait_ready(node,3,.5):raise RuntimeError('post_registration_not_ready')
        if interrupted.is_set():raise RuntimeError('operator_interrupt_before_motion')
        if node.guard.arm(node.get_clock().now().nanoseconds,time.monotonic()):raise RuntimeError('mc_not_armable')
        if math.dist(initial_odom[:2],node.odom[:2])>.10 or abs(wrap(initial_odom[2]-node.odom[2]))>.15:
            raise RuntimeError('robot_moved_during_planning')
        report['started_epoch']=time.time();control['deadline']=time.monotonic()+args.duration
        control['heartbeat']=time.monotonic()
        def watch():
            while not done.wait(.02):
                with lock:
                    now=time.monotonic()
                    if control['abort'] is None and (now-control['heartbeat']>.20 or now>=control['deadline']):
                        control['abort']='duration_limit' if now>=control['deadline'] else 'watchdog_gap'
                        motor.stop()
                    if control['abort']:motor.tick()
        watcher=threading.Thread(target=watch,daemon=True);watcher.start()
        while rclpy.ok():
            cycle=time.monotonic()
            service_observations(node)
            now=time.monotonic()
            with lock:
                if interrupted.is_set():policy.stop('operator_interrupt')
                if control['abort']:policy.stop(control['abort'])
                if node.map_matrix is not None:
                    correction=planar_pose(np.linalg.inv(frozen_map_odom)@node.map_matrix)
                    if math.hypot(*correction[:2])>.30 or abs(correction[2])>.35:policy.stop('localization_correction_changed')
                if grid is not None and node.navigation_ready():
                    # Static swept circle, including all stop/turn maneuvers. Unknown blocks.
                    current=node.map_pose[:2]
                    try:
                        if not grid.allowed[grid.cell(current)]:policy.stop('map_clearance_lost')
                        elif not grid.segment_clear(current,route[policy.index]):policy.stop('route_segment_blocked')
                    except ValueError:policy.stop('outside_map')
                readiness=readiness_snapshot(node)
                command=policy.step(now,node.odom,readiness['ready'])
                control['heartbeat']=now
                motor.set_velocity(command.forward,0.,command.angular);motor.tick()
                report['nonzero_commands']+=int(bool(command.forward or command.angular))
            events.write(json.dumps({'epoch':time.time(),'odom':node.odom,'map_pose':node.map_pose.tolist(),
                                     'forward':command.forward,'angular':command.angular,'phase':command.phase,
                                     'index':policy.index,'reason':policy.reason,'mc_source':node.mc_source,
                                     'readiness':readiness})+'\n');events.flush()
            if policy.reason:break
            time.sleep(max(0.,.02-(time.monotonic()-cycle)))
        motor.stop()
        # End deadline watchdog after motion loop; keep zeros while collecting stop evidence.
        done.set();watcher.join(timeout=1.);watcher=None
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            service_observations(node);motor.tick();time.sleep(.014)
        report['result']=policy.reason;report['final_map_pose']=node.map_pose.tolist();report['final_odom']=node.odom
        report['goal_distance_m']=math.dist(node.map_pose[:2],route[-1])
        report['goal_yaw_error_deg']=math.degrees(wrap(goal_yaw-node.map_pose[2])) if goal_yaw is not None else None
        if report['result']=='arrived':
            if not node.navigation_ready():report['result']='final_localization_not_ready'
            elif not turn_only and report['goal_distance_m']>policy.tolerance:report['result']='final_map_position_outside_tolerance'
            elif goal_yaw is not None and abs(wrap(goal_yaw-node.map_pose[2]))>policy.yaw_tolerance:report['result']='final_map_heading_outside_tolerance'
        report['arrival_checked_after_settling']=report['result']=='arrived'
        report['operator_stop_confirmation']='pending'
    except Exception as error:
        report['error']=repr(error)
    finally:
        done.set()
        if watcher:watcher.join(timeout=1.)
        if motor:motor.stop();motor.destroy_node()
        (args.output/'wire.json').write_text(json.dumps(node.wire))
        report['ended_epoch']=time.time();report['last_localization']=node.status
        (args.output/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
        events.close();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
