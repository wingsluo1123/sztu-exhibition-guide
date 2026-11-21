"""Bounded supervised commissioning: clear site required, no autonomous obstacle avoidance.

Default is diagnostic only. --execute-one-attempt consumes one operator-authorized
attempt. Never automatically retries, turns, reverses or resumes after a fault.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sztu_navigation.mc_state_guard import McStateGuard
from straight_trial_core import StraightTrial


class TrialObserver(Node):
    def __init__(self, legacy_navigation=True):
        from aimdk_msgs.msg import McCommonState, McLocomotionVelocity
        super().__init__('learning_straight_trial')
        self.guard = McStateGuard(.5)
        self.mc_source = None
        self.wire = []
        self.create_subscription(McLocomotionVelocity,
            "/aima/mc/locomotion/velocity", self.receive_wire, qos_profile_sensor_data)
        self.odom = None
        self.status = None
        self.odom_received = self.status_received = None
        self.last_stamp = None
        self.invalid_odometry = False
        if legacy_navigation:
            self.create_subscription(Odometry, '/learning/nav/odometry', self.receive_odom, 10)
            self.create_subscription(String, '/learning/nav/localization_status', self.receive_status, 10)
        self.create_subscription(McCommonState, '/aima/mc/common/state', self.receive_mc, qos_profile_sensor_data)

    def receive_wire(self, message):
        self.wire.append({'epoch':time.time(),'source':message.source,
                          'forward':message.forward_velocity,'lateral':message.lateral_velocity,
                          'angular':message.angular_velocity})

    def receive_mc(self, message):
        self.mc_source = message.input_source.name
        self.guard.receive(message, self.get_clock().now().nanoseconds, time.monotonic())

    def receive_status(self, message):
        try:
            self.status = json.loads(message.data)
        except (ValueError, TypeError):
            self.status = None
        self.status_received = time.monotonic()

    def receive_odom(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        p, q = message.pose.pose.position, message.pose.pose.orientation
        good = (message.header.frame_id == 'learning_nav_odom'
                and message.child_frame_id == 'learning_nav_base'
                and all(math.isfinite(v) for v in [stamp,p.x,p.y,q.x,q.y,q.z,q.w])
                and abs(q.x)<1e-6 and abs(q.y)<1e-6
                and abs(q.z*q.z+q.w*q.w-1)<1e-3
                and (self.last_stamp is None or stamp>self.last_stamp))
        if not good:
            self.invalid_odometry = True
            return
        self.last_stamp = stamp
        self.odom = (p.x,p.y,2*math.atan2(q.z,q.w))
        self.odom_received = time.monotonic()

    def ready(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds*1e-9
        if self.guard.check(self.get_clock().now().nanoseconds,now):
            return False
        if self.invalid_odometry or self.odom is None or self.status is None:
            return False
        stamp = self.status.get('stamp')
        return bool(self.status.get('valid') and self.status.get('map_manifest_sha256') ==
                    'f2b1f398b010e58de0b276646d59d2905c48dac805f90226d67e69429fcf51c0'
                    and isinstance(stamp,(float,int)) and math.isfinite(stamp)
                    and 0<=now-self.odom_received<=.25 and 0<=now-self.status_received<=.25
                    and -.05<=ros_now-self.last_stamp<=.25 and -.05<=ros_now-stamp<=.25)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-one-attempt',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--stop-lead',type=float,default=0.)
    parser.add_argument('--speed',type=float,choices=[.10,.15],default=.10)
    parser.add_argument('--duration',type=float,default=15.)
    parser.add_argument('--observe-deviation',action='store_true')
    args=parser.parse_args()
    policy=StraightTrial(observe_deviation=args.observe_deviation,speed=args.speed,duration=args.duration,stop_lead=args.stop_lead)
    if os.environ.get('ROS_DOMAIN_ID')!='0':
        raise RuntimeError('Robot domain0 required')
    if subprocess.run(['systemctl','--user','is-active','--quiet','sztu-bridge']).returncode==0:
        raise RuntimeError('sztu-bridge is active')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init(); node=TrialObserver(); motor=None
    report={'stop_lead_m':args.stop_lead,'deviation_policy':'observe_only' if args.observe_deviation else 'abort', 'actuation_requested':args.execute_one_attempt,'nonzero_command_count':0,
            'max_command_mps':0.,'limit_s':args.duration,'goal_m':.5,'tolerance_m':.1,
            'obstacle_avoidance':False,'site_clearance':'operator-supervised clear-site commissioning only'}
    events=(args.output/'events.jsonl').open('x')
    lock=threading.Lock(); watchdog_done=threading.Event()
    control={'heartbeat':time.monotonic(),'deadline':None,'abort':None}
    thread=None
    try:
        # Require two continuous seconds ready; fail rather than retry indefinitely.
        deadline=time.monotonic()+12; stable=None
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if node.ready():
                stable=stable or time.monotonic()
                if time.monotonic()-stable>=2:break
            else:stable=None
        if stable is None or time.monotonic()-stable<2:
            raise RuntimeError('preflight_not_continuously_ready')
        names=node.get_node_names()
        if any(name in names for name in ('learning_straight_motor','sztu_safety_gateway','navigation_manager')):
            raise RuntimeError('competing_control_node_present')
        if names.count('learning_straight_trial')>1:
            raise RuntimeError('duplicate_trial_node_present')
        report['preflight_ready']=True
        if not args.execute_one_attempt:
            report['result']='diagnostic_only_no_input_source_registered'
            return
        # Registration itself never publishes a locomotion command.
        from sztu_navigation.locomotion import LocomotionController
        motor=LocomotionController(source='sdk_learning_straight',priority=45,
            command_timeout=.20,release_after=1.,forward_threshold=0.,
            lateral_threshold=0.,angular_threshold=0.,node_name='learning_straight_motor')
        if not motor.register_input_source() or not motor.registration_confirmed:
            raise RuntimeError('input_source_not_confirmed')
        report['registration_responses']=motor.registration_responses
        # Service registration spins the motor node, not the observer. Drain queued
        # observations and establish fresh continuous readiness before arming.
        refresh_deadline=time.monotonic()+3.; refreshed=None
        while time.monotonic()<refresh_deadline:
            rclpy.spin_once(node,timeout_sec=.005)
            if node.ready():
                refreshed=refreshed or time.monotonic()
                if time.monotonic()-refreshed>=.5:break
            else:refreshed=None
        if refreshed is None or time.monotonic()-refreshed<.5:
            raise RuntimeError('post_registration_not_continuously_ready')
        reason=node.guard.arm(node.get_clock().now().nanoseconds,time.monotonic())
        if reason or not node.ready():raise RuntimeError(reason or 'post_registration_not_ready')
        report['attempt_started_epoch']=time.time()
        control['heartbeat']=time.monotonic();control['deadline']=time.monotonic()+args.duration

        def watchdog():
            while not watchdog_done.wait(.02):
                with lock:
                    now=time.monotonic()
                    if control['abort'] is None and (now-control['heartbeat']>.20 or now>=control['deadline']):
                        control['abort']='independent_watchdog_timeout'
                        motor.stop()
                    if control['abort'] is not None:motor.tick()
        thread=threading.Thread(target=watchdog,daemon=True);thread.start()
        trial=policy
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.01)
            with lock:
                now=time.monotonic()
                if control['abort']:trial.stop(control['abort'])
                speed=trial.step(now,node.odom,node.ready())
                control['heartbeat']=now
                motor.set_velocity(speed,0.,0.);motor.tick()
                report['nonzero_command_count']+=int(speed>0)
                report['max_command_mps']=max(report['max_command_mps'],speed)
            events.write(json.dumps({'epoch':time.time(),'command':speed,'progress':trial.progress,
                'deviations':trial.deviations,'pose':node.odom,'reason':trial.reason,'ready':node.ready(),'mc_source':node.mc_source,'mc_observation':node.guard.last_observation})+'\n');events.flush()
            if trial.reason:
                report['result']=trial.reason;report['progress_at_stop_command_m']=trial.progress;break
            time.sleep(.01)
        # Keep observing after issuing zero. This is sensor evidence, not a claim of physical stop.
        motor.stop(); stop_deadline=time.monotonic()+2
        while time.monotonic()<stop_deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            with lock:motor.tick()
        report['final_pose']=node.odom
        report['physical_stop_confirmed']=False
    except Exception as error:
        report['error']=repr(error)
    finally:
        watchdog_done.set()
        if thread:thread.join(timeout=1.)
        if motor:
            motor.stop();motor.destroy_node()
        (args.output/'wire.json').write_text(json.dumps(node.wire))
        report['last_localization_status']=node.status
        report['last_odometry_stamp']=node.last_stamp
        report['invalid_odometry_latched']=node.invalid_odometry
        report['mc_guard_reason']=node.guard.reason
        report['mc_last_observation']=node.guard.last_observation
        report['mc_last_source']=node.mc_source
        report['ended_epoch']=time.time()
        (args.output/'report.json').write_text(json.dumps(report,indent=2))
        events.close();node.destroy_node();rclpy.try_shutdown()
        print(json.dumps(report,indent=2))


if __name__=='__main__':main()
