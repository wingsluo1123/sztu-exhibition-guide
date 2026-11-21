"""Probe the installed RPP with frozen TF and deliberately injected odom speed.

Controller stress test only: the frozen pose is not a simulated robot trajectory.
No AimDK imports, motor objects, or domain0 access. Baseline/candidate reports
show whether rotation follows target error despite opposite measured velocity.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from nav2_integration.smoke_test import wait_future


class Probe(Node):
    def __init__(self):
        super().__init__('learning_nav2_rotation_probe')
        self.pose = [.0079452679, -.0576307147, -1.8254150555]
        self.measured_w = 0.
        self.samples = []
        self.tf = self.create_publisher(TFMessage, '/learning_nav2/tf', 10)
        self.odom = self.create_publisher(Odometry, '/learning_nav2/odom', 10)
        self.create_subscription(Twist, '/learning_nav2/cmd_vel_raw', self.receive, 10)
        self.create_timer(.05, self.tick)
        self.client = ActionClient(self, NavigateToPose, '/learning_nav2/navigate_to_pose')

    def receive(self, message):
        self.samples.append((time.monotonic(), message.linear.x, message.angular.z))

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for parent, child, p in [('learning_map', 'learning_nav_odom', [0., 0., 0.]),
                                  ('learning_nav_odom', 'learning_nav_base', self.pose)]:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.x, t.transform.translation.y = p[:2]
            t.transform.rotation.z, t.transform.rotation.w = math.sin(p[2]/2), math.cos(p[2]/2)
            transforms.append(t)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = 'learning_nav_odom', 'learning_nav_base'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.pose[:2]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.pose[2]/2), math.cos(self.pose[2]/2)
        odom.twist.twist.angular.z = self.measured_w
        self.tf.publish(TFMessage(transforms=transforms))
        self.odom.publish(odom)

    def spin_for(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.01)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('isolated domain42 localhost required')
    args.output.mkdir(parents=True, exist_ok=False)
    source = (HERE/'parameters.py').read_bytes()
    (args.output/'parameters.py').write_bytes(source)
    report = dict(no_robot_api=True, frozen_pose_stress_test=True, cases=[],
                  parameters_sha256=hashlib.sha256(source).hexdigest())
    log = (args.output/'nav2.log').open('x')
    process = subprocess.Popen(['bash', str(HERE/'run_stack.bash')], stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(); node = Probe()
    try:
        deadline = time.monotonic()+45.
        while not node.client.server_is_ready():
            assert time.monotonic() < deadline
            rclpy.spin_once(node, timeout_sec=.02)
        for name in ['map_server', 'planner_server', 'controller_server', 'bt_navigator']:
            client = node.create_client(GetState, '/learning_nav2/'+name+'/get_state')
            while True:
                response = wait_future(node, client.call_async(GetState.Request()), 5.)
                if response.current_state.id == 3:
                    break
                assert time.monotonic() < deadline
                node.spin_for(.1)
            node.destroy_client(client)
        for label, xy, yaw, sign in [
                ('right_to_path', [-.6619954705, -.5016914010], -2.6864483031, -1),
                ('left_final_heading', node.pose[:2], node.pose[2]+.5, 1)]:
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = 'learning_map'
            request.pose.header.stamp = node.get_clock().now().to_msg()
            request.pose.pose.position.x, request.pose.pose.position.y = xy
            request.pose.pose.orientation.z, request.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            handle = wait_future(node, node.client.send_goal_async(request), 5.)
            assert handle.accepted
            for measured in [0., .65, -.65]:
                node.measured_w = measured
                node.spin_for(.6)  # allow controller odom smoothing window to settle
                since = time.monotonic()
                node.spin_for(.3)
                samples = [(v,w) for t,v,w in node.samples if t >= since]
                passed = (len(samples) >= 3 and all(abs(v) < 1e-8 and
                          .69 <= sign*w <= .70000001 for v,w in samples))
                report['cases'].append(dict(case=label, measured_angular=measured,
                                            samples=samples, passed=passed))
            wait_future(node, handle.cancel_goal_async(), 3.)
            node.spin_for(.3)
        report['passed'] = all(c['passed'] for c in report['cases'])
    except Exception as error:
        report['error'] = repr(error)
        raise
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5.)
        report['exit_code'] = process.returncode
        log.close(); node.destroy_node(); rclpy.try_shutdown()
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
