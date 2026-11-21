"""Real Nav2 processes, frozen site map, toy lagged plant, no robot APIs.

Requires domain42/localhost; results are software integration evidence only.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from navigation_observation import observation_packet
from goal_navigation.input_readiness import MAP_HASH
from nav2_integration.command_gate import command_reason


class Plant(Node):
    def __init__(self, events, angular_response_seconds=.12):
        super().__init__('learning_nav2_software_plant')
        self.pose = [.0079452679, -.0576307147, -1.8254150555]
        self.v = self.w = 0.; self.command = (0., 0.)
        self.command_received = 0.; self.last = time.monotonic()
        self.events = events; self.nonzero = 0
        self.angular_response_seconds = angular_response_seconds
        self.command_errors = []
        self.pub = self.create_publisher(String, '/learning/nav/observation', 1)
        self.create_subscription(Twist, '/learning_nav2/cmd_vel_raw', self.receive, 1)
        self.timer = self.create_timer(.05, self.tick)
        self.client = ActionClient(self, NavigateToPose, '/learning_nav2/navigate_to_pose')

    def receive(self, message):
        values = [message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z]
        reason = command_reason(values, 1., 1.)
        if reason: self.command_errors.append(reason)
        self.command = (message.linear.x, message.angular.z)
        self.command_received = time.monotonic()
        self.nonzero += int(any(self.command))

    def tick(self):
        now = time.monotonic(); dt = min(.15, now-self.last); self.last = now
        v, w = self.command if now-self.command_received <= .20 else (0., 0.)
        self.v += (2.*v-self.v)*min(1., dt/.65)
        self.w += (w-self.w)*min(1., dt/self.angular_response_seconds)
        self.pose[0] += self.v*math.cos(self.pose[2])*dt
        self.pose[1] += self.v*math.sin(self.pose[2])*dt
        self.pose[2] = math.remainder(self.pose[2]+self.w*dt, 2*math.pi)
        epoch = self.get_clock().now().nanoseconds*1e-9
        status = dict(valid=True, source_valid=True, stamp=epoch,
                      map_manifest_sha256=MAP_HASH)
        self.pub.publish(String(data=json.dumps(observation_packet(status, self.pose, [0.,0.,0.]))))
        self.events.write(json.dumps(dict(epoch=epoch, pose=self.pose, command=self.command))+'\n')


def wait_future(node, future, seconds):
    deadline = time.monotonic()+seconds
    while not future.done():
        if time.monotonic() > deadline: raise TimeoutError('integration-test deadline')
        rclpy.spin_once(node, timeout_sec=.01)
    return future.result()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--angular-response-seconds', type=float, default=.12,
                   help='Toy first-order response constant, not a hardware calibration')
    p.add_argument('--case', choices=['short', 'recorded-long'], default='short',
                   help='recorded-long uses the last physical goal, with synthetic feedback')
    args = p.parse_args()
    if not math.isfinite(args.angular_response_seconds) or not .05 <= args.angular_response_seconds <= 2.:
        raise ValueError('angular response constant must be finite and in [.05, 2] seconds')
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('software plant requires isolated domain42 localhost')
    args.output.mkdir(parents=True, exist_ok=False)
    handles = []; processes = []; report = dict(no_robot_api=True, results=[],
        model=dict(linear_gain=2., linear_response_seconds=.65,
                   angular_response_seconds=args.angular_response_seconds,
                   synthetic_pose_feedback=True, source_localization_tested=False))
    rclpy.init(); events = (args.output/'plant.jsonl').open('x')
    node = Plant(events, args.angular_response_seconds)
    if args.case == 'recorded-long':
        node.pose = [-.0524241084, -.1369438325, -1.722]
        goal_xy = [2.1802327632904053, -1.423635721206665]
        goal_yaws = [-.808829584721443, .4]
    else:
        goal_xy = [-.6619954705, -.5016914010]
        goal_yaws = [-2.6864483031, -1.2]
    report['case'] = args.case
    report['initial_pose'] = node.pose.copy()
    (args.output/'parameters.py').write_bytes((HERE/'parameters.py').read_bytes())
    try:
        for name, command in [('adapter', [sys.executable, str(HERE/'input_adapter.py')]),
                              ('nav2', ['bash', str(HERE/'run_stack.bash')])]:
            handle = (args.output/(name+'.log')).open('x'); handles.append(handle)
            processes.append(subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        deadline = time.monotonic()+45.
        while not node.client.server_is_ready():
            assert time.monotonic() < deadline, 'Nav2 action server not ready'
            rclpy.spin_once(node, timeout_sec=.02)
        for name in ['map_server','planner_server','controller_server','bt_navigator']:
            client = node.create_client(GetState, '/learning_nav2/'+name+'/get_state')
            while True:
                response = wait_future(node, client.call_async(GetState.Request()), 5.)
                if response.current_state.id == 3: break
                assert time.monotonic() < deadline, name+' not active'
                rclpy.spin_once(node, timeout_sec=.1)
            node.destroy_client(client)
        report['all_lifecycle_nodes_active'] = True
        client = node.create_client(GetParameters, '/learning_nav2/controller_server/get_parameters')
        names = ['FollowPath.plugin', 'FollowPath.primary_controller',
                 'FollowPath.rotate_to_goal_heading', 'FollowPath.closed_loop',
                 'FollowPath.rotate_to_heading_angular_vel',
                 'FollowPath.min_approach_linear_velocity',
                 'progress_checker.plugin', 'progress_checker.required_movement_angle']
        request = GetParameters.Request(names=names)
        response = wait_future(node, client.call_async(request), 5.)
        report['loaded_controller_parameters'] = {
            name: dict(type=v.type, string=v.string_value, boolean=v.bool_value,
                       number=v.double_value) for name, v in zip(names, response.values)}
        assert response.values[0].string_value == 'nav2_rotation_shim_controller::RotationShimController'
        assert response.values[2].bool_value and not response.values[3].bool_value
        assert response.values[4].double_value == .20
        assert response.values[5].double_value == .05
        assert response.values[6].string_value == 'nav2_controller::PoseProgressChecker'
        node.destroy_client(client)
        for yaw in goal_yaws:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'learning_map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = goal_xy
            goal.pose.pose.orientation.z = math.sin(yaw/2)
            goal.pose.pose.orientation.w = math.cos(yaw/2)
            handle = wait_future(node, node.client.send_goal_async(goal), 5.)
            assert handle.accepted, 'Nav2 goal rejected'
            result = wait_future(node, handle.get_result_async(), 80.)
            until = time.monotonic()+2.5
            while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.01)
            distance = math.dist(node.pose[:2], goal_xy)
            error = abs(math.remainder(node.pose[2]-yaw, 2*math.pi))
            report['results'].append(dict(goal=goal_xy, yaw=yaw, status=result.status,
                final_pose=node.pose.copy(), distance_m=distance, yaw_error_rad=error,
                nonzero_commands=node.nonzero))
            assert result.status == GoalStatus.STATUS_SUCCEEDED, str(report['results'][-1])
            assert distance <= .15 and error <= .10, str(report['results'][-1])
            assert not node.command_errors, str(node.command_errors[:5])
        report['passed'] = True
    except Exception as error:
        report['error'] = repr(error); raise
    finally:
        for process in reversed(processes):
            if process.poll() is None: os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5.)
        report['process_exit_codes'] = [p.returncode for p in processes]
        for handle in handles: handle.close()
        events.close(); node.destroy_node(); rclpy.try_shutdown()
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
