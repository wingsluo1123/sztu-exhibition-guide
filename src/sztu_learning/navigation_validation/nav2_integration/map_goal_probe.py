"""Actual Navfn actions on the frozen site map; no navigation or hardware API.

Frozen synthetic localization supplies TF only. Goals cover the connected map,
including paths beyond the former 2 m trial budget. This is planning evidence.
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

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from scipy.sparse.csgraph import connected_components
from std_msgs.msg import String

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from navigation_observation import observation_packet
from goal_navigation.grid_route import GridRoute
from nav2_integration.goal_selection import validate_goal, validate_route


class FrozenInput(Node):
    def __init__(self, grid, pose):
        super().__init__('learning_nav2_map_goal_probe')
        self.grid, self.pose = grid, pose
        self.nonzero = 0
        self.publisher = self.create_publisher(String, '/learning/nav/observation', 1)
        self.create_timer(.05, self.tick)
        self.create_subscription(Twist, '/learning_nav2/cmd_vel_raw', self.command, 1)
        self.planner = ActionClient(self, ComputePathToPose, '/learning_nav2/compute_path_to_pose')

    def tick(self):
        status = dict(valid=True, source_valid=True,
                      stamp=self.get_clock().now().nanoseconds*1e-9,
                      map_manifest_sha256=self.grid.manifest_hash)
        self.publisher.publish(String(data=json.dumps(observation_packet(status, self.pose, [0., 0., 0.]))))

    def command(self, message):
        self.nonzero += int(bool(message.linear.x or message.angular.z))


def wait(node, future, seconds=8.):
    end = time.monotonic()+seconds
    while not future.done():
        if time.monotonic() > end:
            raise TimeoutError('planner_probe_timeout')
        rclpy.spin_once(node, timeout_sec=.01)
    return future.result()


def coverage_goals(grid, start):
    # Positive controls need room for both the trial footprint and Nav2's
    # discretization padding. Do not pick barely admissible boundary cells as
    # guaranteed-success controls. The strict route gate still uses radius .40.
    grid = GridRoute(grid.grid, grid.resolution, grid.origin,
                     grid.radius+math.sqrt(2)*grid.resolution)
    _, labels = connected_components(grid.graph, directed=False)
    component = labels[np.ravel_multi_index(grid.cell(start), grid.grid.shape)]
    cells = np.argwhere(grid.allowed & (labels.reshape(grid.grid.shape) == component))
    # Interior quantiles avoid deliberately testing the most marginal boundary
    # cells as the only positive controls. Include all four map directions.
    points = np.array([grid.world(cell) for cell in cells])
    queries = [np.quantile(points, [qx, qy], axis=0)[[0, 1], [0, 1]]
               for qx, qy in [(a, b) for a in (.15, .5, .85) for b in (.15, .5, .85)]]
    selected = []
    for query in queries:
        target = points[np.argmin(np.linalg.norm(points-query, axis=1))].tolist()
        if target not in selected:
            selected.append(target)
    return selected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('requires isolated domain42/localhost')
    args.output.mkdir(parents=True, exist_ok=False)
    map_dir = HERE.parents[1]/'previous_learning/maps/room_7x3_20261120_map01'
    grid = GridRoute.load(map_dir, .40)
    pose = [.0079452679, -.0576307147, -1.8254150555]
    validate_goal(grid, pose[:2])
    report = dict(mode='actual_navfn_planning_only', synthetic_localization=True,
                  navigation_requested=False, hardware_api=False, results=[],
                  map_manifest_sha256=grid.manifest_hash, radius_m=.40, start=pose,
                  positive_control_padding_m=math.sqrt(2)*grid.resolution,
                  source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in [Path(__file__), HERE/'parameters.py', HERE/'goal_selection.py']})
    handles = []; processes = []
    rclpy.init(); node = FrozenInput(grid, pose)
    try:
        for name, cmd in [('adapter', [sys.executable, str(HERE/'input_adapter.py')]),
                          ('nav2', ['bash', str(HERE/'run_stack.bash')])]:
            handle = (args.output/(name+'.log')).open('x'); handles.append(handle)
            processes.append(subprocess.Popen(cmd, stdout=handle, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        end = time.monotonic()+45.
        while not node.planner.server_is_ready():
            assert time.monotonic() < end, 'planner discovery timeout'
            rclpy.spin_once(node, timeout_sec=.02)
        for name in ('map_server', 'planner_server', 'controller_server', 'bt_navigator'):
            client = node.create_client(GetState, '/learning_nav2/'+name+'/get_state')
            assert client.wait_for_service(timeout_sec=10.), name
            while wait(node, client.call_async(GetState.Request())).current_state.id != 3:
                assert time.monotonic() < end, name+' not active'
                rclpy.spin_once(node, timeout_sec=.05)
            node.destroy_client(client)
        report['runtime_parameters'] = {}
        for name in ('global_costmap', 'local_costmap'):
            client = node.create_client(GetParameters, '/learning_nav2/'+name+'/'+name+'/get_parameters')
            assert client.wait_for_service(timeout_sec=5.)
            keys = ['robot_radius', 'footprint_padding', 'inflation_layer.inflate_around_unknown']
            values = wait(node, client.call_async(GetParameters.Request(names=keys))).values
            assert values[0].double_value == .40
            assert abs(values[1].double_value-math.sqrt(2)*.05) < 1e-9
            assert values[2].bool_value
            report['runtime_parameters'][name] = dict(zip(keys, [v.bool_value if v.type == 1
                                                                                else v.double_value for v in values]))
            node.destroy_client(client)
        targets = coverage_goals(grid, pose[:2])
        for index, target in enumerate(targets):
            validate_goal(grid, target)
            yaw = -math.pi+index*math.pi/4
            goal = PoseStamped(); goal.header.frame_id = 'learning_map'
            goal.header.stamp = node.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = target
            goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            request = ComputePathToPose.Goal(); request.goal = goal; request.planner_id = 'GridBased'
            handle = wait(node, node.planner.send_goal_async(request))
            assert handle.accepted, 'planner rejected action'
            result = wait(node, handle.get_result_async())
            item = dict(goal=target, yaw_rad=yaw, status=result.status)
            report['results'].append(item)
            assert result.status == GoalStatus.STATUS_SUCCEEDED, str(item)
            route = [[p.pose.position.x, p.pose.position.y] for p in result.result.path.poses]
            item['path'] = route
            item['distance_m'] = validate_route(grid, route, target, start=pose[:2])
            end_goal = result.result.path.poses[-1].pose
            assert math.dist(route[-1], target) <= .05
            assert abs(math.remainder(2*math.atan2(end_goal.orientation.z, end_goal.orientation.w)-yaw,
                                      2*math.pi)) < 1e-6
            item['heading_preserved'] = True
            if item['distance_m'] > 2.:
                try:
                    validate_route(grid, route, target, max_distance=2., start=pose[:2])
                except ValueError as error:
                    assert str(error) == 'nav2_path_exceeds_approved_budget'
                    item['old_trial_budget_rejected_without_actuation'] = True
                else:
                    raise AssertionError('execution budget unexpectedly bypassed')
        assert len(targets) >= 6
        assert any(item['distance_m'] > 2. for item in report['results'])
        assert node.nonzero == 0, 'planning unexpectedly emitted a motion command'
        # Analytic invalid target controls also run through the production gate.
        report['invalid_controls'] = []
        for value in (-1, 100):
            cell = np.argwhere(grid.grid == value)[0]
            target = grid.world(cell).tolist()
            try:
                validate_goal(grid, target)
            except ValueError as error:
                report['invalid_controls'].append(dict(occupancy=value, goal=target, reason=str(error)))
            else:
                raise AssertionError('invalid goal admitted')
        report['passed'] = True
    except Exception as error:
        report['error'] = repr(error)
        raise
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5.)
        for handle in handles:
            handle.close()
        report['nonzero_commands'] = node.nonzero
        report['process_exit_codes'] = [p.returncode for p in processes]
        node.destroy_node(); rclpy.try_shutdown()
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k != 'results'}, indent=2), flush=True)


if __name__ == '__main__':
    main()
