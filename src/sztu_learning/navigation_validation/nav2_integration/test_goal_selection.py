"""Spatial freedom must not bypass footprint, endpoint or execution budgets."""
import math

import numpy as np
import pytest

from goal_navigation.grid_route import GridRoute
from nav2_integration.goal_selection import validate_goal, validate_route


def room():
    grid = np.zeros((80, 100), dtype=int)
    grid[20:30, 20:30] = -1
    grid[40:50, 40:50] = 100
    return GridRoute(grid, .1, [0., 0., 0.], .40)


@pytest.mark.parametrize('xy', [(1., 1.), (8., 6.), (2., 6.), (7., 2.)])
def test_free_goals_away_from_old_trial_point_are_accepted(xy):
    validate_goal(room(), xy, [1., 1., 0.])


@pytest.mark.parametrize('xy', [(2.5, 2.5), (4.5, 4.5), (.1, 1.), (4.5, 3.9)])
def test_unknown_wall_border_and_inadequate_clearance_rejected(xy):
    with pytest.raises(ValueError, match='goal_lacks_known_clearance'):
        validate_goal(room(), xy)


@pytest.mark.parametrize('xy', [(-1., 2.), (100., 2.), (math.nan, 2.)])
def test_outside_or_nonfinite_rejected(xy):
    with pytest.raises(ValueError):
        validate_goal(room(), xy)


def test_trial_area_remains_opt_in_and_rotates_with_robot():
    grid = room()
    validate_goal(grid, [7., 2.], [7., 1., math.pi/2], [1., 0.])
    with pytest.raises(ValueError, match='approved_selection_area'):
        validate_goal(grid, [8., 2.], [7., 1., math.pi/2], [1., 0.])


def test_map_wide_plan_is_independent_of_physical_budget():
    grid = room(); route = [[1., 6.], [8., 6.]]
    assert validate_route(grid, route, [8., 6.]) == pytest.approx(7.)
    with pytest.raises(ValueError, match='approved_budget'):
        validate_route(grid, route, [8., 6.], max_distance=2.)
    assert validate_route(grid, route, [8., 6.], max_distance=8.) == pytest.approx(7.)


def test_route_end_and_start_connections_cannot_cut_through_obstacle():
    grid = room()
    # Both endpoints are free, but their connecting line crosses the wall.
    with pytest.raises(ValueError, match='clearance_unverified'):
        validate_route(grid, [[3., 4.5]], [6., 4.5])
    with pytest.raises(ValueError, match='clearance_unverified'):
        validate_route(grid, [[6., 4.5]], [6., 4.5], start=[3., 4.5])
    with pytest.raises(ValueError, match='clearance_unverified'):
        validate_route(grid, [[3., 4.5], [6., 4.5]], [6., 4.5])


def test_empty_nonfinite_and_single_pose_paths():
    grid = room()
    with pytest.raises(ValueError, match='empty_path'):
        validate_route(grid, [], [1., 1.])
    with pytest.raises(ValueError, match='nonfinite_path'):
        validate_route(grid, [[math.inf, 1.]], [1., 1.])
    assert validate_route(grid, [[1., 1.]], [1., 1.], start=[1., 1.]) == 0.


def test_plan_only_main_accepts_long_rviz_goal_without_requesting_navigation(monkeypatch, tmp_path):
    """Run the production orchestration with deterministic planner/readiness seams.

    Real action/TF transport is covered by map_goal_probe. This regression checks
    that the plan-only branch cannot fall through to NavigateToPose or motors.
    """
    from concurrent.futures import Future
    import json
    from types import SimpleNamespace as NS
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from nav2_integration import mission

    def ready(value):
        f = Future(); f.set_result(value); return f

    goal = PoseStamped(); goal.header.frame_id = 'learning_map'
    goal.pose.position.x = 8.; goal.pose.position.y = 6.
    goal.pose.orientation.z, goal.pose.orientation.w = .6, .8
    first = PoseStamped(); first.pose.position.x = 1.; first.pose.position.y = 6.
    path = Path(poses=[first, goal])
    planner_requests = []; paths = []; goals = []; announcements = []

    def plan(request):
        planner_requests.append(request)
        return ready(NS(accepted=True, get_result_async=lambda: ready(NS(
            status=4, result=NS(path=path)))))

    def forbidden(*_):
        raise AssertionError('plan-only requested navigation')

    grid = room(); grid.manifest_hash = 'fixture-map'
    clock = NS(now=lambda: NS(to_msg=lambda: goal.header.stamp))
    node = NS(status={'map_manifest_sha256':'fixture-map'}, goal_message=goal,
              map_pose=np.array([1., 6., 0.]), odom=[1., 6., 0.], map_matrix=np.eye(3), wire=[],
              get_node_names=lambda: [], navigation_ready=lambda: True, get_clock=lambda: clock,
              planner=NS(server_is_ready=lambda:True, send_goal_async=plan),
              navigator=NS(server_is_ready=lambda:True, send_goal_async=forbidden),
              path_pub=NS(publish=paths.append), goal_pub=NS(publish=goals.append),
              announce=lambda *v:announcements.append(v), destroy_node=lambda:None)
    monkeypatch.setattr(mission, 'Nav2Observer', lambda: node)
    monkeypatch.setattr(mission.GridRoute, 'load', lambda *_:grid)
    monkeypatch.setattr(mission, 'wait_ready', lambda *_:True)
    monkeypatch.setattr(mission, 'readiness_snapshot', lambda _: {})
    monkeypatch.setattr(mission, 'service_observations', lambda _:None)
    monkeypatch.setattr(mission.subprocess, 'run', lambda *_:NS(returncode=1))
    monkeypatch.setattr(mission.rclpy, 'init', lambda:None)
    monkeypatch.setattr(mission.rclpy, 'try_shutdown', lambda:None)
    monkeypatch.setattr(mission.signal, 'signal', lambda *_:None)
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    output = tmp_path/'plan_only'
    monkeypatch.setattr(mission.sys, 'argv', ['mission.py', '--rviz-goal', '--plan-only',
                                             '--output', str(output)])
    mission.main()
    report = json.loads((output/'report.json').read_text())
    assert report.get('error') is None
    assert report['result'] == 'planned_only_no_navigation_or_motor'
    assert report['planned_distance_m'] == pytest.approx(7.)
    assert report['max_distance_m'] is None
    assert not report['motor_registered'] and report['nonzero_commands'] == 0
    assert len(planner_requests) == len(paths) == len(goals) == 1
    assert planner_requests[0].goal.pose == goal.pose
