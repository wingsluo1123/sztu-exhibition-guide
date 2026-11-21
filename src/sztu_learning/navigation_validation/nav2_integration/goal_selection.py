"""Map-wide goal admission, separate from a physical attempt's travel budget.

Navfn computes routes; this module checks the saved map's known clearance,
including the exact clicked endpoint (Navfn may return a nearby endpoint).
"""
import math


def validate_goal(grid, xy, base_pose=None, relative_area=None, area_radius=.20):
    if len(xy) != 2 or not all(math.isfinite(v) for v in xy):
        raise ValueError('nonfinite_goal')
    if not grid.allowed[grid.cell(xy)]:
        raise ValueError('goal_lacks_known_clearance')
    if relative_area is not None:
        x, y, yaw = base_pose
        forward, left = relative_area
        expected = (x+math.cos(yaw)*forward-math.sin(yaw)*left,
                    y+math.sin(yaw)*forward+math.cos(yaw)*left)
        if math.dist(xy, expected) > area_radius:
            raise ValueError('goal_outside_approved_selection_area')


def validate_route(grid, route, target, max_distance=None, start=None):
    if not route:
        raise ValueError('nav2_empty_path')
    if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in route):
        raise ValueError('nav2_nonfinite_path')
    validate_goal(grid, target)
    points = ([start] if start is not None else []) + list(route) + [target]
    if not all(grid.segment_clear(a, b) for a, b in zip(points, points[1:])):
        raise ValueError('nav2_path_clearance_unverified')
    distance = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    if max_distance is not None and distance > max_distance:
        raise ValueError('nav2_path_exceeds_approved_budget')
    return distance
