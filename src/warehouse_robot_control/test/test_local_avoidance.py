from math import atan2, hypot, pi
from types import SimpleNamespace

import numpy as np

from warehouse_robot_control.local_avoidance import ScanPlanner


def point_scan(position, points):
    ranges = np.full(720, np.inf)
    for x, y in points:
        dx, dy = x-position[0], y-position[1]
        index = round((atan2(dy, dx)+pi)/(2*pi)*719)
        ranges[index] = min(ranges[index], hypot(dx, dy))
    return SimpleNamespace(ranges=ranges, angle_min=-pi, angle_increment=2*pi/719,
                           range_min=.1, range_max=8.)


def test_scan_obstacle_requires_a_real_detour_and_retains_goal():
    planner = ScanPlanner()
    start, goal = (-1., 0.), (1., 0.)
    points = [(x, y) for x in np.linspace(-.15, .15, 15)
              for y in np.linspace(-.2, .2, 15)]
    planner.update(point_scan(start, points), start, 0., 0.)
    assert not planner.segment_clear(start, goal)
    plan = planner.plan(start, goal)
    assert plan and plan.points[-1] == goal and plan.length > 2.1
    assert max(abs(y) for x, y in plan.points) > .60
    assert planner.lookahead(start, plan.points) is not None


def test_changed_obstacle_position_changes_local_path_without_goal_changes():
    planner = ScanPlanner()
    start, goal = (-1., 0.), (1., 0.)
    paths = []
    for offset in (-.25, .25):
        points = [(x, y+offset) for x in np.linspace(-.15, .15, 12)
                  for y in np.linspace(-.15, .15, 12)]
        planner.update(point_scan(start, points), start, 0., 0.)
        plan = planner.plan(start, goal)
        assert plan
        paths.append(plan.points)
    assert paths[0] != paths[1]


def test_closed_corridor_has_no_path():
    planner = ScanPlanner()
    start, goal = (-1., 0.), (1., 0.)
    planner.update(point_scan(start, [(0., y) for y in np.linspace(-1.5, 1.5, 120)]), start, 0., 0.)
    assert planner.plan(start, goal) is None


def test_current_scan_clears_removed_obstacles():
    planner = ScanPlanner()
    start, goal = (-1., 0.), (1., 0.)
    planner.update(point_scan(start, [(0., 0.)]), start, 0., 0.)
    assert not planner.segment_clear(start, goal)
    planner.update(point_scan(start, []), start, 0., 0.)
    assert planner.segment_clear(start, goal)
    assert planner.plan(start, goal)
