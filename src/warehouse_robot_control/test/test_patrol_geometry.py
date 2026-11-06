from math import cos, pi, sin
from types import SimpleNamespace

import numpy as np
import pytest

from warehouse_robot_control.patrol_geometry import home_wall_position, waypoints


def test_routes_require_complete_finite_coordinates():
    assert waypoints([0., 0., 1., -.8]) == ((0., 0.), (1., -.8))
    for values in ([], [1.], [0., float('nan')], [4., 0.]):
        with pytest.raises(ValueError):
            waypoints(values)


@pytest.mark.parametrize('heading', [0., pi/2, -1., pi])
def test_home_fix_uses_live_wall_ranges_with_lidar_lever_arm(heading):
    x, y = -2.77, .04
    sx, sy = x+.04*cos(heading), y+.04*sin(heading)
    angles = np.linspace(-pi, pi, 720)
    ranges = []
    for angle in angles+heading:
        dx, dy = cos(angle), sin(angle)
        candidates = []
        if dx < 0:
            candidates.append((-3.94-sx)/dx)
        if dx > 0:
            candidates.append((3.94-sx)/dx)
        if dy > 0:
            candidates.append((2.94-sy)/dy)
        if dy < 0:
            candidates.append((-2.94-sy)/dy)
        ranges.append(min(candidates))
    scan = SimpleNamespace(ranges=ranges, angle_min=-pi, angle_increment=2*pi/719)
    result = home_wall_position(scan, heading, .04, -3.94, 2.94, -2.94)
    assert result == pytest.approx((x, y), abs=.001)
    # A peripheral beam may hit the rear crate while the wall remains visible.
    south_indices = [i for i, angle in enumerate(angles) if abs((angle+heading+pi/2+pi) % (2*pi)-pi) < .018]
    scan.ranges[south_indices[0]] = 2.25
    assert home_wall_position(scan, heading, .04, -3.94, 2.94, -2.94) == pytest.approx((x, y), abs=.001)


def test_home_fix_rejects_obstructed_or_missing_walls():
    scan = SimpleNamespace(ranges=[.5]*720, angle_min=-pi, angle_increment=2*pi/719)
    assert home_wall_position(scan, 0., .04, -3.94, 2.94, -2.94) is None
    scan.ranges = [float('inf')]*720
    assert home_wall_position(scan, 0., .04, -3.94, 2.94, -2.94) is None
