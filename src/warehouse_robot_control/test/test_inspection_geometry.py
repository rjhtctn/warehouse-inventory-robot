from math import atan2, cos, pi, sin, tan

import numpy as np
import pytest

from warehouse_robot_control.inspection_geometry import camera_lateral, fit_shelf, wrap


@pytest.mark.parametrize('yaw', [pi/2-.15, pi/2, pi/2+.15])
def test_disconnected_fronts_and_back_plane(yaw):
    angles = np.linspace(-pi, pi, 720)
    ranges = np.full(720, np.inf)
    for i, angle in enumerate(angles):
        global_angle = angle+yaw
        if sin(global_angle) <= 0:
            continue
        x = 2.4*cos(global_angle)/sin(global_angle)
        if any(abs(x-center) < .15 for center in (-.63, 0., .63)):
            ranges[i] = 2.4/sin(global_angle)
        elif abs(x) < .85:
            ranges[i] = 2.86/sin(global_angle)
    line = fit_shelf(ranges, -pi, 2*pi/719, yaw, pi/2, 2.4)
    assert line is not None
    assert abs(line.distance-2.4) < .005
    assert abs(line.heading_error-(pi/2-yaw)) < .005


@pytest.mark.parametrize('yaw', [pi/2-.15, pi/2, pi/2+.15])
def test_camera_lateral_is_invariant_under_in_place_turn(yaw):
    # Project a known point independently, including the camera's lever arm.
    from warehouse_robot_control.inspection_geometry import ShelfLine
    robot_x, robot_y = -.08, 0.
    camera_x, camera_y = robot_x+.16*cos(yaw), robot_y+.16*sin(yaw)
    bearing = wrap(atan2(2.44-camera_y, -camera_x)-yaw)
    pixel = 240-340*tan(bearing)
    lidar_distance = 2.44-.04*sin(yaw)
    line = ShelfLine(lidar_distance, pi/2-yaw, 30, 1.4, 0.)
    assert abs(camera_lateral(pixel, 340, 240, line, .16, .04)-robot_x) < 1e-6


def test_no_line_without_enough_geometry():
    assert fit_shelf([float('inf')]*720, -pi, 2*pi/719, pi/2, pi/2, 2.4) is None
    assert wrap(pi+.01) == pytest.approx(-pi+.01)


def test_latest_image_waiting_for_scan_uses_newest_synchronized_frame():
    from types import SimpleNamespace
    from warehouse_robot_control.inspection_geometry import synchronized_image
    def image(stamp):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=stamp)))
    earlier, latest = image(100000000), image(200000000)
    line = object()
    match = synchronized_image([(3., earlier), (3.1, latest)], [(1.1, line)], 1.22)
    assert match == (3., earlier, line)
    assert synchronized_image([(3., earlier)], [(1.1, line)], 1.5) is None
    assert synchronized_image([(3., earlier)], [(1.3, line)], 1.31) is None
