from math import cos, isfinite, pi, sin
from statistics import median

from warehouse_robot_control.inspection_geometry import wrap


def waypoints(values):
    if len(values) < 2 or len(values) % 2 or not all(isfinite(v) for v in values):
        raise ValueError('Routes require finite x/y pairs')
    points = tuple(zip(values[::2], values[1::2]))
    if any(abs(x) > 3.5 or abs(y) > 2.5 for x, y in points):
        raise ValueError('Route exceeds warehouse bounds')
    return points


def home_wall_position(scan, heading, lidar_forward, west, north, south):
    distances = []
    for direction in (pi, pi/2, -pi/2):
        values = []
        for i, distance in enumerate(scan.ranges):
            error = wrap(scan.angle_min+i*scan.angle_increment+heading-direction)
            if abs(error) < .018 and isfinite(distance):
                projected = distance*cos(error)
                low, high = (.8, 1.6) if direction == pi else (2.5, 3.4)
                if low < projected < high:
                    values.append(projected)
        if len(values) < 2 or max(values)-min(values) > .08:
            return None
        distances.append(median(values))
    left, up, down = distances
    if not .8 < left < 1.6 or abs(up+down-(north-south)) > .08:
        return None
    x = west+left-lidar_forward*cos(heading)
    y = ((north-up)+(south+down))/2-lidar_forward*sin(heading)
    if abs(y) > .35:
        return None
    return x, y
