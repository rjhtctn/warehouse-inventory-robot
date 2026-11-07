from dataclasses import dataclass
from math import atan, cos, hypot, isfinite, pi, sin, tan

import numpy as np


def wrap(angle):
    return (angle+pi) % (2*pi)-pi


def synchronized_image(images, lines, now, max_age=.25, tolerance=.075):
    if not lines:
        return None
    for received, image in reversed(images):
        stamp = image.header.stamp.sec+image.header.stamp.nanosec*1e-9
        if not -.05 <= now-stamp <= max_age:
            continue
        scan_stamp, line = min(lines, key=lambda pair: abs(pair[0]-stamp))
        if line is not None and abs(scan_stamp-stamp) <= tolerance:
            return received, image, line
    return None


def yaw_of(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(isfinite(v) for v in values) or abs(sum(v*v for v in values)-1) > .05:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = values
    return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)).item()


@dataclass(frozen=True)
class ShelfLine:
    distance: float
    heading_error: float
    inliers: int
    span: float
    residual: float


def camera_lateral(center_x, focal_x, principal_x, line, camera_forward, lidar_forward):
    bearing = atan((principal_x-center_x)/focal_x)
    camera_distance = line.distance-(camera_forward-lidar_forward)*cos(line.heading_error)
    return (-camera_forward*sin(line.heading_error)
            + camera_distance*tan(bearing-line.heading_error))


def fit_shelf(ranges, angle_min, angle_increment, heading, normal, desired_distance):
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min+np.arange(len(ranges))*angle_increment
    relative = angles+heading-normal
    forward = ranges*np.cos(angles)
    left = ranges*np.sin(angles)
    depth = ranges*np.cos(relative)
    lateral = ranges*np.sin(relative)
    valid = (np.isfinite(ranges) & (ranges > .1) & (np.cos(relative) > cos(pi/8))
             & (np.abs(lateral) < .95) & (np.abs(depth-desired_distance) < .75))
    if np.count_nonzero(valid) < 10:
        return None
    x, y, d = forward[valid], left[valid], depth[valid]
    # Disconnected box fronts can share a plane; gaps and the rear panel stay separate.
    bins, counts = np.unique(np.round(d/.025).astype(int), return_counts=True)
    for bin_id in bins[counts >= 6]:
        chosen = np.abs(d-bin_id*.025) < .045
        if np.count_nonzero(chosen) < 10 or np.ptp(y[chosen]) < .4:
            continue
        for _ in range(2):
            slope, intercept = np.linalg.lstsq(np.column_stack((y[chosen], np.ones(np.count_nonzero(chosen)))), x[chosen], rcond=None)[0]
            residuals = np.abs(x-slope*y-intercept)/hypot(1, slope)
            chosen = residuals < .018
            if np.count_nonzero(chosen) < 10:
                break
        if np.count_nonzero(chosen) < 10 or np.ptp(y[chosen]) < .4:
            continue
        error = -atan(slope)
        if abs(wrap(heading+error-normal)) > .12:
            continue
        rms = float(np.sqrt(np.mean(residuals[chosen]**2)))
        return ShelfLine(float(intercept/hypot(1, slope)), error,
                         int(np.count_nonzero(chosen)), float(np.ptp(y[chosen])), rms)
    return None


def sector_clearance(scan, direction, half_angle=pi/6):
    values = []
    for i, distance in enumerate(scan.ranges):
        if abs(wrap(scan.angle_min+i*scan.angle_increment-direction)) <= half_angle:
            if distance == float('inf'):
                values.append(scan.range_max)
            elif isfinite(distance) and distance >= 0:
                values.append(distance)
    return min(values) if values else 0.0
