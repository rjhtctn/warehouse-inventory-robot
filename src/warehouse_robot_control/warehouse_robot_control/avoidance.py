from dataclasses import dataclass
from math import isfinite, pi


@dataclass(frozen=True)
class ScanSummary:
    front: float
    left: float
    right: float
    nearest: float


def summarize_scan(ranges, angle_min, angle_increment, range_min, range_max):
    if (not ranges or not all(isfinite(v) for v in
            (angle_min, angle_increment, range_min, range_max))
            or angle_increment <= 0 or range_min < 0 or range_max <= range_min
            or (len(ranges) - 1) * angle_increment < 350 * pi / 180):
        return None
    sectors = {'front': [], 'left': [], 'right': []}
    expected = dict.fromkeys(sectors, 0)
    valid = []
    for i, value in enumerate(ranges):
        angle = (angle_min + i * angle_increment + pi) % (2*pi) - pi
        names = []
        if abs(angle) <= pi/6:
            names.append('front')
        if pi/4 <= angle <= 2*pi/3:
            names.append('left')
        if -2*pi/3 <= angle <= -pi/4:
            names.append('right')
        for name in names:
            expected[name] += 1
        if value == float('inf'):
            distance = range_max
        elif isfinite(value) and value >= 0:
            distance = 0.0 if value < range_min else min(value, range_max)
        else:
            continue
        valid.append(distance)
        for name in names:
            sectors[name].append(distance)
    if len(valid) < 0.8 * len(ranges):
        return None
    if any(not values or len(values) < 0.8 * expected[name]
           for name, values in sectors.items()):
        return None
    return ScanSummary(min(sectors['front']), min(sectors['left']),
                       min(sectors['right']), min(valid))


@dataclass(frozen=True)
class Config:
    forward_speed: float = 0.20
    turn_speed: float = 0.60
    stop_distance: float = 0.65
    resume_distance: float = 0.90
    turn_clearance: float = 0.42
    scan_timeout: float = 0.50
    brake_time: float = 0.40
    min_turn_time: float = 0.50
    max_turn_time: float = 10.0
    clear_scans: int = 3

    def __post_init__(self):
        if (any(not isfinite(v) or v <= 0 for v in vars(self).values())
                or self.resume_distance <= self.stop_distance
                or self.turn_clearance >= self.stop_distance
                or self.max_turn_time <= self.min_turn_time
                or int(self.clear_scans) != self.clear_scans):
            raise ValueError('Invalid obstacle avoidance parameters')


class Avoidance:
    def __init__(self, config=Config()):
        self.config = config
        self.reset()

    def reset(self):
        self.state = 'WAIT_FOR_SCAN'
        self.since = 0.0
        self.direction = 1.0
        self.clear_count = 0

    def step(self, scan, now, new_scan=True):
        c = self.config
        if scan is None:
            self.reset()
            return 0.0, 0.0
        if self.state == 'WAIT_FOR_SCAN':
            self.state = 'FORWARD'
        if self.state == 'BLOCKED':
            if scan.front >= c.resume_distance and scan.nearest >= c.turn_clearance:
                self.state = 'FORWARD'
            else:
                return 0.0, 0.0
        if self.state == 'FORWARD':
            if scan.front > c.stop_distance and scan.nearest > 0.25:
                return c.forward_speed, 0.0
            self.state = 'BRAKE'
            self.since = now
            self.clear_count = 0
            return 0.0, 0.0
        if self.state == 'BRAKE':
            if now - self.since < c.brake_time:
                return 0.0, 0.0
            if scan.nearest < c.turn_clearance:
                self.state = 'BLOCKED'
                return 0.0, 0.0
            self.direction = 1.0 if scan.left >= scan.right else -1.0
            self.state = 'TURN'
            self.since = now
        if self.state == 'TURN':
            if scan.nearest < c.turn_clearance or now-self.since >= c.max_turn_time:
                self.state = 'BLOCKED'
                return 0.0, 0.0
            if new_scan:
                self.clear_count = self.clear_count + 1 if scan.front >= c.resume_distance else 0
            if now-self.since >= c.min_turn_time and self.clear_count >= c.clear_scans:
                self.state = 'FORWARD'
                return c.forward_speed, 0.0
            return 0.0, self.direction*c.turn_speed
        return 0.0, 0.0
