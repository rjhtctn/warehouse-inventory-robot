from dataclasses import dataclass
import heapq
from math import ceil, cos, hypot, sin

import cv2
import numpy as np


@dataclass(frozen=True)
class LocalPlan:
    points: tuple
    length: float
    expanded: int


class ScanPlanner:
    """Short-lived obstacle geometry in the wheel/IMU navigation frame."""

    def __init__(self, resolution=.06, radius=.44, horizon=3.0, corridor=1.08):
        self.resolution = resolution
        self.radius = radius
        self.horizon = horizon
        self.corridor = corridor
        self.hits = np.empty((0, 2))
        self.origin = np.zeros(2)
        self.grid = None

    def update(self, scan, position, heading, lidar_forward):
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min+np.arange(len(ranges))*scan.angle_increment+heading
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges < scan.range_max-.02)
        sensor = np.asarray(position)+lidar_forward*np.array([cos(heading), sin(heading)])
        self.hits = sensor+np.column_stack((ranges[valid]*np.cos(angles[valid]),
                                            ranges[valid]*np.sin(angles[valid])))
        self.origin = np.floor(np.asarray(position)/self.resolution)*self.resolution-self.horizon
        size = int(ceil(2*self.horizon/self.resolution))+1
        occupied = np.zeros((size, size), np.uint8)
        cells = np.rint((self.hits-self.origin)/self.resolution).astype(int)
        inside = np.all((cells >= 0) & (cells < size), axis=1)
        cells = cells[inside]
        occupied[cells[:, 1], cells[:, 0]] = 1
        pixels = int(ceil(self.radius/self.resolution))
        yy, xx = np.ogrid[-pixels:pixels+1, -pixels:pixels+1]
        kernel = ((xx*self.resolution)**2+(yy*self.resolution)**2 <= self.radius**2).astype(np.uint8)
        self.grid = cv2.dilate(occupied, kernel)
        ys = self.origin[1]+np.arange(size)*self.resolution
        self.grid[np.abs(ys) > self.corridor, :] = 1
        self.clearance = cv2.distanceTransform(1-self.grid, cv2.DIST_L2, 3)*self.resolution

    def segment_clear(self, start, end, clearance=None):
        if self.grid is None:
            return False
        start, end = np.asarray(start), np.asarray(end)
        if max(abs(start[1]), abs(end[1])) > self.corridor:
            return False
        delta = end-start
        length2 = float(np.dot(delta, delta))
        if not len(self.hits):
            return True
        fractions = np.clip((self.hits-start) @ delta/max(length2, 1e-9), 0, 1)
        distances = np.linalg.norm(self.hits-start-fractions[:, None]*delta, axis=1)
        return bool(np.all(distances > (self.radius if clearance is None else clearance)))

    def plan(self, start, goal):
        if self.grid is None:
            return None
        grid = self.grid
        size = grid.shape[0]
        def cell(point):
            return tuple(np.rint((np.asarray(point)-self.origin)/self.resolution).astype(int))
        source, target = cell(start), cell(goal)
        if any(not 0 <= v < size for v in (*source, *target)) or grid[target[1], target[0]]:
            return None
        frontier = [(hypot(target[0]-source[0], target[1]-source[1]), 0., source)]
        costs = {source: 0.}
        parents = {}
        expanded = 0
        neighbors = ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1))
        while frontier and expanded < 12000:
            _, cost, current = heapq.heappop(frontier)
            if cost > costs[current]+1e-9:
                continue
            expanded += 1
            if current == target:
                cells = [current]
                while current != source:
                    current = parents[current]
                    cells.append(current)
                points = [tuple(start)]
                points.extend(tuple(self.origin+np.asarray(c)*self.resolution) for c in reversed(cells[:-1]))
                points[-1] = tuple(goal)
                return LocalPlan(tuple(points), sum(hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:])), expanded)
            x, y = current
            for dx, dy in neighbors:
                nx, ny = x+dx, y+dy
                if not (0 <= nx < size and 0 <= ny < size) or grid[ny, nx]:
                    continue
                if dx and dy and (grid[y, nx] or grid[ny, x]):
                    continue
                next_cost = cost+hypot(dx, dy)*(1+.08/(self.clearance[ny, nx]+.06))
                neighbor = nx, ny
                if next_cost < costs.get(neighbor, float('inf')):
                    costs[neighbor] = next_cost
                    parents[neighbor] = current
                    heapq.heappush(frontier, (next_cost+hypot(target[0]-nx, target[1]-ny), next_cost, neighbor))
        return None

    def lookahead(self, position, points, distance=.40):
        closest = min(range(len(points)), key=lambda i: hypot(points[i][0]-position[0], points[i][1]-position[1]))
        chosen = None
        for point in points[closest+1:]:
            if hypot(point[0]-position[0], point[1]-position[1]) > distance:
                break
            if self.segment_clear(position, point, self.radius-.025):
                chosen = point
        if chosen is None:
            for point in points[closest+1:closest+4]:
                if self.segment_clear(position, point, self.radius-.025):
                    return point
        return chosen
