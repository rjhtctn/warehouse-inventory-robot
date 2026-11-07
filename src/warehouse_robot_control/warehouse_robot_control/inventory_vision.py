from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Box:
    color: str
    x: int
    y: int
    width: int
    height: int
    area: int

    @property
    def center(self):
        return self.x+self.width/2, self.y+self.height/2


@dataclass(frozen=True)
class Observation:
    valid: bool
    reason: str
    upper: tuple = ()
    lower: tuple = ()
    split: float = 0.0

    @property
    def counts(self):
        return tuple(sum(b.color == color for b in row)
                     for row in (self.upper, self.lower) for color in ('red', 'blue'))

    @property
    def center_x(self):
        boxes = self.upper+self.lower
        return (min(b.x for b in boxes)+max(b.x+b.width for b in boxes))/2


def detect_boxes(rgb):
    if rgb is None or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        return Observation(False, 'Invalid RGB image')
    height, width = rgb.shape[:2]
    if height < 120 or width < 160:
        return Observation(False, 'Image too small')
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    red = cv2.inRange(hsv, (0, 115, 45), (12, 255, 255))
    red |= cv2.inRange(hsv, (170, 115, 45), (179, 255, 255))
    blue = cv2.inRange(hsv, (95, 115, 45), (130, 255, 255))
    boxes = []
    kernel = np.ones((3, 3), np.uint8)
    for color, mask in (('red', red), ('blue', blue)):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
        for x, y, w, h, area in stats[1:count]:
            x, y, w, h, area = map(int, (x, y, w, h, area))
            if area < max(55, width*height*.0003):
                continue
            if x < 6 or y < 6 or x+w > width-6 or y+h > height-6:
                return Observation(False, 'Colored object clipped by image boundary')
            if (w < 8 or h < 10 or not .65 <= w/h <= 1.90
                    or area/(w*h) < .78 or area > width*height*.08):
                return Observation(False, 'Occluded, merged, or irregular colored object')
            boxes.append(Box(color, x, y, w, h, area))
    if len(boxes) < 2:
        return Observation(False, 'Both shelf levels are not visible')
    boxes.sort(key=lambda box: box.center[1])
    gaps = [b.center[1]-a.center[1] for a, b in zip(boxes, boxes[1:])]
    index = int(np.argmax(gaps))+1
    upper, lower = tuple(boxes[:index]), tuple(boxes[index:])
    size = float(np.median([box.height for box in boxes]))
    upper_bottom = max(b.y+b.height for b in upper)
    lower_top = min(b.y for b in lower)
    if gaps[index-1] < 1.25*size or lower_top-upper_bottom < .35*size:
        return Observation(False, 'Shelf levels are not sufficiently separated')
    for row in (upper, lower):
        centers_y = [b.center[1] for b in row]
        heights = [b.height for b in row]
        if max(centers_y)-min(centers_y) > .5*size or max(heights)/min(heights) > 1.5:
            return Observation(False, 'Inconsistent box visibility within shelf level')
        ordered = sorted(row, key=lambda b: b.x)
        if any(b.x-(a.x+a.width) < 3 for a, b in zip(ordered, ordered[1:])):
            return Observation(False, 'Boxes are not individually separated')
    return Observation(True, '', upper, lower, (upper_bottom+lower_top)/2)


def annotate(rgb, observation, stable_frames=0):
    image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if not observation.valid:
        cv2.putText(image, observation.reason, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    .43, (0, 180, 255), 1, cv2.LINE_AA)
        return image
    cv2.line(image, (0, round(observation.split)), (image.shape[1]-1, round(observation.split)),
             (190, 190, 190), 1)
    for name, row in (('U', observation.upper), ('L', observation.lower)):
        for box in row:
            color = (0, 40, 255) if box.color == 'red' else (255, 150, 20)
            cv2.rectangle(image, (box.x, box.y), (box.x+box.width, box.y+box.height), color, 1)
            cv2.putText(image, name+' '+box.color, (box.x, max(12, box.y-3)),
                        cv2.FONT_HERSHEY_SIMPLEX, .34, color, 1, cv2.LINE_AA)
    ur, ub, lr, lb = observation.counts
    for y, text in [(image.shape[0]-40, f'Upper: red {ur}  blue {ub}'),
                    (image.shape[0]-24, f'Lower: red {lr}  blue {lb}'),
                    (image.shape[0]-8, f'Total: {ur+ub+lr+lb}  Stable frames: {stable_frames}')]:
        cv2.putText(image, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, .42,
                    (240, 240, 240), 1, cv2.LINE_AA)
    return image
