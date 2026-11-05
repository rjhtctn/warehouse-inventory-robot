import math

import pytest

from warehouse_robot_control.avoidance import Avoidance, Config, ScanSummary, summarize_scan


def scan(front=3., left=3., right=3., nearest=1.):
    return ScanSummary(front, left, right, nearest)


def test_stale_scan_always_stops_and_requires_reacquisition():
    controller = Avoidance()
    assert controller.step(None, 0) == (0., 0.)
    assert controller.step(scan(), 1)[0] > 0
    assert controller.step(None, 2) == (0., 0.)
    assert controller.state == 'WAIT_FOR_SCAN'


@pytest.mark.parametrize('left,right,sign', [(2., 1., 1), (1., 2., -1)])
def test_braking_direction_lock_and_hysteresis(left, right, sign):
    controller = Avoidance()
    blocked = scan(.6, left, right, .6)
    assert controller.step(blocked, 0) == (0., 0.)
    assert controller.state == 'BRAKE'
    assert controller.step(blocked, .2) == (0., 0.)
    assert controller.step(blocked, .41)[1]*sign > 0
    assert controller.step(scan(.8, right, left, .6), 1)[1]*sign > 0
    for t in (1.1, 1.2):
        assert controller.step(scan(), t)[1]*sign > 0
    assert controller.step(scan(), 1.3)[0] > 0


def test_clear_count_requires_distinct_scans():
    controller = Avoidance()
    controller.step(scan(.6), 0)
    controller.step(scan(.6), .5)
    for i in range(20):
        assert controller.step(scan(), 1+i*.05, new_scan=False)[0] == 0
    assert controller.state == 'TURN'


def test_insufficient_rotation_clearance_and_turn_timeout_stop():
    controller = Avoidance()
    controller.step(scan(.6, nearest=.3), 0)
    assert controller.step(scan(.6, nearest=.3), .5) == (0., 0.)
    assert controller.state == 'BLOCKED'
    controller = Avoidance()
    controller.step(scan(.6), 0)
    controller.step(scan(.6), .5)
    assert controller.step(scan(.7), 11) == (0., 0.)
    assert controller.state == 'BLOCKED'


def test_metadata_angles_infinite_clear_ranges_and_narrow_obstacle():
    ranges = [math.inf]*721
    ranges[360] = .5
    result = summarize_scan(ranges, -math.pi, math.pi/360, .1, 8.)
    assert result.front == .5
    assert result.left == 8.
    ranges[360] = math.inf
    assert summarize_scan(ranges, -math.pi, math.pi/360, .1, 8.).front == 8.
    ranges[360] = .05
    assert summarize_scan(ranges, -math.pi, math.pi/360, .1, 8.).front == 0.


@pytest.mark.parametrize('ranges,increment', [([], .01), ([math.nan]*721, math.pi/360),
                                             ([math.inf]*721, 0), ([1.]*20, .01)])
def test_unusable_scans_are_rejected(ranges, increment):
    assert summarize_scan(ranges, -math.pi, increment, .1, 8.) is None


def test_invalid_front_sector_is_not_hidden_by_valid_rear_returns():
    ranges = [2.]*721
    ranges[320:401] = [math.nan]*81
    assert summarize_scan(ranges, -math.pi, math.pi/360, .1, 8.) is None


@pytest.mark.parametrize('values', [{'stop_distance': .95}, {'forward_speed': math.nan},
                                    {'clear_scans': 1.5}, {'turn_clearance': .8}])
def test_bad_configuration_is_rejected(values):
    with pytest.raises(ValueError):
        Config(**values)
