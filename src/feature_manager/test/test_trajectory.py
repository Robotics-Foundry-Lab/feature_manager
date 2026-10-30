import math

from feature_manager.trajectory import LOOKAHEAD, Route, TrajectoryStore
import pytest

SQUARE = [(0, 0), (4, 0), (4, 4), (0, 4)]


def test_open_route_ends_at_last_waypoint():
    route = Route(SQUARE + [(-4, 4), (-4, 0)])
    assert route.steps == 6
    assert route.window(0) == list(range(LOOKAHEAD))
    assert route.window(4) == [4, 5]
    assert route.is_last(5) and not route.is_last(4)


def test_closed_route_returns_to_first_waypoint():
    route = Route(SQUARE, closed=True)
    assert route.steps == 5
    assert route.point(4) == (0, 0)
    # A window never reaches the waypoint just passed, so its end is never at the robot
    assert route.window(1) == [1, 2, 3]
    assert route.window(3) == [3, 4]


def test_repeating_route_never_ends():
    route = Route(SQUARE[:3], closed=True, repeat=True)
    assert route.steps is None
    assert route.window(7) == [7, 8]
    assert [route.index(s) for s in route.window(7)] == [1, 2]
    assert route.lap(7) == 3
    assert not route.is_last(10 ** 6)


def test_passing_a_waypoint():
    route = Route(SQUARE, closed=True)
    assert route.passed(1, 3.7, 0.1)        # close to it
    assert route.passed(1, 4.6, 1.0)        # beyond it in driving direction
    assert not route.passed(1, 3.0, -0.6)   # still in front of it
    assert not route.passed(0, 1.0, 0.2)    # waypoint 1 has to be reached
    assert not route.passed(4, 0.0, 0.0)    # the last one ends the goal instead
    assert route.advance(1, 4.0, 0.0) == 2


def test_headings():
    route = Route(SQUARE, closed=True)
    assert route.yaw(1) == pytest.approx(math.pi / 4)   # corner: between east and north
    assert route.yaw(4) == pytest.approx(-math.pi / 2)  # arrive straight back at the start
    route = Route(SQUARE)
    assert route.yaw(0) == pytest.approx(0)
    assert route.yaw(3) == pytest.approx(math.pi)


@pytest.mark.parametrize('points, closed, repeat', [
    ([(0, 0)], False, False),
    ([(0, 0), (1, 1)], True, False),
    (SQUARE, False, True),
    ([(0, 0), (math.nan, 1)], False, False),
])
def test_invalid_routes(points, closed, repeat):
    with pytest.raises(ValueError):
        Route(points, closed, repeat)


def test_store(tmp_path):
    store = TrajectoryStore(tmp_path / 'state' / 'trajectories.json')
    assert store.list() == []
    store.save('  Hall  loop ', SQUARE, True, True, '/map')
    store.save('back', SQUARE[:2], False, False, 'map')
    saved = store.list()
    assert [t['name'] for t in saved] == ['back', 'Hall loop']
    assert saved[1]['frame_id'] == 'map' and saved[1]['repeat']
    assert saved[1]['waypoints'][1] == {'x': 4.0, 'y': 0.0}
    assert store.delete('back') and not store.delete('back')
    with pytest.raises(ValueError):
        store.save('a/b', SQUARE, False, False, 'map')
