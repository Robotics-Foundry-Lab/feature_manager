"""Waypoint trajectories: which waypoints Nav2 drives through next, and named trajectories on disk.

Nav2 drives through poses with the NavigateThroughPoses action without stopping in between, but
its controller ends the goal as soon as the robot is near the last pose of the path. Handing Nav2
a closed loop at once would therefore stop the robot already at waypoint 1, and a loop that
repeats forever cannot be handed over at all. So nav_goal.py sends a window of the next few
waypoints and moves it on (preempting the running goal, which Nav2 does without stopping) every
time the robot passed a waypoint. The end of the window always stays a few waypoints ahead.

Waypoints are numbered by step: step i is waypoint i % n. An open trajectory has n steps, a closed
one n + 1 (back to waypoint 1), a repeating one is endless.
"""

import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

LOOKAHEAD = 4          # waypoints per NavigateThroughPoses goal
PASS_RADIUS = 0.5      # m: a waypoint counts as passed this close to it ...
CAPTURE_RADIUS = 1.5   # ... or this close once the robot is beyond it in driving direction
MIN_WAYPOINTS = 2      # a single waypoint is a goal
MIN_LOOP_WAYPOINTS = 3
MAX_WAYPOINTS = 200
MAX_NAME = 60

Point = Tuple[float, float]


def _unit(a: Point, b: Point) -> Point:
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    return (dx / d, dy / d) if d > 1e-6 else (0.0, 0.0)


class Route:
    """The waypoints of one trajectory and how to drive through them."""

    def __init__(self, points: Sequence[Point], closed: bool = False, repeat: bool = False):
        points = [(float(x), float(y)) for x, y in points]
        if not all(math.isfinite(v) for p in points for v in p):
            raise ValueError('Waypoints must be finite numbers')
        if len(points) < MIN_WAYPOINTS:
            raise ValueError(f'A trajectory needs at least {MIN_WAYPOINTS} waypoints')
        if len(points) > MAX_WAYPOINTS:
            raise ValueError(f'A trajectory has at most {MAX_WAYPOINTS} waypoints')
        if closed and len(points) < MIN_LOOP_WAYPOINTS:
            raise ValueError(f'A closed loop needs at least {MIN_LOOP_WAYPOINTS} waypoints')
        if repeat and not closed:
            raise ValueError('Only a closed loop can repeat')
        self.points = points
        self.closed = bool(closed)
        self.repeat = bool(repeat)
        self.steps: Optional[int] = None if repeat else len(points) + (1 if closed else 0)
        # A window must not reach the waypoint the robot just passed: that would be the path's end
        self.lookahead = min(LOOKAHEAD, len(points) - 1) if closed else LOOKAHEAD
        # Heading at each waypoint: the mean of the directions in and out of it
        n = len(points)
        self._headings = []
        for i, p in enumerate(points):
            vx = vy = 0.0
            if i > 0 or closed:
                ux, uy = _unit(points[i - 1], p)
                vx, vy = vx + ux, vy + uy
            if i < n - 1 or closed:
                ux, uy = _unit(p, points[(i + 1) % n])
                vx, vy = vx + ux, vy + uy
            self._headings.append(math.atan2(vy, vx) if math.hypot(vx, vy) > 1e-6 else 0.0)

    def index(self, step: int) -> int:
        return step % len(self.points)

    def point(self, step: int) -> Point:
        return self.points[self.index(step)]

    def lap(self, step: int) -> int:
        return step // len(self.points) + 1

    def is_last(self, step: int) -> bool:
        return self.steps is not None and step >= self.steps - 1

    def yaw(self, step: int) -> float:
        """Heading the robot should have at the waypoint of this step."""
        if self.is_last(step):
            # Arrive straight, without turning on the spot at the end
            (ux, uy) = _unit(self.point(step - 1), self.point(step))
            if ux or uy:
                return math.atan2(uy, ux)
        return self._headings[self.index(step)]

    def window(self, step: int) -> List[int]:
        """Return the steps of the goal to send while the robot drives towards `step`."""
        end = step + self.lookahead
        if self.steps is not None:
            end = min(end, self.steps)
        return list(range(step, max(end, step + 1)))

    def passed(self, step: int, x: float, y: float) -> bool:
        """Did the robot at (x, y) pass the waypoint of `step`? Never true for the last one."""
        if self.is_last(step):
            return False
        px, py = self.point(step)
        dx, dy = x - px, y - py
        d = math.hypot(dx, dy)
        if d < PASS_RADIUS:
            return True
        if step == 0:
            return False  # the robot comes from anywhere, it has to drive to waypoint 1 first
        yaw = self._headings[self.index(step)]
        return d < CAPTURE_RADIUS and dx * math.cos(yaw) + dy * math.sin(yaw) > 0

    def advance(self, step: int, x: float, y: float) -> int:
        """First step whose waypoint the robot has not passed yet."""
        for _ in range(len(self.points)):
            if not self.passed(step, x, y):
                break
            step += 1
        return step


def clean_name(name: str) -> str:
    name = ' '.join(str(name).split())
    if not name:
        raise ValueError('The trajectory needs a name')
    if len(name) > MAX_NAME:
        raise ValueError(f'Names have at most {MAX_NAME} characters')
    if '/' in name:
        raise ValueError('Names must not contain "/"')
    return name


class TrajectoryStore:
    """Named trajectories in a JSON file, shared by all browsers."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self._lock = threading.Lock()

    def _read(self) -> Dict[str, Any]:
        try:
            with open(self.path, 'r', encoding='utf-8') as fh:
                data = json.load(fh)
        except FileNotFoundError:
            return {}
        return dict(data.get('trajectories') or {})

    def _write(self, trajectories: Dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix('.tmp')
        with open(tmp, 'w', encoding='utf-8') as fh:
            json.dump({'version': 1, 'trajectories': trajectories}, fh, indent=1)
        os.replace(tmp, self.path)

    def list(self) -> List[Dict[str, Any]]:
        with self._lock:
            trajectories = self._read()
        return sorted(trajectories.values(), key=lambda t: t['name'].casefold())

    def save(self, name: str, points: Sequence[Point], closed: bool, repeat: bool,
             frame_id: str) -> Dict[str, Any]:
        name = clean_name(name)
        route = Route(points, closed, repeat)  # validates
        frame_id = frame_id.strip().lstrip('/')
        if not frame_id:
            raise ValueError('frame_id must not be empty')
        entry = {
            'name': name,
            'frame_id': frame_id,
            'waypoints': [{'x': round(x, 3), 'y': round(y, 3)} for x, y in route.points],
            'closed': route.closed,
            'repeat': route.repeat,
            'updated_at': time.time(),
        }
        with self._lock:
            trajectories = self._read()
            trajectories[name] = entry
            self._write(trajectories)
        return entry

    def delete(self, name: str) -> bool:
        with self._lock:
            trajectories = self._read()
            if trajectories.pop(name, None) is None:
                return False
            self._write(trajectories)
        return True
