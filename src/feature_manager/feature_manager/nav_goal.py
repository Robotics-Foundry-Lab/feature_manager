"""Send navigation goals from the web UI to Nav2.

Nav2's bt_navigator takes goals as the NavigateToPose action (/navigate_to_pose), which reports
progress and the result. Without nav2_msgs installed the goal falls back to the /goal_pose topic
(what RViz's "2D Goal Pose" uses): the robot drives, but there is no progress or result.
Stopping always works, because it cancels all goals through the action's cancel service, which
only needs action_msgs.

Waypoint trajectories go to Nav2's NavigateThroughPoses action (/navigate_through_poses) as a
window of the next few waypoints, which moves on every time the robot passed one (see
trajectory.py). They need nav2_msgs.
"""

import copy
import logging
import math
import queue
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .trajectory import Route

try:
    from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
    from nav2_msgs.srv import ClearEntireCostmap
    from rclpy.action import ActionClient
except ImportError:
    NavigateToPose = None

LOG = logging.getLogger('feature_manager.nav_goal')

FEEDBACK_INTERVAL_S = 0.5
RESULTS = {
    GoalStatus.STATUS_SUCCEEDED: ('succeeded', 'Goal reached.'),
    GoalStatus.STATUS_CANCELED: ('canceled', 'Goal canceled.'),
    GoalStatus.STATUS_ABORTED: ('aborted',
                                'Nav2 gave up: no path to the goal or the robot got stuck.'),
}
TRAJECTORY_RESULTS = {
    GoalStatus.STATUS_SUCCEEDED: ('succeeded', 'Trajectory finished.'),
    GoalStatus.STATUS_CANCELED: ('canceled', 'Trajectory stopped.'),
}
ACTIVE = ('sending', 'active', 'sent')
COSTMAPS = ('global_costmap', 'local_costmap')


def _seconds(duration) -> float:
    return duration.sec + duration.nanosec * 1e-9


class NavGoalClient:
    """Goal state for the web UI.

    send() and cancel() are called from the web server's threads; all ROS calls are handed to
    the executor thread through a guard condition, so rclpy objects are only used there.
    """

    def __init__(self, node: Node, action: str = 'navigate_to_pose', topic: str = 'goal_pose',
                 through_action: str = 'navigate_through_poses'):
        self.node = node
        self.mode = 'action' if NavigateToPose is not None else 'topic'
        self.version = 0
        self._lock = threading.Lock()
        self._listeners: List[Callable[[Dict[str, Any]], None]] = []
        self._seq = 0            # id of the newest goal; callbacks of older goals are ignored
        self._handle = None
        self._cancel_seq = None  # goal that should be canceled as soon as Nav2 accepts it
        self._last_feedback = 0.0
        # Running trajectory: the route, the step the robot drives towards, and the id of the
        # window goal sent last; feedback and results of replaced windows are ignored
        self._route: Optional[Route] = None
        self._route_frame = ''
        self._step = 0
        self._window = 0
        self._state: Dict[str, Any] = {
            'mode': self.mode, 'ready': False, 'trajectory_ready': False, 'goal': None,
            'trajectory': None, 'status': 'idle', 'message': '', 'feedback': None,
        }

        if self.mode == 'action':
            self._client = ActionClient(node, NavigateToPose, action)
            self._through = ActionClient(node, NavigateThroughPoses, through_action)
            self._clear_costmaps = [
                node.create_client(ClearEntireCostmap, f'{name}/clear_entirely_{name}')
                for name in COSTMAPS]
        else:
            LOG.warning('nav2_msgs not installed: goals go to /%s without progress', topic)
            self._pub = node.create_publisher(PoseStamped, topic, 10)
        self._cancel = [node.create_client(CancelGoal, f'{name}/_action/cancel_goal')
                        for name in (action, through_action)]
        self._calls: queue.SimpleQueue = queue.SimpleQueue()
        self._wake = node.create_guard_condition(self._run_calls)
        node.create_timer(1.0, self._check_ready)

    # ------------------------------------------------------------ state

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._state)

    def add_listener(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        self._listeners.append(callback)

    def _update(self, seq=None, **changes) -> bool:
        """Apply changes, unless they belong to a goal that was replaced in the meantime."""
        with self._lock:
            if seq is not None and seq != self._seq:
                return False
            if all(self._state.get(k) == v for k, v in changes.items()):
                return True
            self._state.update(changes)
            self.version += 1
            snapshot = copy.deepcopy(self._state)
        for listener in list(self._listeners):
            try:
                listener(snapshot)
            except Exception:  # noqa: B902 - a broken listener must not stop navigation
                LOG.exception('Navigation listener failed')
        return True

    def _call_soon(self, fn: Callable[[], None]) -> None:
        self._calls.put(fn)
        self._wake.trigger()

    def _run_calls(self):
        while not self._calls.empty():
            try:
                self._calls.get_nowait()()
            except Exception:  # noqa: B902
                LOG.exception('Navigation call failed')

    def _check_ready(self):
        if self.mode == 'action':
            self._update(ready=self._client.server_is_ready(),
                         trajectory_ready=self._through.server_is_ready())
        else:
            self._update(ready=self._pub.get_subscription_count() > 0)

    # ------------------------------------------------------------ goals

    @staticmethod
    def _pose(x: float, y: float, yaw: float, frame_id: str) -> PoseStamped:
        pose = PoseStamped()
        # The stamp stays 0: Nav2 then uses the latest transform into its global frame
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def _driving(self, trajectory: bool) -> bool:
        """Tell if a goal (trajectory=False) or a trajectory (True) of this client runs."""
        state = self._state
        return state['status'] in ACTIVE and bool(state['trajectory']) == trajectory

    def send(self, x: float, y: float, yaw: float, frame_id: str) -> None:
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('x, y and yaw must be finite numbers')
        frame_id = frame_id.strip().lstrip('/')
        if not frame_id:
            raise ValueError('frame_id must not be empty')
        if not self._state['ready']:
            raise RuntimeError('Nav2 is not running. Start Navigation with Lidar SLAM + Nav2.')
        # Nav2 runs one navigator at a time and rejects the other action meanwhile
        if self._driving(trajectory=True):
            raise RuntimeError('A trajectory is running. Stop it first.')
        pose = self._pose(x, y, yaw, frame_id)

        with self._lock:
            self._seq += 1
            seq = self._seq
            self._handle = None
            self._cancel_seq = None
        goal = {'x': x, 'y': y, 'yaw': yaw, 'frame_id': frame_id, 'sent_at': time.time()}

        if self.mode == 'topic':
            self._call_soon(lambda: self._pub.publish(pose))
            self._update(seq, goal=goal, trajectory=None, status='sent', feedback=None,
                         message='Sent on /goal_pose. Progress needs nav2_msgs in the feature '
                                 'manager image.')
            return

        def send_goal():
            future = self._client.send_goal_async(
                NavigateToPose.Goal(pose=pose),
                feedback_callback=lambda msg: self._on_feedback(seq, msg))
            future.add_done_callback(lambda f: self._on_response(seq, f))

        self._update(seq, goal=goal, trajectory=None, status='sending', message='',
                     feedback=None)
        self._call_soon(send_goal)

    def send_trajectory(self, route: Route, frame_id: str, name: str = '') -> None:
        """Drive to the first waypoint and through all others without stopping."""
        frame_id = frame_id.strip().lstrip('/')
        if not frame_id:
            raise ValueError('frame_id must not be empty')
        if self.mode != 'action':
            raise RuntimeError('Trajectories need nav2_msgs in the feature manager image.')
        if not self._state['trajectory_ready']:
            raise RuntimeError('Nav2 is not running. Start Navigation with Lidar SLAM + Nav2.')
        if self._driving(trajectory=False):
            raise RuntimeError('The robot is driving to a goal. Stop it first.')

        with self._lock:
            self._seq += 1
            seq = self._seq
            self._handle = None
            self._cancel_seq = None
            self._route, self._route_frame, self._step = route, frame_id, 0
        trajectory = {
            'name': name, 'frame_id': frame_id, 'closed': route.closed, 'repeat': route.repeat,
            'waypoints': [{'x': x, 'y': y} for x, y in route.points],
            'target': 0, 'lap': 1, 'sent_at': time.time(),
        }
        self._update(seq, goal=None, trajectory=trajectory, status='sending', message='',
                     feedback=None)
        self._call_soon(lambda: self._send_window(seq))

    def _send_window(self, seq):
        """Send the next waypoints as goal; Nav2 replaces the running one without stopping."""
        with self._lock:
            if seq != self._seq or self._cancel_seq == seq:
                return
            self._window += 1
            window, route, frame_id = self._window, self._route, self._route_frame
            steps = route.window(self._step)
        poses = [self._pose(*route.point(s), route.yaw(s), frame_id) for s in steps]
        future = self._through.send_goal_async(
            NavigateThroughPoses.Goal(poses=poses),
            feedback_callback=lambda msg: self._on_window_feedback(seq, window, steps, msg))
        future.add_done_callback(lambda f: self._on_window_response(seq, window, steps, f))

    def _current_window(self, seq, window) -> bool:
        with self._lock:
            return seq == self._seq and window == self._window

    def _set_step(self, seq, step) -> bool:
        with self._lock:
            if seq != self._seq:
                return False
            self._step = step
            route = self._route
            trajectory = dict(self._state['trajectory'])
        trajectory.update(target=route.index(step), lap=route.lap(step))
        return self._update(seq, trajectory=trajectory)

    def cancel(self) -> None:
        """Cancel every goal Nav2 is working on, also those of other nodes (exploration, RViz)."""
        clients = [c for c in self._cancel if c.service_is_ready()]
        if not clients:
            raise RuntimeError('Nav2 is not running, there is nothing to stop.')
        with self._lock:
            seq = self._seq
            # A trajectory must also stop sending its next waypoints
            if self._state['status'] == 'sending' or self._state['trajectory']:
                self._cancel_seq = seq
        answers = []

        def done(future):
            try:
                answers.append(len(future.result().goals_canceling))
            except Exception as exc:  # noqa: B902
                answers.append(exc)
            if len(answers) < len(clients):
                return
            counts = [a for a in answers if not isinstance(a, Exception)]
            if not counts:
                self._update(seq, message=f'Stop failed: {answers[0]}')
                return
            canceling = sum(counts)
            status = self.snapshot()['status']
            if self.mode == 'action' and status in ('sending', 'active'):
                return  # reported by _on_response (pending cancel) or by the goal's result
            if status in ACTIVE:
                status = 'canceled'  # a /goal_pose goal never reports a result
            self._update(seq, status=status, message='Navigation stopped.' if canceling
                         else 'Nav2 had no active goal.')

        # An empty request (zero goal id and stamp) cancels all goals of the action server
        self._update(seq, message='Stopping…')
        self._call_soon(lambda: [c.call_async(CancelGoal.Request()).add_done_callback(done)
                                 for c in clients])

    def clear_costmaps(self) -> None:
        """Empty Nav2's global and local costmap (after the map was reset)."""
        if self.mode != 'action':
            LOG.warning('Clearing the costmaps needs nav2_msgs')
            return

        def clear():
            for client in self._clear_costmaps:
                if client.service_is_ready():
                    client.call_async(ClearEntireCostmap.Request())
                else:
                    LOG.warning('%s is not available', client.srv_name)
        self._call_soon(clear)

    def _on_response(self, seq, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: B902
            self._update(seq, status='failed', message=f'Sending the goal failed: {exc}')
            return
        if not handle.accepted:
            self._update(seq, status='rejected', message='Nav2 rejected the goal.')
            return
        with self._lock:
            if seq != self._seq:
                return
            self._handle = handle
            cancel = self._cancel_seq == seq
        if not self._update(seq, status='active', message='Stopping…' if cancel else ''):
            return
        if cancel:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda f: self._on_result(seq, f))

    def _on_feedback(self, seq, msg):
        now = time.monotonic()
        if now - self._last_feedback < FEEDBACK_INTERVAL_S:
            return
        self._last_feedback = now
        fb = msg.feedback
        self._update(seq, feedback={
            'distance_remaining': round(float(fb.distance_remaining), 2),
            'eta_s': round(_seconds(fb.estimated_time_remaining), 1),
            'navigation_time_s': round(_seconds(fb.navigation_time), 1),
            'recoveries': int(fb.number_of_recoveries),
        })

    def _on_result(self, seq, future):
        try:
            code = future.result().status
        except Exception as exc:  # noqa: B902
            self._update(seq, status='failed', message=f'No result from Nav2: {exc}')
            return
        status, message = RESULTS.get(code, ('failed', f'Goal ended with status {code}.'))
        with self._lock:
            if seq == self._seq:
                self._handle = None
        self._update(seq, status=status, message=message)

    # ------------------------------------------------------------ trajectory windows

    def _on_window_response(self, seq, window, steps, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: B902
            if self._current_window(seq, window):
                self._update(seq, status='failed', message=f'Sending the trajectory failed: {exc}')
            return
        with self._lock:
            if seq != self._seq or window != self._window:
                return  # a newer window replaces this goal anyway
            if handle.accepted:
                self._handle = handle
            cancel = self._cancel_seq == seq
        if not handle.accepted:
            self._update(seq, status='rejected', message='Nav2 rejected the trajectory.')
            return
        if not self._update(seq, status='active', message='Stopping…' if cancel else ''):
            return
        if cancel:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f: self._on_window_result(seq, window, steps, f))

    def _on_window_feedback(self, seq, window, steps, msg):
        with self._lock:
            if seq != self._seq or window != self._window or self._cancel_seq == seq:
                return
            route, frame_id, step = self._route, self._route_frame, self._step
        fb = msg.feedback
        # Nav2 drops waypoints the robot came within 0.7 m of (RemovePassedGoals in the BT) ...
        new_step = step
        remaining = int(fb.number_of_poses_remaining)
        if 0 < remaining <= len(steps):
            new_step = max(step, steps[-1] + 1 - remaining)
        # ... and the robot may also pass one at a larger distance, e.g. around an obstacle
        position = fb.current_pose.pose.position
        same_frame = fb.current_pose.header.frame_id.lstrip('/') == frame_id
        if same_frame:
            new_step = route.advance(new_step, position.x, position.y)
        if new_step > step and self._set_step(seq, new_step):
            self._send_window(seq)

        now = time.monotonic()
        if now - self._last_feedback < FEEDBACK_INTERVAL_S:
            return
        self._last_feedback = now
        distance = None
        if same_frame:
            x, y = route.point(new_step)
            distance = round(math.hypot(x - position.x, y - position.y), 2)
        self._update(seq, feedback={
            'distance_to_waypoint': distance,
            'navigation_time_s': round(_seconds(fb.navigation_time), 1),
            'recoveries': int(fb.number_of_recoveries),
        })

    def _on_window_result(self, seq, window, steps, future):
        if not self._current_window(seq, window):
            return  # replaced by the next window: Nav2 aborts the old goal
        try:
            code = future.result().status
        except Exception as exc:  # noqa: B902
            self._update(seq, status='failed', message=f'No result from Nav2: {exc}')
            return
        with self._lock:
            route = self._route
            stopping = self._cancel_seq == seq
            if seq == self._seq:
                self._handle = None
        if code == GoalStatus.STATUS_SUCCEEDED and not route.is_last(steps[-1]):
            if stopping:
                code = GoalStatus.STATUS_CANCELED
            else:
                # The robot reached the end of the window before it was moved on: keep going
                if self._set_step(seq, steps[-1] + 1):
                    self._send_window(seq)
                return
        status, message = TRAJECTORY_RESULTS.get(code, (None, None))
        if status is None:
            status, message = RESULTS.get(
                code, ('failed', f'Trajectory ended with status {code}.'))
            if code == GoalStatus.STATUS_ABORTED:
                target = route.index(self._step) + 1
                message = (f'Nav2 gave up on the way to waypoint {target}: no path there or the '
                           'robot got stuck.')
        self._update(seq, status=status, message=message)
