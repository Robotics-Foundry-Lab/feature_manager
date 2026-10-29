"""Send navigation goals from the web UI to Nav2.

Nav2's bt_navigator takes goals as the NavigateToPose action (/navigate_to_pose), which reports
progress and the result. Without nav2_msgs installed the goal falls back to the /goal_pose topic
(what RViz's "2D Goal Pose" uses): the robot drives, but there is no progress or result.
Stopping always works, because it cancels all goals through the action's cancel service, which
only needs action_msgs.
"""

import copy
import logging
import math
import queue
import threading
import time
from typing import Any, Callable, Dict, List

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

try:
    from nav2_msgs.action import NavigateToPose
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
ACTIVE = ('sending', 'active', 'sent')


def _seconds(duration) -> float:
    return duration.sec + duration.nanosec * 1e-9


class NavGoalClient:
    """Goal state for the web UI.

    send() and cancel() are called from the web server's threads; all ROS calls are handed to
    the executor thread through a guard condition, so rclpy objects are only used there.
    """

    def __init__(self, node: Node, action: str = 'navigate_to_pose', topic: str = 'goal_pose'):
        self.node = node
        self.mode = 'action' if NavigateToPose is not None else 'topic'
        self.version = 0
        self._lock = threading.Lock()
        self._listeners: List[Callable[[Dict[str, Any]], None]] = []
        self._seq = 0            # id of the newest goal; callbacks of older goals are ignored
        self._handle = None
        self._cancel_seq = None  # goal that should be canceled as soon as Nav2 accepts it
        self._last_feedback = 0.0
        self._state: Dict[str, Any] = {
            'mode': self.mode, 'ready': False, 'goal': None, 'status': 'idle',
            'message': '', 'feedback': None,
        }

        if self.mode == 'action':
            self._client = ActionClient(node, NavigateToPose, action)
        else:
            LOG.warning('nav2_msgs not installed: goals go to /%s without progress', topic)
            self._pub = node.create_publisher(PoseStamped, topic, 10)
        self._cancel = node.create_client(CancelGoal, f'{action}/_action/cancel_goal')
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
            ready = self._client.server_is_ready()
        else:
            ready = self._pub.get_subscription_count() > 0
        self._update(ready=ready)

    # ------------------------------------------------------------ goals

    def send(self, x: float, y: float, yaw: float, frame_id: str) -> None:
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('x, y and yaw must be finite numbers')
        frame_id = frame_id.strip().lstrip('/')
        if not frame_id:
            raise ValueError('frame_id must not be empty')
        if not self._state['ready']:
            raise RuntimeError('Nav2 is not running. Start Navigation with Lidar SLAM + Nav2.')

        pose = PoseStamped()
        # The stamp stays 0: Nav2 then uses the latest transform into its global frame
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)

        with self._lock:
            self._seq += 1
            seq = self._seq
            self._handle = None
            self._cancel_seq = None
        goal = {'x': x, 'y': y, 'yaw': yaw, 'frame_id': frame_id, 'sent_at': time.time()}

        if self.mode == 'topic':
            self._call_soon(lambda: self._pub.publish(pose))
            self._update(seq, goal=goal, status='sent', feedback=None,
                         message='Sent on /goal_pose. Progress needs nav2_msgs in the feature '
                                 'manager image.')
            return

        def send_goal():
            future = self._client.send_goal_async(
                NavigateToPose.Goal(pose=pose),
                feedback_callback=lambda msg: self._on_feedback(seq, msg))
            future.add_done_callback(lambda f: self._on_response(seq, f))

        self._update(seq, goal=goal, status='sending', message='', feedback=None)
        self._call_soon(send_goal)

    def cancel(self) -> None:
        """Cancel every goal Nav2 is working on, also those of other nodes (exploration, RViz)."""
        if not self._cancel.service_is_ready():
            raise RuntimeError('Nav2 is not running, there is nothing to stop.')
        with self._lock:
            seq = self._seq
            if self._state['status'] == 'sending':
                self._cancel_seq = seq

        def done(future):
            try:
                canceling = len(future.result().goals_canceling)
            except Exception as exc:  # noqa: B902
                self._update(seq, message=f'Stop failed: {exc}')
                return
            status = self.snapshot()['status']
            if self.mode == 'action' and status in ('sending', 'active'):
                return  # reported by _on_response (pending cancel) or by the goal's result
            if status in ACTIVE:
                status = 'canceled'  # a /goal_pose goal never reports a result
            self._update(seq, status=status, message='Navigation stopped.' if canceling
                         else 'Nav2 had no active goal.')

        # An empty request (zero goal id and stamp) cancels all goals of the action server
        self._update(seq, message='Stopping…')
        self._call_soon(lambda: self._cancel.call_async(CancelGoal.Request())
                        .add_done_callback(done))

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
