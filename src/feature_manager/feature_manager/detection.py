"""Live object detection image and NanoOWL settings for the web UI.

The annotated JPEGs of the object detection container (/yolov8_processed_image/compressed or
/nanoowl_processed_image/compressed, whichever publishes) are kept as the newest frame and
streamed to the browser as MJPEG by web.py. Only one detector runs at a time.

The NanoOWL query and score threshold go out on /nanoowl/query and /nanoowl/threshold. The node
echoes the values it uses on the latched /nanoowl/active_query and /nanoowl/active_threshold,
which is what the UI shows, so changes made with `ros2 topic pub` show up as well. A value set in
the UI is sent again until the node confirms it, and once more whenever the node (re)starts,
because a restarted node begins with NANOOWL_QUERY / NANOOWL_THRESHOLD.
"""

from collections import deque
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, String

IMAGE_TOPICS = {
    'nanoowl': '/nanoowl_processed_image/compressed',
    'yolov8': '/yolov8_processed_image/compressed',
}
MAX_QUERY_LEN = 500
THRESHOLD_RANGE = (0.01, 0.99)
RESEND_INTERVAL_S = 2.0
RESEND_TRIES = 5


def clean_query(text: str) -> str:
    """Normalize a comma-separated query: ' a person ,, a dog ' -> 'a person, a dog'."""
    labels = [label.strip() for label in str(text).split(',') if label.strip()]
    if not labels:
        raise ValueError('The query is empty. Enter things to detect, separated by commas.')
    query = ', '.join(labels)
    if len(query) > MAX_QUERY_LEN:
        raise ValueError(f'The query is longer than {MAX_QUERY_LEN} characters.')
    return query


def clean_threshold(value: float) -> float:
    """Validate a score threshold and round it to what the UI shows."""
    value = float(value)
    low, high = THRESHOLD_RANGE
    if not low <= value <= high:
        raise ValueError(f'The threshold must be between {low} and {high}.')
    return round(value, 2)


class ConfirmedSetting:
    """One NanoOWL setting: sent on `topic`, confirmed by the node on the latched `active_topic`.

    The value set last in the UI (`wanted`) is re-sent until the node echoes it, and again after
    the node restarted (its subscription disappeared and came back).
    """

    def __init__(self, node: Node, msg_type, topic: str, active_topic: str,
                 same: Callable[[Any, Any], bool] = lambda a, b: a == b):
        """Create the publisher and the latched subscription on `node`."""
        self._lock = threading.Lock()
        self._msg_type = msg_type
        self._same = same
        self.active = None
        self.wanted = None
        self._tries_left = 0
        self._had_subscriber = False
        self._pub = node.create_publisher(msg_type, topic, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(msg_type, active_topic, self._on_active, latched)

    def _on_active(self, msg):
        with self._lock:
            self.active = msg.data
            if self.wanted is not None and self._same(msg.data, self.wanted):
                self._tries_left = 0

    def resend(self):
        """Timer callback in the ROS thread: re-send until confirmed, and after node restarts."""
        has_subscriber = self.listeners() > 0
        with self._lock:
            restarted = has_subscriber and not self._had_subscriber
            self._had_subscriber = has_subscriber
            if self.wanted is None or not has_subscriber:
                return
            if restarted:
                self._tries_left = RESEND_TRIES
            if self._tries_left <= 0:
                return
            self._tries_left -= 1
            value = self.wanted
        self._pub.publish(self._msg_type(data=value))

    def set(self, value):
        """Send a new value (web thread)."""
        with self._lock:
            self.wanted = value
            self._tries_left = RESEND_TRIES
        self._pub.publish(self._msg_type(data=value))

    def listeners(self) -> int:
        """Return the number of subscribers, i.e. whether a NanoOWL node runs."""
        return self._pub.get_subscription_count()

    def pending(self) -> bool:
        """Return whether the value set last is not confirmed yet."""
        with self._lock:
            return self._tries_left > 0


class DetectionView:
    """Newest annotated frame and the NanoOWL settings.

    Image and confirmation callbacks run in the ROS executor thread; the web server reads through
    latest() / snapshot() and calls set_query() / set_threshold() from its own threads.
    """

    def __init__(self, node: Node):
        """Create the image subscriptions and the NanoOWL settings on `node`."""
        self._lock = threading.Lock()
        self._frame: Optional[bytes] = None
        self._seq = 0
        self._source: Optional[str] = None
        self._received_at = 0.0
        self._arrivals = deque(maxlen=20)

        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for source, topic in IMAGE_TOPICS.items():
            node.create_subscription(CompressedImage, topic,
                                     lambda msg, s=source: self._on_image(s, msg), image_qos)
        self.query = ConfirmedSetting(node, String, '/nanoowl/query', '/nanoowl/active_query')
        # Float32 does not hold 0.3 exactly
        self.threshold = ConfirmedSetting(node, Float32, '/nanoowl/threshold',
                                          '/nanoowl/active_threshold',
                                          same=lambda a, b: abs(a - b) < 1e-3)
        node.create_timer(RESEND_INTERVAL_S, self._resend)

    def _on_image(self, source: str, msg: CompressedImage):
        now = time.monotonic()
        with self._lock:
            self._frame = bytes(msg.data)
            self._seq += 1
            self._source = source
            self._received_at = now
            self._arrivals.append(now)

    def _resend(self):
        self.query.resend()
        self.threshold.resend()

    def set_query(self, text: str) -> str:
        """Send a new NanoOWL query and keep re-sending it until the node confirms it."""
        query = clean_query(text)
        self.query.set(query)
        return query

    def set_threshold(self, value: float) -> float:
        """Send a new NanoOWL score threshold and keep re-sending it until confirmed."""
        threshold = clean_threshold(value)
        self.threshold.set(threshold)
        return threshold

    def latest(self) -> Tuple[Optional[bytes], int]:
        """Return the newest JPEG frame and its sequence number."""
        with self._lock:
            return self._frame, self._seq

    def snapshot(self) -> Dict[str, Any]:
        """Return the image and settings state for the web UI."""
        now = time.monotonic()
        with self._lock:
            arrivals = [t for t in self._arrivals if now - t < 5.0]
            rate = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 2 else 0.0
            image = {
                'source': self._source,
                'age_s': now - self._received_at if self._frame else None,
                'rate_hz': round(rate, 1),
            }
        active_threshold = self.threshold.active
        return {
            **image,
            'active_query': self.query.active,
            'wanted_query': self.query.wanted,
            'query_pending': self.query.pending(),
            'query_listeners': self.query.listeners(),
            'active_threshold': None if active_threshold is None else round(active_threshold, 2),
            'wanted_threshold': self.threshold.wanted,
            'threshold_pending': self.threshold.pending(),
        }
