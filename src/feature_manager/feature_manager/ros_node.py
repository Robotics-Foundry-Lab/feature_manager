"""ROS 2 interface: publish feature status and accept start/stop commands."""

import json

from rclpy.node import Node
from std_msgs.msg import String

from .controller import FeatureController
from .nav_goal import NavGoalClient


def parse_command(text: str):
    """Parse a feature command.

    JSON: {"action": "start", "name": "navigation", "options": {"USE_NVBLOX": false}}
    Text: "start navigation USE_NVBLOX=false", "stop rplidar", "toggle joy_to_twist"
    """
    text = text.strip()
    try:
        data = json.loads(text)
    except ValueError:
        data = None
    if isinstance(data, dict):
        return data.get('action'), data.get('name') or data.get('feature'), data.get('options')

    parts = text.split()
    if len(parts) < 2:
        return None, None, None
    options = dict(p.split('=', 1) for p in parts[2:] if '=' in p)
    return parts[0].lower(), parts[1], options or None


class FeatureManagerNode(Node):

    def __init__(self, controller: FeatureController, interval: float = 5.0):
        super().__init__('feature_manager')
        self.controller = controller
        self.pub = self.create_publisher(String, '/feature_manager/status', 10)
        self.create_subscription(String, '/feature_manager/cmd', self.on_command, 10)
        # Publish periodically and additionally on every status change
        self.create_timer(interval, self.publish_status)
        controller.add_listener(lambda _snapshot: self.publish_status())
        self.get_logger().info('Listening for commands on /feature_manager/cmd')
        # Navigation goals from the 3D map of the web UI
        self.nav = NavGoalClient(self)

    def publish_status(self):
        snapshot = self.controller.snapshot()
        features = {
            fid: {k: s[k] for k in ('state', 'running', 'pending', 'active_options',
                                    'started_at', 'cpu_percent', 'mem_bytes', 'last_error')}
            for fid, s in snapshot['features'].items()
        }
        self.pub.publish(String(data=json.dumps(
            {'timestamp': snapshot['timestamp'], 'features': features})))

    def on_command(self, msg: String):
        action, name, options = parse_command(msg.data)
        if not action or not name:
            self.get_logger().warn(
                f"Invalid command {msg.data!r}; expected JSON or 'action name [KEY=value ...]'")
            return
        try:
            self.controller.request(name, action, options)
            self.get_logger().info(f'{action} {name} {options or ""}')
        except (KeyError, ValueError, RuntimeError) as exc:
            self.get_logger().error(f'{action} {name} rejected: {exc}')
