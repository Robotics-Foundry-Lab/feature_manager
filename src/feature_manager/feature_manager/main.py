"""Entry point: feature controller + web server + (optional) ROS node in one process."""

import argparse
import logging
import os
from pathlib import Path
import threading

import uvicorn

from .config import load_config
from .controller import FeatureController
from .web import create_app

LOG = logging.getLogger('feature_manager')


def _share_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('feature_manager'))
    except Exception:  # noqa: B902 - not installed via colcon, use the source tree
        return Path(__file__).resolve().parent.parent


def _start_ros(controller: FeatureController):
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
    except ImportError:
        LOG.warning('rclpy not available, running without the ROS interface')
        return None

    from .ros_node import FeatureManagerNode
    try:
        # uvicorn owns SIGINT/SIGTERM; rclpy must not install its own handlers
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:
        rclpy.init()
    node = FeatureManagerNode(controller)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, name='ros', daemon=True).start()
    return rclpy, node


def main(argv=None):
    share = _share_dir()
    parser = argparse.ArgumentParser(
        description='Monitor and control the robot feature containers')
    parser.add_argument('--config', type=Path, default=share / 'config' / 'features.yml',
                        help='Path to features.yml')
    parser.add_argument('--web-dir', type=Path, default=share / 'web',
                        help='Directory with the web UI')
    state_home = Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local' / 'state'))
    parser.add_argument('--state-file', type=Path,
                        default=state_home / 'feature_manager' / 'state.json',
                        help='Where the last handled boot id is stored')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--no-autostart', action='store_true',
                        help='Do not start the autostart features on a new boot')
    parser.add_argument('--no-ros', action='store_true', help='Run without the ROS interface')
    args, _ros_args = parser.parse_known_args(argv)

    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(name)s: %(message)s')

    config = load_config(args.config)
    LOG.info('Loaded %d features from %s (workspace %s)', len(config.features), args.config,
             config.workspace_root)
    controller = FeatureController(config, args.state_file)
    controller.start_background()
    if not args.no_autostart:
        controller.autostart_once()

    ros = None if args.no_ros else _start_ros(controller)
    nav = ros[1].nav if ros else None

    LOG.info('Web UI on http://%s:%d', args.host, args.port)
    try:
        uvicorn.run(create_app(controller, args.web_dir, nav), host=args.host, port=args.port,
                    log_level='warning')
    finally:
        controller.shutdown()
        if ros:
            rclpy, node = ros
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
