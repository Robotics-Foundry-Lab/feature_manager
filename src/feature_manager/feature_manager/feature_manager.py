import os
import json
import time
import logging
import subprocess
from pathlib import Path
from typing import Dict, Any, List

import yaml
import docker
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LOG = logging.getLogger("feature_manager")
logging.basicConfig(level=logging.INFO)

# Default config path under workspace .devcontainer
WORKSPACE_ROOT = Path(os.environ.get("WORKSPACE_ROOT", Path.cwd()))
DEFAULT_CONFIG = WORKSPACE_ROOT / ".devcontainer" / "projects.yml"

def load_config(config_path: Path = None) -> Dict[str, Any]:
    if config_path is None:
        config_path = DEFAULT_CONFIG

    if not Path(config_path).exists():
        LOG.warning("Config file %s not found", config_path)
        return {}

    with open(config_path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}

def print_existing_containers(client) -> None:
    """Print existing containers and some key properties."""
    if client is None:
        LOG.warning("Docker client not available; cannot list containers")
        return

    try:
        containers = client.containers.list(all=True)
    except Exception as e:
        LOG.exception("Failed to list containers: %s", e)
        return

    if not containers:
        print("No Docker containers found.")
        return

    print("Existing Docker containers (name: status):")
    for c in containers:
        print(f"- {getattr(c, 'name', '')}: {getattr(c, 'status', '')}")


def get_project_containers(client, project_name: str) -> List[Dict[str, Any]]:
    if client is None:
        return []
    try:
        # Prefer compose label filtering when possible
        containers = client.containers.list(all=True, filters={"name": project_name})
        return [{"name": c.name, "short_id": getattr(c, "short_id", ""), "status": getattr(c, "status", "unknown")} for c in containers]
    except Exception as e:
        LOG.exception("Error listing containers for %s: %s", project_name, e)
        return []


def collect_status(config: Dict[str, Any], client) -> Dict[str, Any]:
    projects = config.get("projects") or {}
    out: Dict[str, Any] = {"projects": {}}
    for name, info in projects.items():
        # support both short form and dict form
        if isinstance(info, dict):
            display_name = info.get("display_name") or name
        else:
            display_name = name

        containers = get_project_containers(client, name)
        out["projects"][name] = {
            "display_name": display_name,
            "running": any(c.get("status") == "running" for c in containers),
        }

    out["timestamp"] = time.time()
    return out

class FeatureManagerNode(Node):
    def __init__(self, config_path: Path, interval: float = 10.0):
        if rclpy is None:
            raise RuntimeError("rclpy not available")
        super().__init__("feature_manager_node")
        self.config_path = Path(config_path)
        self.interval = interval
        self.client = docker.from_env()
        self.pub = self.create_publisher(String, "/feature_manager/status", 10)
        self.sub_cmd = self.create_subscription(String, "feature_cmd", self.on_command, 10)
        self.get_logger().info(f"FeatureManagerNode initialized, config={self.config_path}")
        self.timer = self.create_timer(self.interval, self._on_timer)

    def _on_timer(self):
        cfg = load_config(self.config_path)
        status = collect_status(cfg, self.client)
        msg = String()
        msg.data = json.dumps(status)
        self.pub.publish(msg)
        self.get_logger().info(f"Published status for {len(status.get('projects', {}))} projects")

    def on_command(self, msg: String):
        """Handle incoming feature commands on `feature_cmd` topic.

        Accepts JSON: {"action":"start"|"stop"|"toggle","name":"project_name"}
        Or plain text: "start project_name", "stop project_name", "toggle project_name"
        """
        text = msg.data.strip()
        self.get_logger().info(f"Received command: {text}")

        # try JSON first
        action = None
        name = None
        try:
            data = json.loads(text)
            action = data.get("action")
            name = data.get("name") or data.get("project")
        except Exception:
            # parse simple text
            parts = text.split()
            if len(parts) >= 2:
                action = parts[0].lower()
                name = parts[1]

        if not action or not name:
            self.get_logger().warn("Invalid command format; expected JSON or 'action name'")
            return

        cfg = load_config(self.config_path)
        projects = cfg.get("projects") or {}
        if name not in projects:
            self.get_logger().warn(f"Unknown project: {name}")
            return

        # determine folder
        info = projects.get(name)
        folder = info.get("folder") if isinstance(info, dict) else str(info)
        project_path = WORKSPACE_ROOT / folder / ".devcontainer"

        # toggle semantics
        if action == "toggle":
            containers = get_project_containers(self.client, name)
            is_running = any(c.get("status") == "running" for c in containers)
            action = "stop" if is_running else "start"

        try:
            if action == "start":
                self._run_compose(project_path, up=True)
                self.get_logger().info(f"Started project {name}")
            elif action == "stop":
                self._run_compose(project_path, up=False)
                self.get_logger().info(f"Stopped project {name}")
            else:
                self.get_logger().warn(f"Unknown action: {action}")
        except Exception as e:
            self.get_logger().error(f"Failed to run compose action: {e}")

    def _run_compose(self, project_path: Path, up: bool = True):
        if not project_path.exists():
            raise FileNotFoundError(f"{project_path} does not exist")

        if up:
            cmd = ["docker", "compose", "up", "-d"]
        else:
            cmd = ["docker", "compose", "down"]

        self.get_logger().info(f"Running: {' '.join(cmd)} in {project_path}")
        subprocess.run(cmd, cwd=project_path, check=True)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default=str(DEFAULT_CONFIG), help="Path to projects.yml")
    parser.add_argument("--interval", type=float, default=10.0, help="Publish interval in seconds")
    args = parser.parse_args()

    if rclpy is None:
        LOG.error("rclpy not available. Run inside a ROS2 environment with rclpy installed.")
        raise SystemExit(1)

    # Print existing containers at startup
    docker_client = docker.from_env()
    print_existing_containers(docker_client)

    rclpy.init()
    node = FeatureManagerNode(Path(args.config), interval=args.interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


# Create a global manager instance loaded at import time
