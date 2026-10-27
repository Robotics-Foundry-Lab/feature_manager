#!/bin/bash
# Container entrypoint: rebuild the workspace and start the feature manager
# (web UI on port 8080 + ROS topics /feature_manager/status and /feature_manager/cmd).
set -eo pipefail

source /opt/ros/humble/setup.bash
cd /workspace
# Rebuild on every start (a few seconds) so code changes are never stale in install/
./build.sh
source install/setup.bash

# Config and web UI are read from the source tree, so edits only need a container restart
exec python3 -m feature_manager.main \
    --config /workspace/src/feature_manager/config/features.yml \
    --web-dir /workspace/src/feature_manager/web \
    --state-file /workspace/.state/feature_manager.json \
    --port "${FEATURE_MANAGER_PORT:-8080}"
