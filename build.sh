#!/usr/bin/env bash
# Build the workspace. --merge-install puts every package's share/ side by side, which the
# world MJCF files rely on to <include> the rover by relative path (see DEVLOG, Phase 1).
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --symlink-install "$@"
