#!/usr/bin/env bash
# One-time WSL setup: ROS 2 Jazzy + Nav2 + ros2_control + MuJoCo bridge + Python venv.
# Re-runnable (apt and pip are idempotent).
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

sudo apt-get update -qq
sudo apt-get install -y -qq software-properties-common curl
sudo add-apt-repository -y universe

# ROS apt source (official method for Jazzy since 2025)
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | cut -d'"' -f4)
  CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")
  curl -sL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.${CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi
sudo apt-get update -qq

sudo apt-get install -y -qq \
  ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-mujoco-ros2-control ros-jazzy-mujoco-ros2-control-demos \
  ros-jazzy-xacro ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-teleop-twist-keyboard \
  python3-venv python3-pip gdal-bin

[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update -q

grep -q 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

# Python venv for offline tools (terrain, analysis, rendering).
# mujoco pinned to the version mujoco_vendor ships, so tools and sim parse MJCF identically.
# --system-site-packages so rclpy from /opt/ros stays importable.
cd "$(dirname "$0")/.."
[ -d .venv ] || python3 -m venv --system-site-packages .venv
.venv/bin/pip install -q --upgrade pip
.venv/bin/pip install -q "mujoco==3.12.0" rasterio numpy pandas matplotlib imageio imageio-ffmpeg usd-core pytest pyyaml
