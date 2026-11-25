# Mars Yard: ROS 2 Jazzy + Nav2 + ros2_control + MuJoCo bridge, workspace built.
#   docker build -t mars-yard .
#   docker run --rm -it mars-yard                     # shell with the workspace sourced
#   (the simulation checks in tools/ are not run in CI; untested inside the container)
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
# Same apt set as tools/setup_wsl.sh (minus interactive teleop).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      ros-jazzy-robot-localization \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-mujoco-ros2-control \
      ros-jazzy-xacro ros-jazzy-rosbag2-storage-mcap \
      python3-venv python3-pip bc \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws

# Offline tools and tests run in a venv that still sees rclpy (--system-site-packages);
# mujoco is pinned to the version the bridge ships so both parse MJCF identically.
RUN python3 -m venv --system-site-packages .venv \
    && .venv/bin/pip install -q --no-cache-dir "mujoco==3.12.0" rasterio numpy pandas \
         "matplotlib>=3.8" pytest pyyaml pillow

RUN ./build.sh --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash' >> /root/.bashrc
SHELL ["/bin/bash", "-c"]
CMD ["bash"]
