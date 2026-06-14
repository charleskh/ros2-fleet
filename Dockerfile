# Plain Fleet ROS2 image — stock ROS2 Humble, NO GPU and NO Jetson multimedia stack.
# Used by the `ros2` service for non-Jetson hosts (caliban/amd64) and for any node that
# doesn't need the camera or GPU: demo nodes, telemetry, teleop, fleet tooling.
#
# >>> The Jetson (hyperion) does NOT build this file. <<<  It needs the CSI camera and the
# GPU, which this stock image can't provide — it uses Dockerfile.jetson (an NVIDIA L4T base)
# via the `ros2-jetson` service instead. Both images join the same ROS2 graph (domain 10).
#
# Build here (caliban):  docker compose build ros2
#
# Why Humble: it matches the Jetson's JetPack (Ubuntu 22.04), has the widest arm64 package
# support, and is what NVIDIA Isaac ROS targets — so the whole fleet standardizes on it.
# Containerizing it means the host OS (caliban's 26.04, the Jetson's 22.04) no longer matters.

FROM ros:humble-ros-base

# Tools we want in every container: demo nodes for smoke tests, colcon to build your
# packages, and a couple of debugging utilities.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-demo-nodes-cpp \
        ros-humble-demo-nodes-py \
        python3-colcon-common-extensions \
        iputils-ping \
        nano \
    && rm -rf /var/lib/apt/lists/*

# Your packages live here; docker-compose mounts the host ./ros2_ws over this path so
# edits persist outside the container.
WORKDIR /ros2_ws

# Source ROS (and your built workspace, if present) in every interactive shell.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /root/.bashrc

# The base image's entrypoint already sources ROS before running CMD.
CMD ["bash"]
