# Fleet ROS2 image — runs the same ROS2 Humble on the PC (amd64) and Jetson (arm64).
#
# Build this ON EACH MACHINE (`docker compose build`). The base image is multi-arch,
# so the PC build produces an amd64 image and the Jetson build produces an arm64 image
# automatically — no cross-compiling needed for now.
#
# Why Humble: it's the distro the Jetson's JetPack (Ubuntu 22.04) lines up with, it has
# the widest package/arm64 support, and it's what NVIDIA Isaac ROS targets. Containerizing
# it means the host OS (your 26.04 PC, the Jetson's 22.04) no longer has to match.

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
