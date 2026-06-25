#!/usr/bin/env bash
# Bring up the yolo_detector node on the TensorRT FP16 engine (the LIVE perception graph). Sources
# ROS first because a command passed to `docker compose run` is NON-interactive and does NOT read
# /root/.bashrc (same reason `ros2: command not found` bites you). Kept as a script so the long
# `--ros-args -p ...` line can't be mangled by terminal line-wrapping on paste.
#
# Run from hyperion, with the camera node already up:
#   docker compose run --rm ros2-jetson bash run_detector.sh
#
# The engine is Orin-specific + gitignored — build it in ros2_ws/rung4/ (see that README) first.
# Override the engine path with ENGINE=/path/to/x.engine docker compose run ...
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -euo pipefail   # AFTER sourcing ROS: setup.bash reads unset AMENT_TRACE_SETUP_FILES, which -u kills

ENGINE="${ENGINE:-/ros2_ws/rung4/yolov8n.fp16.engine}"

exec ros2 run yolo_detector detector_node --ros-args \
  -p backend:=trt \
  -p engine_path:="${ENGINE}"
