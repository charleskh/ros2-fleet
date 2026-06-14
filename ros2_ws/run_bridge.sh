#!/usr/bin/env bash
# Launch foxglove_bridge subscribing with BEST-EFFORT QoS, so it matches the camera node's
# sensor QoS. (A reliable subscriber + a best-effort publisher are incompatible -> no data,
# which shows up as a black Image panel in Foxglove.) Serves Foxglove on ws://<host>:8765.
#
#   docker compose run --rm ros2-jetson bash run_bridge.sh
source /opt/ros/humble/setup.bash
exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p best_effort_qos_topic_whitelist:='[".*"]'
