#!/usr/bin/env bash
# M5 entrypoint for the `bringup` compose service. Two jobs, in order:
#   1. VALIDATE the TensorRT engine is present. The engine is a hardware-specific PROVISIONING
#      artifact — built once per Orin in rung4/ and mounted in, NEVER built here. Production never
#      builds in the runtime path; the container only checks the artifact exists, then runs.
#   2. LAUNCH the whole perception graph via the bringup launch file.
#
# Source ROS BEFORE `set -u`: ROS's setup.bash dereferences the unset AMENT_TRACE_SETUP_FILES, so
# nounset would abort the source with "unbound variable" (same gotcha as record.sh / run_detector.sh).
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -euo pipefail

BACKEND="${BACKEND:-trt}"
ENGINE="${ENGINE:-/ros2_ws/rung4/yolov8n.fp16.engine}"

if [ "${BACKEND}" = "trt" ] && [ ! -s "${ENGINE}" ]; then
  echo "FATAL: backend=trt but no TensorRT engine at ${ENGINE}" >&2
  echo "  The engine is Orin-specific + gitignored — build it once on hyperion (see rung4/README.md)," >&2
  echo "  it lands in ./ros2_ws/rung4/ which is mounted into this container." >&2
  echo "  Or run the non-TRT path:  BACKEND=ultralytics docker compose up bringup" >&2
  exit 1
fi

echo ">> bringup: backend=${BACKEND} engine=${ENGINE} — launching camera + detector + foxglove_bridge"
exec ros2 launch bringup bringup.launch.py backend:="${BACKEND}" engine_path:="${ENGINE}"
