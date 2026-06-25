#!/usr/bin/env bash
# Record the live perception graph to an MCAP bag (M4). Sources ROS first, because a command
# passed to `docker compose run` is NON-interactive and does NOT read /root/.bashrc (this is the
# same reason `ros2: command not found` bites you). Writes UNDER /ros2_ws so the bag survives the
# --rm container (the workspace is the only host-mounted path).
#
# Run from hyperion, in a THIRD terminal, with the camera + detector already up (see RUNBOOK below):
#   docker compose run --rm ros2-jetson bash record.sh
#
# Override topics/name by env:  BAG=my_run TOPICS="/image_raw /detections" docker compose run ...
# Source ROS BEFORE `set -u`: ROS's setup.bash dereferences AMENT_TRACE_SETUP_FILES while it's
# unset, so `set -u` (nounset) aborts the source with "unbound variable". Strict flags guard OUR
# logic below, not ROS's setup scripts.
source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
set -euo pipefail

BAG="${BAG:-hyperion_run_$(date +%Y-%m-%d_%H%M%S)}"
TOPICS="${TOPICS:-/image_raw /image_annotated /detections}"
OUT="/ros2_ws/bags/${BAG}"

echo ">> recording ${TOPICS} -> ${OUT} (mcap). Ctrl-C to stop."
exec ros2 bag record -s mcap -o "${OUT}" ${TOPICS}
