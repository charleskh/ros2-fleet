#!/bin/sh
# Pure GStreamer — no Python, no appsink. Does the nvargus -> CPU-BGR pipeline itself work?
#
#   docker compose run --rm ros2-jetson sh test_gst.sh
#
# If this CRASHES with "double free or corruption", the problem is the GStreamer/L4T libraries
# themselves (most likely the base image's r36.3 libs vs the host's r36.5 libs that CDI mounts
# over them) -> fix is to change the base image.
# If this RUNS CLEAN ("Got EOS" / no crash), the pipeline is fine and the crash is in the
# appsink/buffer-pull layer -> fix is in how we pull frames.
set -x
gst-launch-1.0 -e \
  nvarguscamerasrc num-buffers=30 ! \
  nvvidconv ! video/x-raw,format=BGRx ! \
  videoconvert ! video/x-raw,format=BGR ! \
  fakesink
