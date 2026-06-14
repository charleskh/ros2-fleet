#!/usr/bin/env python3
"""Isolation test #2: pull frames via GStreamer's gi bindings (NO OpenCV).

cv2.VideoCapture(CAP_GSTREAMER) double-frees on this L4T image, so the node uses gi / GstApp
instead. This confirms the gi capture path works cleanly before we rely on it in the ROS node.

    docker compose run --rm ros2-jetson python3 test_camera.py
"""
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp  # noqa: E402

Gst.init(None)

PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv ! video/x-raw,format=BGRx ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink name=sink max-buffers=1 drop=true sync=false"
)

print("parsing pipeline (gi / GstApp, no OpenCV)...")
pipeline = Gst.parse_launch(PIPELINE)
sink = pipeline.get_by_name("sink")
pipeline.set_state(Gst.State.PLAYING)

print("pulling 5 frames...")
for i in range(5):
    sample = sink.try_pull_sample(int(2 * Gst.SECOND))
    if sample is None:
        print(f"  frame {i}: None (timeout)")
        continue
    s = sample.get_caps().get_structure(0)
    w = s.get_value("width")
    h = s.get_value("height")
    buf = sample.get_buffer()
    ok, info = buf.map(Gst.MapFlags.READ)
    size = info.size if ok else -1
    if ok:
        buf.unmap(info)
    print(f"  frame {i}: {w}x{h}, {size} bytes")

pipeline.set_state(Gst.State.NULL)
print("DONE - no crash")
