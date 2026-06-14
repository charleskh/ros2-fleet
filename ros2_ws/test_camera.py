#!/usr/bin/env python3
"""Isolation test: does cv2 + GStreamer + nvargus capture work WITHOUT ROS in the process?

If this prints "DONE" with frame shapes, the double-free is a cv2<->rclpy interaction
(import order / threading) and the fix is cheap. If this ALSO crashes with
"double free or corruption", the problem is the cv2 + GStreamer + nvargus combo itself
(a native library conflict) and we fix it at that layer.

Run inside the L4T container (from /ros2_ws):
    docker compose run --rm ros2-jetson python3 test_camera.py
"""
import sys
import numpy as np
import cv2

print("python :", sys.version.split()[0])
print("numpy  :", np.__version__)
print("opencv :", cv2.__version__)
print("gst in cv2 build:", "GStreamer" in cv2.getBuildInformation())

PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv ! video/x-raw,format=BGRx ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=1"
)

print("opening pipeline (no rclpy in this process)...")
cap = cv2.VideoCapture(PIPELINE, cv2.CAP_GSTREAMER)
print("isOpened:", cap.isOpened())

for i in range(5):
    ok, frame = cap.read()
    print(f"  read {i}:", ok, None if frame is None else frame.shape)

cap.release()
print("DONE - no crash")
