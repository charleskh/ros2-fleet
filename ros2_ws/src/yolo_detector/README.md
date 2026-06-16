# yolo_detector — M3 Rung 2

**Goal:** wrap the Rung 1 inference pipeline in a live ROS2 node. Subscribe `/image_raw` from
`csi_camera`, run pretrained YOLO on the Orin GPU, publish `vision_msgs/Detection2DArray` on
`/detections`. **Checkpoint:** `ros2 topic echo /detections` shows live detections off the camera.

## Prereqs
- Rungs 0 + 1 done (torch+CUDA and a working YOLO pipeline in `fleet-ros2:l4t`).
- `ros-humble-vision-msgs` in the image — added to `Dockerfile.jetson`. **Rebuild after pulling:**
  ```bash
  docker compose build ros2-jetson
  ```

## Build the workspace (inside the container)
`colcon build` compiles both packages (`csi_camera`, `yolo_detector`) into `/ros2_ws/install`,
which `.bashrc` auto-sources in new interactive shells.
```bash
docker compose run --rm ros2-jetson bash
# inside:
cd /ros2_ws && colcon build && source install/setup.bash
```

## Run (two shells into the SAME container graph, domain 10)
```bash
# shell A — camera (publishes /image_raw):
ros2 run csi_camera camera_node

# shell B — detector (subscribes /image_raw, publishes /detections):
ros2 run yolo_detector detector_node

# shell C — watch detections (best_effort to match the sensor QoS):
ros2 topic echo /detections --qos-reliability best_effort
```

## Notes / gotchas
- **QoS must match:** the camera publishes best-effort sensor QoS; the detector subscribes with the
  same. A reliable subscriber silently receives nothing from a best-effort publisher.
- **No cv_bridge:** `sensor_msgs/Image` is unpacked into numpy by hand (`msg.step` handles row
  padding). cv_bridge → OpenCV-GStreamer → the L4T double-free, same constraint as `csi_camera`.
- **vision_msgs API:** field access targets the Humble (4.x) layout
  (`bbox.center.position.x`, `results[].hypothesis.class_id/.score`). If a build errors, confirm
  with `ros2 interface show vision_msgs/msg/Detection2D` and `.../BoundingBox2D`.
- **GPU check:** `tegrastats` (or `jtop`) — GPU% should sit busy while the camera streams.

→ Next: **Rung 3** — Foxglove overlay of `/detections` + an annotated `/image_annotated` topic.
