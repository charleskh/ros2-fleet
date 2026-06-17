# yolo_detector — M3 Rung 2 + Rung 3

**Rung 2 goal:** wrap the Rung 1 inference pipeline in a live ROS2 node. Subscribe `/image_raw` from
`csi_camera`, run pretrained YOLO on the Orin GPU, publish `vision_msgs/Detection2DArray` on
`/detections`. **Checkpoint:** `ros2 topic echo /detections` shows live detections off the camera. ✅

**Rung 3 goal:** see it. The node also publishes an annotated `sensor_msgs/Image` on
`/image_annotated` (boxes + labels drawn via `r.plot()`, reusing the same inference). View it in
Foxglove on the Mac through the M2 `foxglove_bridge`. **Checkpoint:** live camera feed with bounding
boxes in Foxglove — the M3 "wow".

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

## Rung 4 — run the live graph on the TensorRT engine (closes the M3 DoD)
Build the FP16 engine once (see `../../rung4/`), then point the node at it:
```bash
ros2 run yolo_detector detector_node --ros-args \
    -p backend:=trt -p engine_path:=/ros2_ws/rung4/yolov8n.fp16.engine
```
`backend:=trt` swaps the Ultralytics forward pass for the engine (~5.8x faster) via
`yolo_detector/trt_backend.py` (the productized Rung1/Rung4 pre/post + TRT runner). The engine is
**Orin-specific** and gitignored — build it on each target. Default `backend:=ultralytics` needs no
engine and works out of the box.

## Rung 3 — view in Foxglove
The detector publishes `/image_annotated` automatically (disable with
`ros2 run yolo_detector detector_node --ros-args -p publish_annotated:=false`). To view it:
```bash
# shell D — foxglove bridge (best-effort whitelist; serves ws://hyperion:8765):
docker compose run --rm ros2-jetson bash run_bridge.sh
```
Then on the Mac: open Foxglove → **Open connection** → `ws://hyperion:8765` → add an **Image** panel
→ set its topic to `/image_annotated`. Boxes + labels are already drawn into the frame, so it works
regardless of Foxglove's `vision_msgs` overlay support. (Optional "proper" overlay: an Image panel on
`/image_raw` with `/detections` added as an annotation — finickier across Foxglove versions, which is
why `/image_annotated` is the reliable demo path.)

## Notes / gotchas
- **QoS must match:** the camera publishes best-effort sensor QoS; the detector subscribes with the
  same. A reliable subscriber silently receives nothing from a best-effort publisher.
- **No cv_bridge:** `sensor_msgs/Image` is unpacked into numpy by hand (`msg.step` handles row
  padding). cv_bridge → OpenCV-GStreamer → the L4T double-free, same constraint as `csi_camera`.
- **vision_msgs API:** field access targets the Humble (4.x) layout
  (`bbox.center.position.x`, `results[].hypothesis.class_id/.score`). If a build errors, confirm
  with `ros2 interface show vision_msgs/msg/Detection2D` and `.../BoundingBox2D`.
- **GPU check:** `tegrastats` (or `jtop`) — GPU% should sit busy while the camera streams.
- **Rebuild after editing the node:** this is an `ament_python` package, so `colcon build` copies the
  source into `install/`. Re-run `colcon build` (or use `colcon build --symlink-install` once to make
  future Python edits live without rebuilding).
- **`/image_annotated` is bgr8** like `/image_raw`; if Foxglove shows a black panel, the bridge isn't
  subscribing best-effort — confirm it was launched via `run_bridge.sh`.

→ Next: **Rung 4** — export to a TensorRT FP16 engine, swap it in for the torch forward pass, and
measure the FPS/latency delta (the edge differentiator + the M6 Grafana story).
