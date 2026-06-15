# M3 Rung 1 — YOLO inference in isolation (no ROS)

**Goal:** run a pretrained COCO YOLO model on ONE saved image, on the Orin GPU, and understand
every stage. **Checkpoint:** it prints class/score/box AND you can explain pre/forward/post.

Two scripts, same result:
- `detect_ultralytics.py` — the working version (`model(img)` does everything). The checkpoint.
- `detect_manual.py` — reconstructs each step (letterbox, normalize, raw forward, decode, NMS) so
  you can explain it. Its pre/post is reused in Rung 4 when torch is swapped for a TensorRT engine.

## Prereqs
- Rung 0 done (torch 2.8.0 + CUDA in `fleet-ros2:l4t`).
- `ultralytics` in the image — added to `Dockerfile.jetson` (Rung 1 layer). **Rebuild + verify**
  (first task tomorrow — the build hasn't been run yet):
  ```bash
  docker compose build ros2-jetson
  # guard: confirm ultralytics didn't drag in a CPU torch
  docker compose run --rm ros2-jetson python3 -c "import torch; print(torch.cuda.is_available())"
  ```
  Must still print `True`. If it prints `False`, ultralytics pulled a non-GPU torch — pin
  `ultralytics==<ver>` and reinstall torch from the jp6/cu126 index after it.

## Get a test image
Drop any `.jpg` here as `sample.jpg`, or grab a frame from the CSI camera on hyperion (host) using
the M2 GStreamer pipeline (see `../test_gst.sh`) and save it to `ros2_ws/rung1/sample.jpg`. Using a
real camera frame is more satisfying than a stock photo.

## Run
```bash
# working (Ultralytics-first):
docker compose run --rm ros2-jetson \
    python3 /ros2_ws/rung1/detect_ultralytics.py --image /ros2_ws/rung1/sample.jpg

# learning (manual pre/post — boxes should match the above):
docker compose run --rm ros2-jetson \
    python3 /ros2_ws/rung1/detect_manual.py --image /ros2_ws/rung1/sample.jpg
```

## Verify GPU, not CPU
In a second terminal on hyperion: `tegrastats` (or `jtop`) — `GR3D_FREQ` / GPU% should spike during
inference. (jtop install: `sudo pip3 install -U jetson-stats` on the host, then re-login.)

## The pipeline (what detect_manual.py reconstructs)
1. letterbox to 640×640 (aspect-preserving, gray pad)  2. BGR→RGB  3. /255 normalize
4. HWC→CHW + batch → (1,3,640,640) GPU tensor  5. forward → (1,84,8400) grid
6. decode xywh + max-class-score (no objectness in v8/v11)  7. confidence threshold
8. xywh→xyxy  9. NMS  10. undo letterbox → original-image pixels

→ Next: **Rung 2** wraps this in a ROS2 node `yolo_detector` publishing `vision_msgs/Detection2DArray`.
