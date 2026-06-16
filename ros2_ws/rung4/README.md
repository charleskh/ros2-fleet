# M3 Rung 4 — optimize to TensorRT (the edge differentiator)

**Goal:** replace the torch forward pass with a **TensorRT FP16 engine** and **measure the
FPS/latency delta**. The model changes; the hand-rolled pre/post from Rung 1 does not. The measured
speedup is a portfolio artifact and pre-loads the M6 Grafana story.

**Checkpoint:** `trt_infer.py` prints boxes matching `rung1/detect_manual.py` on the same image, and
`benchmark.py` reports a torch-vs-TensorRT FPS delta.

## ⚠️ Step 0 — install TensorRT (the likely grind, like Rung 0)
TensorRT is part of the **CUDA toolkit tier** — CDI does NOT mount it (same finding as Rung 0:
`jetson-cdi-driver-not-toolkit`), so it must be baked into the image. Versions are JetPack-specific,
so **discover them on the host first**, then pin (exactly how the CUDA toolkit was pinned in Rung 0).

On the **host** (hyperion), find what JetPack provides:
```bash
python3 -c "import tensorrt as trt; print(trt.__version__)" 2>/dev/null   # host python may not have it
dpkg -l | grep -iE 'nvinfer|tensorrt'                                     # the apt packages + versions
ls /usr/src/tensorrt/bin/trtexec                                          # the builder binary
```
JetPack 6.2 / L4T r36.x ships **TensorRT 10.3**. Candidate image packages (from the SAME
`jetson/common r36.5` repo we already trust for CUDA — see `Dockerfile.jetson`):
`tensorrt`, `python3-libnvinfer`, `libnvinfer10`, `libnvinfer-plugin10`, `libnvonnxparsers10`
(+ whatever provides `trtexec`). **Pin to the host's exact versions** before adding them.

A commented `# --- M3 Rung 4: TensorRT ---` block is staged in `Dockerfile.jetson` with this note —
fill in the verified package=version list, uncomment, `docker compose build ros2-jetson`, then:
```bash
docker compose run --rm ros2-jetson python3 -c "import tensorrt as t; print(t.__version__)"
```
If `trt_infer.py` errors on the API, check this version against its CAVEAT (it targets TRT **10.x**).

## The pipeline (once TensorRT is in)
```bash
docker compose run --rm ros2-jetson bash
cd /ros2_ws/rung4
python3 export_onnx.py                                   # 1. yolov8n.pt -> yolov8n.onnx
bash    build_engine.sh                                  # 2. onnx -> yolov8n.fp16.engine (slow: autotuning)
python3 trt_infer.py  --engine yolov8n.fp16.engine --image /ros2_ws/rung1/trucks.jpg   # 3. verify boxes
python3 benchmark.py  --engine yolov8n.fp16.engine --image /ros2_ws/rung1/trucks.jpg   # 4. measure delta
```

## What each script does
- **export_onnx.py** — Ultralytics exports the model to ONNX (the last Ultralytics-managed step;
  static 640x640, opset 12). ONNX = a portable, framework-free description of the graph.
- **build_engine.sh** — `trtexec` parses the ONNX and runs the TensorRT *builder*: **layer fusion**
  (fewer kernel launches), **kernel autotuning** (benchmarks candidate CUDA kernels on THIS GPU and
  picks the fastest — why the build is slow and the engine is Orin-specific), **precision reduction**
  (`--fp16`). Output: a serialized `.engine` plan.
- **trt_infer.py** — loads the engine and runs it, **reusing rung1's letterbox/preprocess/NMS
  verbatim**. Device memory = torch CUDA tensors handed to TRT by `.data_ptr()` (no pycuda). Boxes
  should match `detect_manual.py` to within FP16 drift.
- **benchmark.py** — times the **forward pass only** (same input, sync + warmup) for torch fp32 vs
  TensorRT fp16, reporting mean/p50/p99 latency, FPS, and the speedup. **This number is the artifact.**

## Notes
- The engine is **hardware-specific** (built on the Orin, for the Orin) — don't commit it / expect
  it to load elsewhere; rebuild on each target. (Add `*.engine` + `*.onnx` to `.gitignore`.)
- FP16 introduces tiny numeric drift vs fp32 — boxes match closely, not bit-exactly (same nature as
  the Ultralytics-vs-manual drift in Rung 1).
- Optional follow-on: swap the TensorRT path into the `yolo_detector` ROS node (Rung 2) behind a
  `backend:=trt` parameter, so the live graph runs on the engine. That closes M3's DoD ("live
  detections in Foxglove, on the GPU via TensorRT, measured FPS").
