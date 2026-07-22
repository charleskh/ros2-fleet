# ros2-fleet — Code Review / Interview-Defense Guide

**Self-contained study guide for reviewing this repo OFFLINE** (e.g. on a plane, no Claude).
Goal: be able to **defend every part of this codebase in your own words** in an interview. Walk it
in **data-flow order** — that's also the exact order you'd narrate it live ("a frame comes in the
camera, and…"), so the review doubles as rehearsal.

> How to use each module: (1) **open the file cold** and try to explain it out loud *before* reading
> the notes; (2) answer the **self-quiz** from memory, then check yourself; (3) if you miss one, that's
> the reflex to drill. The state tracker (what's covered, quiz scores) lives in the datasmith repo at
> `docs/review-to-mastery-progress.md`.

---

## The review map

| # | Module | Files | The story / what you'll be quizzed on |
|---|---|---|---|
| 1 | **Camera in** | `ros2_ws/src/csi_camera/csi_camera/camera_node.py`, `ros2_ws/test_gst.sh` | GStreamer CSI pipeline; the `uint8[]` fast-assign trick (580ms→0.3ms) |
| 2 | **Perception (the heart)** | `ros2_ws/src/yolo_detector/yolo_detector/detector_node.py`, `trt_backend.py`, `ros2_ws/rung1/`, `ros2_ws/rung4/` | from-scratch → Ultralytics → TensorRT 5.8×; ONNX export, engine build, benchmark |
| 3 | **Composition / bringup** | `ros2_ws/src/bringup/launch/bringup.launch.py`, `Dockerfile.jetson`, `docker-compose.yml` | "one `compose up` brings up the graph"; `ipc:host` zero-copy SHM; CDI driver-vs-toolkit |
| 4 | **Observability** | `monitoring/jetson-exporter/jetson_exporter.py`, `monitoring/prometheus/prometheus.yml`, detector `/metrics` | the instrumentation that drove the ~16–18× perf fix; power rails via ina3221 |
| 5 | **Dashboard-as-code (the DoD)** | `monitoring/grafana/provisioning/dashboards/fleet-overview.json` | 13 panels auto-provisioned; `up{job=…}` liveness / kill-a-node |
| 6 | **Bridge** | `ros2_ws/run_bridge.sh` | microROS ESP32 motor bridge |
| 7 | **The write-up** | `README.md`, `docs/architecture.md` | reviewed **last** — verify every claim against the code you just walked |

---

## Vocabulary cheat-sheet (nail these first — the whole defense stands on them)

- **`ws` = colcon workspace.** Just a directory whose `src/` holds packages. Started empty; each package
  scaffolded by `ros2 pkg create --build-type ament_python <name>` (a generator, *not* a cloned template).
- **colcon** = the **orchestrator**: discovers packages in `src/`, topo-sorts by `package.xml` deps,
  delegates the actual build. ≈ a monorepo build driver (Bazel/`make -C`).
- **ament** = the **per-package build system** colcon delegates to — `ament_python` (→ setuptools, used by
  all 3 packages here) or `ament_cmake` (→ CMake, for C++). Also owns the install layout + the ament index
  (runtime package discovery).
- **package** = unit of **build / dependency / reuse** (has a `package.xml`). **node** = a **participant in
  the ROS graph** — a process that does one job, talking over **topics** (pub/sub), **services** (RPC),
  **actions** (long-running goals). *Don't call a node "a service"* — service is one specific primitive.
- **package granularity**: draw boundaries where build/dep/reuse align. Camera vs mic vs temp have disjoint
  deps → separate packages. (This repo already keeps the camera OUT of the detector package. ✓)
- **two abstraction seams**: the **message contract** (`sensor_msgs/Image` on a topic) decouples
  *consumers*; a **shared base class** decouples *producers* (DRY across drivers). **YAGNI:** the standard
  topic gives the swap-point for free — don't factor a base class until a 2nd real driver exists.

---

## Module 1 — Camera in (`camera_node.py`)  ✅ *covered together*

### The universal node skeleton (true of EVERY ros2 node)
Read `main()` at the bottom first:
```
rclpy.init()            # bootstrap the ROS client library / context
node = CsiCameraNode()  # construct the Node subclass (registers it in the graph)
rclpy.spin(node)        # hand control to the EXECUTOR — the run-loop that fires callbacks
node.destroy_node()     # graceful teardown
rclpy.shutdown()        # tear down the context
```
It's a **reactor / event-loop service** (≈ Go daemon, Node.js). There's **no `while True`** in the node —
`create_timer(1/fps, tick)` registers a callback and the executor inside `spin()` fires it. `tick` is the
handler that runs ~30×/sec.

- **`class CsiCameraNode(Node)` + `super().__init__("csi_camera")`** — subclass `Node`; the string is the
  node's name in the graph. Package = `csi_camera`, node = `csi_camera`.
- **`create_publisher(Image, topic, qos)`** — declares the node's output edge.
- **`create_timer(period, cb)`** — the engine. **period = 1 / frequency**, so 30 Hz → `1.0/30 = 0.0333s`.
  (Read `1/fps` as "one-thirtieth of a second," NOT "one frame per second.")
- **Parameters** (`declare_parameter`) = externalized 12-factor config, defaulted + overridable at
  launch/CLI. `frame_id` and `topic` are params too → same code, many cameras. This IS "parameterize the
  interface-specific bits."

### The data path (`tick`)
pull appsink sample → read w/h from the sample's **caps** → **map** the buffer, **copy** bytes out,
**unmap** → build `Image` (header **stamp** = when captured, **frame_id** = which sensor; `encoding=bgr8`;
`step` = stride) → **publish**. The header's stamp+frame_id is the foundation of sensor fusion ("when &
from where was this seen?").

### Definitions
- **dims** = width × height in pixels.
- **stride / `step`** = bytes from one pixel row to the next *in memory*. Not always `width×3` — hardware
  **pads** rows to an alignment boundary. `step = size // h` recovers the true stride so a consumer reading
  row-by-row doesn't skew. (≈ padded struct size vs logical field size.)
- **double-free** = freeing the same heap block twice → allocator corruption / abort. OpenCV 4.8's
  GStreamer capture double-frees the NVMM buffer on this image → that's why the node avoids it.
- **L4T** = **Linux for Tegra**: NVIDIA's Ubuntu-based BSP for Jetson. Ships nvargus (camera ISP daemon),
  NVMM (GPU buffers), nvvidconv. The double-free is specific to how OpenCV was built against *this* L4T.

### The 5 war stories (the defensible "why not the obvious thing?" decisions)
1. **`array.array('B')` not `bytes` for `msg.data`** — assigning `bytes` makes rclpy validate the `uint8[]`
   element-by-element across the Python↔C++ bridge: **~580 ms** for a 2.76 MB frame (caps the node ~1.7
   fps). `array.array('B')` hits the **memcpy fast path**: **~0.3 ms**. Instrumentation surfaced it; the
   node went ~1.7 fps → full rate. **Your single strongest story.** *(Verify the exact end-to-end
   multiplier — 16× vs 18× — against `architecture.md` in module 7.)*
2. **best-effort, keep-last QoS** on multi-MB images — reliable QoS → retransmit storms that collapse a
   LAN. Want *the latest frame*, not *every frame*. **Gotcha:** subscribers must match or they see nothing.
3. **`sensor_framerate=60`, not 30** — the 720p IMX219 mode is **60-fps-only**; `30/1` matched no sensor
   mode → zero frames. Fix = the mode's real rate (60), then the ROS timer + `drop=true` downsample to the
   output rate.
4. **GStreamer bindings, not `cv2.VideoCapture`** — the L4T double-free (see above). Node packs `Image` by
   hand, no OpenCV in the process.
5. **appsink `max-buffers=1 drop=true sync=false`** — always the newest frame, low latency, deliver as fast
   as captured.

### Self-quiz (answer from memory, then check)
1. Where's the loop that keeps grabbing frames? → *There isn't one; the **executor** inside `rclpy.spin()`
   fires the timer callback.*
2. `ros2 topic echo /image_raw` shows nothing though the topic exists and the node publishes. Why + fix? →
   *QoS mismatch: publisher best-effort, echo defaults reliable → incompatible, no connection. Fix:
   `--qos-reliability best_effort`.*
3. `try_pull_sample` can block up to 0.5s; the timer wants to fire every 33ms. What happens to the other
   callbacks while a tick blocks? → *Default executor is **single-threaded** → the whole node freezes for
   that 0.5s; callbacks don't overlap. A node does one thing at a time. Fixes: `MultiThreadedExecutor` +
   callback groups, or keep callbacks non-blocking.*

### Make reflexive before the defense
- "What happens if a callback blocks?" → single-threaded executor, one thing at a time.
- period = 1 / frequency (the `1/fps` thing) — no hesitation.

---

## Module 2 — Perception (`detector_node.py`, `trt_backend.py`, `rung1/`, `rung4/`)  ⬜ *self-study*

*Not yet covered together — read cold and answer these; we'll close gaps next session.*
- **Contrast with the camera:** this is a **consumer**. Find the **subscription** — what topic, what QoS?
  (Cross-check against camera story #2 — do they match?) Why a subscription callback here instead of a
  timer? (message-driven vs clock-driven)
- Revisit the single-threaded-executor lesson from the *consumer* side: if TensorRT inference in the
  callback is slow, what happens to incoming frames? What QoS/queue behavior saves you?
- `rung1/` = the "YOLO on one image, from scratch + Ultralytics" scaffold; `rung4/` = the TensorRT
  export/build/infer/benchmark path. Trace: **PyTorch/Ultralytics → ONNX export → TensorRT engine build →
  inference**. Why FP16? Why does an engine build tie you to *this* GPU/TensorRT version?
- The headline claim is **TensorRT ~5.8× vs the baseline** — find where that number comes from and be ready
  to say what the baseline was.
- How does a raw model output (tensor of boxes) become a ROS `Detection2DArray`? Where's NMS / the
  confidence threshold?

## Module 3 — Composition / bringup (`bringup.launch.py`, `Dockerfile.jetson`, `docker-compose.yml`)  ⬜
- What does a **launch file** actually do vs running nodes by hand? What gets parameterized here?
- **"one `compose up` brings up the graph"** — trace it: compose service → entrypoint → launch → nodes.
- **`ipc:host`** — why? (same-host containers need it or FastDDS shared-memory transport silently drops
  data, esp. images.) This is a zero-copy SHM story — be ready to explain shared memory vs loopback.
- **`Dockerfile.jetson` — CDI driver-vs-toolkit two-tier**: CDI mounts the *driver* (libcuda) but NOT the
  CUDA *toolkit* → you must bake cudart/cuDNN/cupti into the image to run torch/TensorRT. Why is that split
  the way it is?

## Module 4 — Observability (`jetson_exporter.py`, `prometheus.yml`, detector `/metrics`)  ⬜
- What's a Prometheus **exporter**? Pull vs push model — who scrapes whom?
- `jetson_exporter.py` reads **jtop** → exposes GPU/thermals + **ina3221 power rails (W)** + RAM. What's
  ina3221, and why report *watts*?
- The detector exposes **`/metrics`** — what does it measure (FPS, inference p95, e2e p95)? This is the
  instrumentation that **surfaced the perf fix**. Be ready: what did you measure, what did it reveal?
- Watch the node-exporter gotcha: default collectors hang on Jetson `/sys` → disable-defaults + whitelist.

## Module 5 — Dashboard-as-code / the DoD (`fleet-overview.json`)  ⬜
- Why is a **checked-in JSON dashboard** ("dashboard-as-code") the Definition of Done, vs clicking in the
  Grafana UI? (reproducibility, provisioning, review)
- **`up{job=…}`** liveness + the **kill-a-node demo** (tile goes red, restart recovers). What is `up`, and
  why use it instead of cAdvisor? (cAdvisor was dropped — Docker 29 overlayfs/snapshotter incompat.)

## Module 6 — Bridge (`run_bridge.sh`)  ⬜
- **microROS ESP32 motor bridge** — what is microROS (ROS on a microcontroller) and what's the bridge
  bridging (DDS agent ↔ ESP32 serial/UDP)? What best-effort QoS is set and why?

## Module 7 — The write-up (`README.md`, `docs/architecture.md`)  ⬜ *review LAST*
- **These were drafted by Claude → highest defense risk.** Read every claim and check it against the code
  you just walked. Flag anything you can't source.
- Known claims to verify: the perf multiplier (**16× vs 18×** — reconcile the number), **TensorRT 5.8×**,
  **~28 FPS / inference & e2e p95 ~49ms / 8.6W**, **13 Grafana panels**, **4 Prometheus targets green**,
  the `ipc:host` SHM and CDI two-tier explanations.
- For each architecture claim, be able to point at the exact file/line that backs it.

---

## The interview narration (glue it into one story)
"A frame comes off the **CSI camera** (module 1) — pulled via GStreamer, packed into a `sensor_msgs/Image`
by hand for speed, published best-effort. The **YOLO detector** (2) subscribes, runs TensorRT inference,
publishes detections. The whole graph stands up with **one `compose up`** (3), containers sharing memory via
`ipc:host`. Everything is **instrumented** (4) — Prometheus scrapes the Jetson's power/thermals and the
detector's latency — and it all lands on a **checked-in Grafana dashboard** (5) with kill-a-node liveness.
An **ESP32 bridge** (6) closes the loop to motors. The **write-up** (7) tells that story end-to-end."
