# ros2-fleet — Hyperion sees and reports

A single-vehicle perception robot built the production way: a **ROS 2** graph that drives a CSI
camera, runs **YOLOv8 object detection at the edge on a Jetson GPU via TensorRT**, ships as **one
immutable container**, records to **MCAP** for replay, and is **observed from a ground station**
with Prometheus + Grafana.

One project that demonstrates **ROS 2 + edge inference + containerization + observability** at once —
each layer built from scratch, not black-boxed.

> **Hyperion** = NVIDIA Jetson Orin Nano (JetPack 6 / L4T r36.5, Ubuntu 22.04 → ROS 2 Humble),
> Arducam 8 MP (IMX219) CSI camera. **Caliban** = x86 ground station (Ubuntu 26) running the
> monitoring stack. A Mac is a thin Foxglove viewer.

---

## The result

The whole perception pipeline runs on the Orin's GPU and reports live to a ground-station dashboard:

| Metric | Value | Notes |
|---|---|---|
| Detector throughput | **~28 FPS** | 720p, live camera → YOLOv8n → detections |
| End-to-end latency | **~37 ms** | measured from the camera capture **timestamp**, across the ROS graph |
| Inference backend | **TensorRT FP16** | hand-built ONNX→engine; **5.8× faster** than the torch fp32 forward (28.6 → 4.9 ms) |
| Board power | **~8 W** | full graph running, read from the Jetson's INA3221 rails |
| Deploy | **one command** | `docker compose up bringup` brings the entire graph up reproducibly |
| Observability | **13-panel dashboard, provisioned as code** | FPS, latency percentiles, power, thermals, per-node liveness |

The instrumentation earned its keep immediately: the first `/metrics` scrape exposed the robot
running at **1.5 FPS / 865 ms**, which drove a **~18× throughput fix** to the numbers above
(see [Engineering highlights](#engineering-highlights)).

## Screenshots

> _Grab these from a live run — the vehicle graph up on hyperion, the monitoring stack up on
> caliban. Prometheus retains 15 days, so the dashboard renders the live window well after the run.
> Shot list + exact steps: [`docs/screenshots/SHOTLIST.md`](docs/screenshots/SHOTLIST.md)._

| | |
|---|---|
| ![Grafana — Fleet: Hyperion Perception (live robot)](docs/screenshots/grafana-dashboard.png) | ![Foxglove — camera + detections replay](docs/screenshots/foxglove-replay.png) |
| **Grafana** — the live-robot dashboard (DoD): FPS, inference-vs-e2e latency, power rails, thermals, GPU/RAM, per-node liveness. | **Foxglove** — an MCAP recording scrubbed on one timeline: camera frames and green detection boxes move together. |
| ![Kill-a-node — liveness tile flips red](docs/screenshots/kill-a-node-red.png) | |
| **Liveness demo** — kill `detector_node`; its tile flips **red** within one scrape while node/jetson exporters stay green; restarting the graph recovers it. | |

---

## Architecture

### The perception graph (on the vehicle)

Three ROS 2 nodes, wired by **one launch file** and shipped as **one container** — not one
container per node (the reasoning is in [`docs/architecture.md`](docs/architecture.md#2-the-node-graph-is-not-the-deployment-unit)):

```
                   Jetson Orin Nano (hyperion) — fleet-ros2:l4t container
  ┌──────────────────────────────────────────────────────────────────────────────────┐
  │                                                                                    │
  │  IMX219 CSI ─▶ csi_camera ──/image_raw──▶ yolo_detector ──/detections──────▶       │
  │  (Argus/ISP)  (GStreamer,      (bgr8)     (TensorRT FP16    (Detection2DArray)      │
  │               hand-packed                  on the GPU)   └──/image_annotated──▶     │
  │               sensor_msgs/Image)                            (bgr8, boxes drawn)     │
  │                                                                    │               │
  │                                              foxglove_bridge ◀──────┘               │
  └───────────────────────────────────────────────────────│──────────────────────────┘
                                                           │ ws://hyperion:8765
                                                           ▼
                                                   Foxglove (Mac) — live view + MCAP replay
```

### Observability (vehicle emits, ground station scrapes)

The prod/fleet-shaped split: the vehicle is cheap to run and only exposes `/metrics`; the ground
station owns storage and dashboards, so Prometheus + Grafana never steal from the Orin's inference
budget.

```
   hyperion (vehicle, 10.0.0.64)                  caliban (ground station, 10.0.0.29)
   └─ exporters.compose.yml                       └─ monitoring/docker-compose.yml
        detector /metrics    :9808  ◀──scrape───────  prometheus  :9090
        node-exporter        :9100  ◀──scrape──┘           │
        jetson-exporter      :9809  ◀──scrape──┘           ▼
        (FPS, latency, power, thermals)               grafana     :3000
                                                      (13 panels, provisioned as code)
```

Full write-up — the design decisions and the hard-won gotchas — is in
**[`docs/architecture.md`](docs/architecture.md)**.

---

## Run it

**Vehicle graph — on hyperion:**

```bash
docker compose up --build bringup      # camera + TensorRT detector + foxglove_bridge, one command
```

The container runs the **baked** colcon workspace and validates the Orin-specific TensorRT engine
exists at start — it never builds at runtime. View it live in Foxglove on the Mac via
`ws://hyperion:8765` (Image panel on `/image_annotated`).

**Ground station — on caliban:**

```bash
docker compose -f monitoring/docker-compose.yml up -d
#   Grafana    → http://10.0.0.29:3000   (dashboard: "Fleet — Hyperion Perception (live robot)")
#   Prometheus → http://10.0.0.29:9090   (Status → Targets for up/down)
```

**Vehicle exporters — on hyperion:**

```bash
docker compose -f monitoring/exporters.compose.yml up -d   # node-exporter :9100 + jetson-exporter :9809
```

Record & replay a run (on hyperion):

```bash
ros2_ws/record.sh          # ros2 bag record -s mcap of /image_raw /image_annotated /detections
# then open the .mcap in Foxglove and scrub the timeline
```

See [`monitoring/README.md`](monitoring/README.md) for the observability stack details.

---

## Repository layout

```
ros2-fleet/
├── Dockerfile / Dockerfile.jetson   # x86 build image / the Orin (arm64, L4T + CUDA + TensorRT) image
├── docker-compose.yml               # the `bringup` service — the whole graph as one deployable
├── bringup-entrypoint.sh            # validates the TRT engine, then `ros2 launch`es (never builds)
├── ros2_ws/
│   ├── src/
│   │   ├── csi_camera/              # CSI camera node: GStreamer → hand-packed sensor_msgs/Image
│   │   ├── yolo_detector/           # YOLOv8n node, torch OR TensorRT backend (backend:=trt)
│   │   └── bringup/                 # bringup.launch.py — the 3-node graph as one launch unit
│   ├── rung1/                       # from-scratch inference (letterbox → decode → NMS), the reference
│   ├── rung4/                       # ONNX export → trtexec engine build → TRT runtime + benchmark
│   └── record.sh                    # MCAP recording
├── monitoring/
│   ├── docker-compose.yml           # ground station: Prometheus + Grafana
│   ├── exporters.compose.yml        # vehicle: node-exporter + jetson-exporter
│   ├── jetson-exporter/             # jtop client → :9809 (GPU, thermals, INA3221 power rails)
│   ├── prometheus/                  # scrape config (host-agnostic via extra_hosts)
│   └── grafana/provisioning/        # datasource + the 13-panel dashboard, as code
└── docs/
    ├── architecture.md              # the deep-dive write-up
    └── screenshots/                 # DoD screenshots + shot list
```

---

## Engineering highlights

The hard parts — each is a place where the "professional" way differs from the tutorial way. Full
detail in [`docs/architecture.md`](docs/architecture.md).

- **Edge inference from scratch, then optimized.** Built the full YOLO pre/post pipeline by hand
  (letterbox → normalize → decode `(1,84,8400)` → NMS → un-letterbox) matching Ultralytics to
  sub-pixel, *then* swapped the torch forward for a hand-built **TensorRT FP16 engine** —
  **5.8× faster** (28.6 → 4.9 ms), boxes still matching to sub-pixel.
- **Observability that paid for itself.** End-to-end latency is measured from the camera's ROS
  **header timestamp**, not wall-clock in one process. The first scrape exposed a robot crawling at
  **1.5 FPS / 865 ms**; the metrics localized a **~580 ms rclpy quirk** (assigning `bytes` to a
  `uint8[]` message field is O(n) slow — `array.array('B', …)` is ~0.3 ms) plus a camera-mode and a
  duplicate-publish bug → **~28 FPS / ~37 ms**.
- **The GPU/camera-in-a-container grind (JetPack 6).** The CSI camera and GPU live on the host
  behind NVIDIA's proprietary stack. Injected them with **CDI** (the `runtime: nvidia` shim injects
  nothing on JP6), learned the hard way that **CDI mounts the driver tier (`libcuda`) but not the
  CUDA toolkit** — so cudart + cuDNN + CUPTI + TensorRT are baked into the image, pinned to the
  exact CUDA the driver supports.
- **Same-host DDS shared-memory trap.** Two ROS nodes in separate containers *discover* each other
  but silently *exchange no data*, because FastDDS picks shared-memory transport for same-host peers
  and each container has a private `/dev/shm`. Fix: `ipc: host`.
- **One distro, two host OSes.** hyperion is Ubuntu 22.04, caliban is 26 — ROS 2 distros don't talk
  across versions. Instead of fighting the hosts, pin **one distro (Humble) in a container** on
  both; the host just runs Docker. The distro mismatch is the *reason* to containerize, not a
  problem to solve.
- **Dashboards and scrape config as code.** No click-ops: the Grafana datasource, the 13-panel
  dashboard, and the Prometheus scrape jobs are all provisioned from this repo and hot-reload.

---

_Part of a robotics/autonomy portfolio. This is Phase 1 — a stationary vehicle that sees and
reports. Phase 2 adds a chassis (ESP32-S3 motor control + IMU over micro-ROS); the telemetry to
watch it move is already in place._
