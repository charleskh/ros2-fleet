# Architecture & engineering notes

How `ros2-fleet` is built, and why each layer is built the way it is. The theme throughout: at
every layer the *production* way differs from the tutorial way, and this project takes the
production way and reconstructs each piece rather than importing a black box.

The system is a single perception vehicle — a Jetson Orin Nano ("hyperion") with a CSI camera —
that runs object detection at the edge and reports its health to a ground station. Four concerns,
one artifact: **ROS 2**, **edge inference**, **containerization**, **observability**.

---

## 1. One distro in a container, not on the host

The two machines run different Ubuntu releases: hyperion is 22.04 (JetPack 6, → ROS 2 Humble),
caliban is 26 (→ a newer ROS). **ROS 2 distributions do not reliably interoperate across versions.**

Rather than fight the host OSes, the project pins **one distro — Humble — inside a container** and
runs that container on both machines. The host only has to run Docker. The same `Dockerfile` built
on each machine yields the right architecture automatically (arm64 on the Jetson, amd64 on the PC)
because the ROS base image is multi-arch.

The distro mismatch isn't a problem to be solved — it's the *reason to containerize*. It's also the
cleanest way to run robotics software in production: reproducible, immutable, no "works on my
machine." Every subsequent decision (bake the workspace, treat the TensorRT engine as a provisioning
artifact) follows from taking containerization seriously.

---

## 2. The node graph is *not* the deployment unit

The single most important orchestration idea in the project, and the classic beginner mistake to
avoid: **there are two independent orchestration layers, and collapsing them is wrong.**

1. **The deployment unit** — what gets built, versioned, shipped, restarted. Owned by
   Docker / systemd / a fleet layer.
2. **The node graph** — how a set of tightly-coupled ROS nodes share params, QoS, lifecycle, and
   *memory*. Owned by **ROS 2 launch**.

**Container boundary ≠ node boundary.** This project's perception graph is three nodes —

```
csi_camera ──/image_raw──▶ yolo_detector ──/detections──────▶ (future: tracker / planner)
                                        └──/image_annotated──▶ foxglove_bridge ──▶ Foxglove
```

— wired by a single `bringup.launch.py` and shipped as **one container**, not one container per
node. Why it matters here specifically: `/image_raw` is a hot path (720p frames every ~33 ms).
Putting each node in its own Docker container *forces* each into its own process, which *forbids*
ROS 2 **intra-process (zero-copy) communication** — you'd pay a serialize → copy → deserialize tax
on every frame. Keeping the graph in one deployment unit, orchestrated by launch, keeps that door
open and lets the nodes share params/QoS/lifecycle.

**Build discipline that pairs with this:** the `colcon` workspace is baked into the image as the
last layer (a `.dockerignore` keeps host build artifacts out of the context). The Orin-specific
TensorRT engine is treated as an explicit provisioning artifact — the entrypoint *validates it
exists*, then `ros2 launch`es. It never builds at runtime. The container is immutable; nothing
important happens on first boot.

---

## 3. Edge inference: from scratch, then optimized

The approach was deliberately the hard way: build every stage of the perception pipeline by hand
for deep understanding, using a stock pretrained COCO model (YOLOv8n) — no custom training.

**Stage 1 — reconstruct the pipeline (`ros2_ws/rung1/`).** A hand-rolled path that matches
Ultralytics to 1–3 px: letterbox → BGR→RGB → `/255` → HWC→CHW → forward `(1,84,8400)` → decode
`xywh` + max-class-score (YOLOv8/v11 have no objectness term) → confidence threshold → NMS →
un-letterbox. The residual drift is class-agnostic vs. per-class NMS. This hand-written pre/post is
the reusable artifact — it's reused verbatim when the backend changes.

**Stage 2 — a ROS node (`yolo_detector`).** Subscribes `/image_raw` with best-effort sensor QoS,
hand-unpacks `sensor_msgs/Image` (step-aware, no `cv_bridge` — see §5 for why), runs YOLOv8n on the
GPU, and publishes `vision_msgs/Detection2DArray` on `/detections` plus an annotated `/image_annotated`
for Foxglove (same inference, no second pass).

**Stage 3 — optimize to TensorRT (`ros2_ws/rung4/`).** Ultralytics → ONNX (the last managed step) →
`trtexec --fp16` → a `.engine` → a TensorRT 10.x runtime that reuses the Stage-1 letterbox/NMS
verbatim, with device memory backed by torch CUDA tensors via `.data_ptr()` (no pycuda). Result:

| Backend | Forward pass | Throughput |
|---|---|---|
| torch fp32 | 28.6 ms | 34.9 FPS |
| **TensorRT fp16** | **4.9 ms** | **202.7 FPS** |

**5.8× faster**, boxes still matching the hand-rolled reference to sub-pixel (a single 0.89-vs-0.88
confidence delta from FP16 rounding). The live graph runs the engine via a `backend:=trt` switch on
the node — the engine is what ships, not torch.

---

## 4. Getting the camera and GPU into a container (JetPack 6)

The core problem: **a container is an isolated userspace, but the camera and GPU live on the host.**
The IMX219 is driven by NVIDIA's proprietary stack (Argus + ISP, CUDA, EGL, the `nv*` GStreamer
plugins) plus a kernel driver — all host-side. To use them in a container you must inject the host's
device nodes *and* its driver libraries, version-matched to the running kernel.

**CDI, not `runtime: nvidia`.** On JetPack 6 / L4T r36.5, NVIDIA removed the apt package whose CSV
file told the `runtime: nvidia` hook what to mount — so that shim now injects *nothing* (the
container looks GPU-enabled but is blind). The supported path is **CDI (Container Device
Interface)**: `nvidia-ctk cdi generate` introspects the board once and writes a declarative YAML
manifest of every device node, bind-mount, and env var; compose then requests `nvidia.com/gpu=all`.
Mental model: `runtime: nvidia` is "trust this magic hook (whose config no longer exists)"; CDI is
"here is an explicit manifest of exactly what to mount."

**The two-tier catch — the thing that costs a day.** NVIDIA's stack is two tiers:

- **Driver tier** (`libcuda.so`, Argus, EGL, `nvvidconv`, the `nv*` plugins) — versioned to the
  L4T release, **mounted by CDI** from the host. *Do not bake it* (baking r36.3 libs under the r36.5
  host overlay double-frees `nvbufsurftransform`). Carry zero NVIDIA driver libs.
- **Toolkit tier** (`libcudart`, cuBLAS, cuDNN, **CUPTI**, NVTX, **TensorRT**) — versioned to CUDA
  12.6, and **CDI does *not* mount it** (`find / -name libcudart.so*` comes up empty in a fresh CDI
  container, while `libcuda.so` is present). Any GPU framework needs the toolkit, so it's **baked
  into the image** — which is safe precisely because CDI never overlays it.

So `Dockerfile.jetson` bakes the CUDA 12.6 toolkit (cudart + cuDNN + CUPTI + NVTX) and TensorRT,
each pinned to the exact version the host driver supports, and installs a torch wheel built for that
exact CUDA (`cu126`, **not** `cu130` — a CUDA-13 wheel silently substituted via an `--extra-index-url`
fallback will import but report `CUDA available: False`). A few more one-liners were needed along the
way (generic GStreamer plugins for `appsink`/`videoconvert`; an EGL ICD env override so glvnd finds
NVIDIA's tegra-path vendor JSON; pinning `numpy<2` because the torch wheel is built against the
numpy-1.x C-ABI and Ultralytics pulls numpy 2.x → `torch.from_numpy()` aborts "Numpy is not
available").

**One more camera subtlety:** the IMX219's 720p mode is 60-fps-*native* (lower resolution → pixel
binning → higher readout rate). Pinning `framerate=30/1` makes Argus look for a 720p@30 mode that
doesn't exist → caps negotiation fails → **zero frames**. Fix: don't pin the sensor rate; let it run
native 60 and throttle *downstream* (the publish timer fires at the target rate, `appsink
max-buffers=1 drop=true` keeps only the freshest frame). Decouple "what the sensor runs at" from
"what you publish."

---

## 5. The same-host DDS shared-memory trap

The finisher of getting the graph running: a published image topic was **visible to a subscriber in
the same container but invisible to one in a sibling container on the same host** — yet it had worked
fine across two *different* machines. That asymmetry is the whole tell.

ROS 2 doesn't move data; **DDS** does (here FastDDS), and DDS separates two concerns that
succeed/fail independently:

- **Discovery** — participants finding each other, small metadata over UDP multicast.
- **Data delivery** — the actual samples, over a chosen *transport*.

For two participants it detects on the **same host**, FastDDS prefers the **shared-memory (SHM)**
transport — a memcpy through a `/dev/shm` segment both processes `mmap`, far faster than the network
stack for image-sized payloads. But Docker gives each container its **own IPC namespace → its own
private `/dev/shm`**. With `network_mode: host`, UDP discovery works (they *find* each other and
agree "same host → use SHM"), but each writes SHM segments the other can't open → **data silently
vanishes**. Across two machines it never tries SHM, falls back to UDP, and works — which is exactly
what disguises the bug.

**Fix:** `ipc: host` on the compose services, so containers share the host's `/dev/shm`.

> One sentence: with `network_mode: host`, ROS 2 nodes in separate containers discover each other
> but can't exchange data, because FastDDS picks shared-memory transport for same-host peers and each
> container has a private `/dev/shm` — fix with `ipc: host`.

(A related trap on the same topic: image topics use **best-effort** sensor QoS, so a reliable
subscriber sees nothing even when delivery works. And this project hand-packs `sensor_msgs/Image`
instead of using `cv_bridge`/OpenCV, which double-free on this L4T image handling the NVMM/Argus
buffers.)

---

## 6. Observability that earns its keep

This is the SRE layer, and it justified itself on day one.

**Latency is measured across the graph, not in one process.** The detector exposes a
`prometheus_client` endpoint (`start_http_server`) with an inference-latency histogram (the model
forward), a frames counter (→ FPS via `rate()`), detections-per-frame, and — the robotics-specific
piece — **end-to-end latency computed as `now − Image.header.stamp`** when detections publish. That
measures real time across the camera→detector hop using the message timestamp, not wall-clock inside
a single node.

**What it caught immediately.** The first scrape showed the whole robot running at **1.5 FPS /
865 ms end-to-end** — invisible without instrumentation. The metrics localized three bugs:

1. The dominant one: assigning `bytes` to the `uint8[]` `Image.data` field goes through a
   ~580 ms-per-frame slow path in rclpy for a 720p image; `array.array('B', …)` does the same
   assignment in ~0.3 ms.
2. The camera framerate mis-pin from §4 (starving the graph of frames).
3. A latent duplicate publish of the annotated image.

Fixing them took the robot to **~28 FPS / ~37 ms** — roughly an **18× throughput** improvement,
found and verified through the dashboard rather than guessed at.

**Jetson hardware metrics** come from a small `jetson-exporter` (`:9809`) that reads **jetson-stats'
jtop** over the host's `jtop.service` socket — GPU load/frequency, thermal zones, RAM, and the
robotics-flavored signal an x86 box lacks: **per-rail power draw in watts from the INA3221**. It
deliberately does *not* read raw tegra `/sys`, which hangs a stock node-exporter.

**Per-node liveness** falls out of Prometheus' own `up{job=…}` series — one target per node. The
demo: kill `detector_node` and its dashboard tile flips **red** within one scrape interval while the
node/jetson exporters stay green; restarting the graph recovers it.

---

## 7. Fleet-shaped monitoring topology

The observability stack is split across the two machines on purpose:

```
   hyperion (vehicle)                          caliban (ground station)
   └─ exporters.compose.yml                    └─ monitoring/docker-compose.yml
        detector /metrics :9808  ◀──scrape───────  prometheus :9090
        node-exporter     :9100  ◀──scrape──┘           │
        jetson-exporter   :9809  ◀──scrape──┘           ▼
                                                    grafana    :3000
```

The **vehicle only emits** `/metrics`; the **ground station scrapes, stores, and graphs**. Running
Prometheus + Grafana on the Jetson would steal CPU/RAM from the Orin's inference budget; keeping them
on the ground station is the real fleet pattern and keeps the vehicle cheap to run. Everything is
**provisioned as code** — the Grafana datasource, the 13-panel dashboard, and the Prometheus scrape
jobs all live in this repo and hot-reload; the scrape config stays host-agnostic (`hyperion` resolves
via `extra_hosts`). No click-ops, so the repo is the source of truth.

> Per-container metrics (cAdvisor) were intentionally dropped — it's incompatible with the vehicle's
> Docker 29 overlayfs/containerd-snapshotter store (it hard-codes an overlay2 layerdb lookup that
> doesn't exist there). Host CPU/RAM is covered by node-exporter; per-container is deferred, not in
> the definition of done.

---

## Hardware & software

| | |
|---|---|
| **Vehicle (hyperion)** | Jetson Orin Nano dev kit, JetPack 6 / L4T r36.5, Ubuntu 22.04 |
| **Camera** | Arducam 8 MP IMX219 (CSI, driven via `nvarguscamerasrc`) |
| **Ground station (caliban)** | x86 workstation, Ubuntu 26 |
| **Viewer** | Mac running Foxglove |
| **ROS** | ROS 2 Humble (containerized on both machines) |
| **Inference** | YOLOv8n COCO, TensorRT 10.3 FP16 engine on the Orin GPU |
| **GPU stack** | CUDA 12.6 toolkit baked; driver tier via CDI |
| **Record/replay** | `ros2 bag record -s mcap`, replayed in Foxglove |
| **Observability** | Prometheus + Grafana + node-exporter + a custom jtop exporter |
