# Screenshot shot list — the M7 portfolio artifacts

Three screenshots the README embeds. Grab them from a **live run** and drop the PNGs in this folder
with the exact filenames below. Prometheus retains **15 days**, so the Grafana shots render the live
window well after the run — but the **kill-a-node** and **Foxglove** shots are live actions.

Filenames the README expects:

| File | What it shows |
|---|---|
| `grafana-dashboard.png` | The full "Fleet — Hyperion Perception (live robot)" dashboard, healthy |
| `kill-a-node-red.png` | The liveness tile flipped **red** after killing `detector_node` |
| `foxglove-replay.png` | An MCAP recording in Foxglove: camera + green detection boxes |

Reference — a healthy live snapshot looked like this (2026-07-05, TensorRT backend):
**~28 FPS · inference p95 ~49 ms · e2e p95 ~49 ms · board ~8.6 W · max temp ~58 °C · all 4 targets green.**

---

## 1. `grafana-dashboard.png` — the DoD dashboard

**Precondition — the robot is live and being scraped:**
```bash
# on hyperion
docker compose up --build bringup                                   # the perception graph
docker compose -f monitoring/exporters.compose.yml up -d            # node + jetson exporters
# on caliban
docker compose -f monitoring/docker-compose.yml up -d               # prometheus + grafana
```

Confirm all four targets are green: **http://10.0.0.29:9090/targets** →
`prometheus`, `hyperion-node`, `jetson`, `detector` all UP.

- Open **http://10.0.0.29:3000** → dashboard **"Fleet — Hyperion Perception (live robot)"**.
  _(From the Mac, use the IP `10.0.0.29`, not the hostname.)_
- Set the time range to **Last 15 minutes**, and let a few minutes of live data accumulate so the
  timeseries panels have shape (not a flat line).
- Full-screen the browser, then capture the whole dashboard (all 13 panels). A full-page capture
  (Firefox: `Ctrl+Shift+S` → "Save full page"; or a full-page-screenshot extension) beats a viewport
  grab, since the dashboard scrolls.

## 2. `kill-a-node-red.png` — the liveness demo

The "wow" for the observability story: kill a node, watch the tile go red, restart, watch it recover.

```bash
# on hyperion, find and kill the detector process (or its container)
docker compose kill detector       # if detector is a separate service, OR:
# pkill -f detector_node           # if killing the process inside the bringup container
```

- Within one scrape interval (~5 s) the **Node liveness** tile for `detector` flips **red** while
  `hyperion-node` and `jetson` stay green.
- Capture the dashboard at that moment (the red tile + the FPS/latency panels going flat is the
  strongest single frame).
- Restart the graph (`docker compose up -d bringup` / restart the node) → the tile recovers to green.
  Optionally grab a recovery frame too.

## 3. `foxglove-replay.png` — camera + detections replay

```bash
# on hyperion, record a run of the live graph
ros2_ws/record.sh                  # writes an .mcap under ros2_ws/bags/  (Ctrl-C to stop)
```

- Copy the `.mcap` to the Mac (`scp`, via `-J caliban` if needed) and open it in **Foxglove**
  (Data source → Open local file).
- Layout: an **Image panel on `/image_annotated`** (camera frame with green boxes) — optionally a
  Raw Messages / plot panel on `/detections` beside it.
- Scrub the timeline to a frame with a clear detection (a person or object with a labeled green box)
  and capture. The point is that camera + boxes move together on **one** timeline.

---

_Once the three PNGs are in this folder, the README image links resolve and M7 is done — commit and
push `ros2-fleet`._
