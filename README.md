# Fleet ROS2 (Docker)

Reproducible ROS2 **Humble** environment that runs identically on the **PC** and the
**Jetson**, so the two machines share one ROS2 graph despite running different host OSes.

## Why this exists

The Jetson's JetPack is Ubuntu 22.04 (→ native ROS2 Humble); the PC is 26.04 (→ Lyrical).
ROS2 distros don't reliably talk across versions, so instead of fighting the host OSes we
**pin one distro (Humble) in a container** on both machines. The host just runs Docker. This
is also the cleanest way to run robotics software in production — reproducible, no "works on
my machine."

## Architecture

```
   PC (Ubuntu 26.04 host)              Jetson (Ubuntu 22.04 host)
   └─ docker: fleet-ros2:humble        └─ docker: fleet-ros2:humble
        (amd64, built here)                 (arm64, built here)
            │                                   │
            └──────── same LAN, ROS_DOMAIN_ID=10, --network host ────────┘
                         DDS auto-discovery across machines
```

- Build the **same Dockerfile on each machine** — the multi-arch base image yields the
  right CPU architecture automatically (amd64 on the PC, arm64 on the Jetson).
- `network_mode: host` + matching `ROS_DOMAIN_ID` is what lets nodes on one machine
  discover topics on the other.
- Your packages live in `ros2_ws/` on the host and are mounted into the container, so code
  survives container restarts and you edit it with your normal tools.

## Usage

On **each machine**, from this directory:

```bash
docker compose build                 # build the image (per-arch, one time)
docker compose run --rm ros2         # interactive shell with ROS sourced
```

Cross-machine smoke test (proves the graph works):

```bash
# Jetson
docker compose run --rm ros2 ros2 run demo_nodes_cpp talker
# PC
docker compose run --rm ros2 ros2 run demo_nodes_py listener   # -> "I heard: Hello World"
```

Other handy commands inside the container:

```bash
ros2 topic list                      # should show the other machine's topics too
ros2 node list
```

## Building your own packages

Drop packages into `ros2_ws/src/`, then inside the container:

```bash
cd /ros2_ws
colcon build
source install/setup.bash            # (already auto-sourced in new shells)
```

## When you need more (uncomment in docker-compose.yml)

- **ESP32 / serial** — map the device: `devices: [/dev/ttyUSB0:/dev/ttyUSB0]`.
- **Jetson GPU / Isaac ROS** — add `runtime: nvidia` and `NVIDIA_VISIBLE_DEVICES=all`.

## Gotchas

- `ROS_DOMAIN_ID` must be identical on every machine (here: **10**), and stay ≤ 101.
- Both machines must be on the **same subnet** (WiFi + wired on one router is fine; guest
  networks / AP isolation will block discovery).
- `network_mode: host` works on Linux hosts (your PC + Jetson). It does *not* behave the
  same on Docker Desktop for Mac — but the Mac isn't running ROS2 nodes, so that's moot.
