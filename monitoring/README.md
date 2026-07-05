# Fleet monitoring — ground-station observability (M6)

Prometheus + Grafana observability for the perception fleet. **The SRE differentiator** of
Phase 1: the live robot exports metrics; a ground station scrapes and graphs them.

## Topology (the M6 Rung-0 decision)

Two hosts, prod/fleet-shaped — the **vehicle only emits, the ground station scrapes**:

```
   hyperion (vehicle, 10.0.0.64)              caliban (ground station, 10.0.0.29)
   └─ exporters.compose.yml                   └─ docker-compose.yml
        node-exporter        :9100  ◀──scrape──  prometheus  :9090
        (rung2) jetson/power :9101  ◀──scrape──       │
        (rung3) cAdvisor     :8080  ◀──scrape──       ▼
                                                  grafana     :3000
```

Why not everything on the Jetson? Prometheus + Grafana would steal CPU/RAM from the Orin's
inference budget. Keeping them on caliban is the real fleet pattern: the vehicle is cheap to
run and only exposes `/metrics`; the ground station owns storage + dashboards. (Acceptable
fallback for a quick look: run both composes on one host.)

## Run it

**Ground station — on caliban:**
```bash
docker compose -f monitoring/docker-compose.yml up -d
#   Grafana    → http://10.0.0.29:3000   (admin / admin on first login)
#   Prometheus → http://10.0.0.29:9090   (Status → Targets for up/down)
```

**Vehicle exporters — on hyperion:**
```bash
docker compose -f monitoring/exporters.compose.yml up -d
curl -s localhost:9100/metrics | head    # local sanity check
```
Within one scrape (5s) the `hyperion-node` target on caliban flips **DOWN → UP**. That
red→green flip is the end-to-end proof the cross-host scrape works.

## What's provisioned as code (no click-ops)

- `prometheus/prometheus.yml` — scrape jobs. `hyperion` resolves to its LAN IP via
  `extra_hosts` in the compose file, so the scrape config stays host-agnostic.
- `grafana/provisioning/datasources/` — the Prometheus datasource is wired on Grafana
  startup; the UI can't edit it (`editable: false`). Repo is the source of truth.
- `grafana/provisioning/dashboards/` — dashboard *provider* is live; dashboard JSON lands
  here in Rung 5 and hot-reloads every 30s.

## Rung status (see `docs/phase1-hyperion-perception.md` for the full scope)

- **Rung 0 — topology + first light** ✅ Prometheus + Grafana up on caliban, datasource
  provisioned, self-scrape green, `hyperion:9100` staged.
- **Rung 1 — app metrics** — `prometheus_client` endpoint on the detector node: inference
  latency (histogram), FPS (counter), detections/frame, + **end-to-end latency from ROS
  header timestamps**.
- **Rung 2 — Jetson hardware** ✅ `jetson-exporter/` (jtop client → :9809): GPU load/freq,
  thermal zones, and the **ina3221 power rails (W)** + RAM. Runs on hyperion via jtop.service.
- **Rung 3 — liveness / containers** — cAdvisor for per-container CPU/mem; kill a node →
  panel flips red.
- **Rung 4 — logs (optional)** — Loki + Promtail → Grafana Logs panel.
- **Rung 5 — dashboard as code + DoD** — checked-in dashboard JSON: FPS, latency p50/p95,
  thermals/power, per-node up/down.
