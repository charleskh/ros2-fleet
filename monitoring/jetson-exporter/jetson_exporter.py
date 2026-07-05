#!/usr/bin/env python3
"""Prometheus exporter for Jetson tegra hardware metrics (M6 Rung 2).

The robotics-new tier of the fleet's observability: GPU load, thermal zones, and — the signal an
x86 box simply doesn't have — the **ina3221 power rails (Watts)**. node-exporter deliberately does
NOT read the tegra /sys (its hwmon/thermal collectors HANG on the Orin — see exporters.compose.yml),
so this fills that gap using jetson-stats' `jtop` client. jtop talks to the host `jtop.service` over
`/run/jtop.sock` and parses the tegra sensors correctly across JetPack versions — so we read a stable
Python API, never the fragile sysfs directly.

Exposes on :9809/metrics; caliban's Prometheus scrapes it (job 'jetson').
"""
import os

from jtop import jtop
from prometheus_client import Gauge, start_http_server

PORT = int(os.environ.get("JETSON_EXPORTER_PORT", "9809"))

# Gauges — labels (rail/zone) keep cardinality bounded and stay generic across Jetson models.
POWER = Gauge("jetson_power_watts", "Power draw per rail, from the ina3221 monitor", ["rail"])
POWER_TOTAL = Gauge("jetson_power_total_watts", "Total board power draw (VDD_IN rail)")
TEMP = Gauge("jetson_temperature_celsius", "Thermal zone temperature", ["zone"])
GPU_LOAD = Gauge("jetson_gpu_load_percent", "Integrated GPU utilization (0-100)")
GPU_FREQ = Gauge("jetson_gpu_freq_hz", "Integrated GPU current clock frequency")
RAM_USED = Gauge("jetson_ram_used_bytes", "RAM used")
RAM_TOTAL = Gauge("jetson_ram_total_bytes", "RAM total")


def update(j):
    """Copy one jtop snapshot into the Prometheus gauges. Units converted to SI base units."""
    pw = j.power
    for rail, d in pw.get("rail", {}).items():
        POWER.labels(rail=rail).set(d.get("power", 0) / 1000.0)   # mW -> W
    POWER_TOTAL.set(pw.get("tot", {}).get("power", 0) / 1000.0)   # mW -> W

    for zone, d in j.temperature.items():
        t = d.get("temp")
        # cv0-2 zones are often offline and report a sentinel; skip them so they don't pollute panels.
        if d.get("online", True) and t is not None and t > -100:
            TEMP.labels(zone=zone).set(t)                          # already °C

    for _name, g in j.gpu.items():                                # {'gpu': {...}} on the Orin iGPU
        GPU_LOAD.set(g.get("status", {}).get("load", 0))          # percent
        GPU_FREQ.set(g.get("freq", {}).get("cur", 0) * 1000)      # kHz -> Hz
        break

    ram = j.memory.get("RAM", {})
    RAM_USED.set(ram.get("used", 0) * 1024)                        # KB -> bytes
    RAM_TOTAL.set(ram.get("tot", 0) * 1024)


def main():
    start_http_server(PORT)
    # jtop() connects to /run/jtop.sock; j.ok() blocks until the next sample, so the loop self-paces
    # at the service's interval (no sleep needed) and updates the gauges continuously.
    with jtop() as j:
        while j.ok():
            update(j)


if __name__ == "__main__":
    main()
