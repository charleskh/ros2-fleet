#!/usr/bin/env python3
"""M3 Rung 4, step 4 — measure the FPS/latency delta: torch vs TensorRT FP16.

This delta is the portfolio artifact (and pre-loads the M6 Grafana story). To isolate what TensorRT
actually changes, we time the **forward pass only** for both — same preprocessed input tensor, no
pre/post in the timed region — so the number reflects layer fusion + kernel autotuning + fp16, not
pre/post overhead (which is identical for both and constant).

    python3 /ros2_ws/rung4/benchmark.py --engine yolov8n.fp16.engine --image /ros2_ws/rung1/trucks.jpg

NOTE: GPU calls are async — every timing loop calls torch.cuda.synchronize() before stopping the
clock, and runs warmup iterations first (the first calls pay one-time costs: cuDNN autotune for
torch, lazy kernel load for TRT). Without both, the numbers are garbage.
"""
import argparse
import os
import statistics
import sys
import time

import cv2
import torch
from ultralytics import YOLO

sys.path.insert(0, os.path.dirname(__file__))
from trt_infer import TrtYolo  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "rung1"))
from detect_manual import preprocess  # noqa: E402


def timed(fn, iters, warmup):
    for _ in range(warmup):
        fn()
    torch.cuda.synchronize()
    ms = []
    for _ in range(iters):
        t0 = time.perf_counter()
        fn()
        torch.cuda.synchronize()
        ms.append((time.perf_counter() - t0) * 1000.0)
    ms.sort()
    mean = statistics.mean(ms)
    return {
        "mean_ms": mean,
        "p50_ms": ms[len(ms) // 2],
        "p99_ms": ms[min(len(ms) - 1, int(len(ms) * 0.99))],
        "fps": 1000.0 / mean,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--engine", required=True)
    ap.add_argument("--image", required=True)
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--warmup", type=int, default=30)
    args = ap.parse_args()

    assert torch.cuda.is_available(), "CUDA not available (check Rung 0)"
    img = cv2.imread(args.image)
    assert img is not None, f"could not read {args.image}"
    x, _, _ = preprocess(img)                       # one shared (1,3,640,640) GPU input for both

    # torch baseline: the raw nn.Module forward (same tensor as TRT sees), like rung1.
    net = YOLO(args.model).model.to(0).eval()

    def torch_fwd():
        with torch.no_grad():
            net(x)

    # tensorrt forward.
    yolo = TrtYolo(args.engine)

    def trt_fwd():
        yolo.infer(x)

    print(f"benchmarking forward pass | iters={args.iters} warmup={args.warmup}\n")
    t = timed(torch_fwd, args.iters, args.warmup)
    g = timed(trt_fwd, args.iters, args.warmup)

    def row(name, r):
        print(f"  {name:<14} mean {r['mean_ms']:6.2f} ms | p50 {r['p50_ms']:6.2f} | "
              f"p99 {r['p99_ms']:6.2f} | {r['fps']:6.1f} FPS")

    row("torch fp32", t)
    row("tensorrt fp16", g)
    print(f"\n  speedup: {t['mean_ms'] / g['mean_ms']:.2f}x   (+{t['fps']:.0f} -> {g['fps']:.0f} FPS)")


if __name__ == "__main__":
    main()
