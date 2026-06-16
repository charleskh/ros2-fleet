#!/usr/bin/env bash
# M3 Rung 4, step 2 — build a TensorRT FP16 engine from the ONNX with trtexec.
#
# What trtexec does (the "understand the optimizer" part of this rung):
#   1. PARSE the ONNX graph.
#   2. LAYER FUSION — fold sequences (e.g. conv + bias + activation) into one fused kernel, so the
#      GPU launches fewer kernels and keeps intermediate data in registers instead of round-tripping
#      to global memory.
#   3. KERNEL AUTOTUNING — for each layer, TensorRT actually BENCHMARKS several candidate CUDA
#      kernels ON THIS GPU (the Orin) and picks the fastest. This is why the build is slow and why
#      the engine is hardware-specific.
#   4. PRECISION REDUCTION — with --fp16 the builder runs layers in half precision where accuracy
#      allows (~2x throughput + half the memory on Orin's tensor cores; I/O stays fp32 by default).
#   5. SERIALIZE the result to a .engine "plan" file (a ready-to-run, GPU-specific binary).
#
# The engine is built ON the Orin, FOR the Orin — it will NOT load on a different GPU/TensorRT.
#
#   bash /ros2_ws/rung4/build_engine.sh [yolov8n.onnx] [yolov8n.fp16.engine]
set -euo pipefail
ONNX="${1:-yolov8n.onnx}"
ENGINE="${2:-yolov8n.fp16.engine}"
# trtexec ships with TensorRT; on Jetson it lives under /usr/src/tensorrt/bin. Override with TRTEXEC=
# if the install put it elsewhere (e.g. on PATH as plain `trtexec`).
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"

if [ ! -x "$TRTEXEC" ] && ! command -v "$TRTEXEC" >/dev/null 2>&1; then
  echo "trtexec not found at '$TRTEXEC'. Is TensorRT installed in the image? (see rung4/README.md)" >&2
  exit 1
fi

"$TRTEXEC" --onnx="$ONNX" --saveEngine="$ENGINE" --fp16
echo "engine -> $ENGINE"
