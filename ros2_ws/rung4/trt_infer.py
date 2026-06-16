#!/usr/bin/env python3
"""M3 Rung 4, step 3 — run inference with the TensorRT engine (no torch forward pass).

The MODEL changes (a TensorRT engine instead of the torch nn.Module); the WRAPPING does not — we
reuse letterbox / preprocess / NMS from rung1/detect_manual.py verbatim. That reuse is the whole
payoff of having hand-rolled the pipeline in Rung 1: only the middle slice swaps out.

Device-memory trick: torch is already here with CUDA, so we allocate the engine's input/output as
torch.cuda tensors and hand TensorRT their .data_ptr() — no pycuda / cuda-python needed, and the
data never leaves the GPU between preprocess, inference, and postprocess.

CAVEAT — TensorRT version: this targets the **TRT 10.x** Python API (I/O by tensor NAME,
set_tensor_address + execute_async_v3). On TRT 8.x the API differs (bindings by index, execute_v2).
Confirm after install: `python3 -c "import tensorrt as t; print(t.__version__)"`.

    python3 /ros2_ws/rung4/trt_infer.py --engine yolov8n.fp16.engine --image /ros2_ws/rung1/trucks.jpg
"""
import argparse
import os
import sys

import cv2
import torch
import tensorrt as trt

# Reuse the EXACT pre/post from Rung 1 — the point of this rung is that the wrapping is
# framework-independent, so we import it rather than copy it.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "rung1"))
from detect_manual import letterbox, preprocess, nms  # noqa: E402,F401  (letterbox via preprocess)


class TrtYolo:
    """Minimal TensorRT 10.x runner: load an engine, run one (1,3,640,640) input -> (1,84,8400)."""

    def __init__(self, engine_path, device=0):
        self.device = device
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as rt:
            self.engine = rt.deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        self.stream = torch.cuda.Stream(device=device)

        # Identify the input vs output tensor by name (static single-in / single-out for YOLOv8 detect).
        self.in_name = self.out_name = None
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.in_name = name
            else:
                self.out_name = name
        self.in_shape = tuple(self.engine.get_tensor_shape(self.in_name))    # expect (1,3,640,640)
        self.out_shape = tuple(self.engine.get_tensor_shape(self.out_name))  # expect (1,84,8400)

        # Pre-allocate the output on the GPU; TRT writes straight into it. fp32 because trtexec keeps
        # engine I/O at fp32 by default even with --fp16 (only internal layers go half).
        self.out_buf = torch.empty(self.out_shape, dtype=torch.float32, device=device)

    def infer(self, input_tensor):
        """input_tensor: (1,3,640,640) float32 CUDA tensor (from rung1 preprocess()). Returns out_buf."""
        x = input_tensor.contiguous()
        self.ctx.set_input_shape(self.in_name, self.in_shape)
        self.ctx.set_tensor_address(self.in_name, x.data_ptr())
        self.ctx.set_tensor_address(self.out_name, self.out_buf.data_ptr())
        self.ctx.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()
        return self.out_buf


def postprocess(out, ratio, pad, orig_w, orig_h, conf_thr=0.25, iou_thr=0.45):
    """IDENTICAL math to rung1/detect_manual.py main(): decode -> threshold -> xywh2xyxy -> NMS -> unpad."""
    pred = out[0].transpose(0, 1)                  # (1,84,8400) -> (8400,84)
    boxes_xywh, class_scores = pred[:, :4], pred[:, 4:]
    conf, cls = class_scores.max(dim=1)
    m = conf > conf_thr
    boxes_xywh, conf, cls = boxes_xywh[m], conf[m], cls[m]
    cx, cy, w, h = boxes_xywh.unbind(1)
    boxes = torch.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], dim=1)
    keep = nms(boxes, conf, iou_thr)
    boxes, conf, cls = boxes[keep], conf[keep], cls[keep]
    pad_left, pad_top = pad
    boxes[:, [0, 2]] -= pad_left
    boxes[:, [1, 3]] -= pad_top
    boxes /= ratio
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clamp(0, orig_w)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clamp(0, orig_h)
    return boxes, conf, cls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--engine", required=True)
    ap.add_argument("--image", required=True)
    ap.add_argument("--conf", type=float, default=0.25)
    args = ap.parse_args()

    assert torch.cuda.is_available(), "CUDA not available (check Rung 0)"
    yolo = TrtYolo(args.engine)
    print(f"engine loaded | in {yolo.in_shape} -> out {yolo.out_shape}")

    img = cv2.imread(args.image)
    assert img is not None, f"could not read {args.image}"
    h, w = img.shape[:2]

    x, ratio, pad = preprocess(img)                # rung1 preprocess: returns a GPU tensor
    out = yolo.infer(x)
    boxes, conf, cls = postprocess(out, ratio, pad, w, h, args.conf)

    # Class ids only here (the benchmark cares about boxes/speed, not names). Sanity-check the boxes
    # against rung1's detect_manual.py on the same image — they should match closely (FP16 + a
    # different kernel path means tiny numeric drift, same as Ultralytics-vs-manual in Rung 1).
    print(f"\n{len(boxes)} detections (conf >= {args.conf}):")
    for b, c, k in zip(boxes.tolist(), conf.tolist(), cls.tolist()):
        print(f"  cls {int(k):<3} {c:.2f}  box={[round(v, 1) for v in b]}")


if __name__ == "__main__":
    main()
