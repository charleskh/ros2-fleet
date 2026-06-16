#!/usr/bin/env python3
"""M3 Rung 4, step 1 — export the pretrained YOLOv8n to ONNX.

ONNX is the portable graph format TensorRT ingests (a frozen description of the network: ops +
weights + shapes, no framework). Ultralytics handles this export — it's the LAST Ultralytics-managed
step. From here we hand-roll the engine build (trtexec), the inference (TensorRT runtime), and the
pre/post (reused verbatim from rung1), so every layer is reconstructed, not black-boxed.

Static 640x640 input, opset 12, simplified graph — fixed + reproducible so the engine build is
deterministic.

    python3 /ros2_ws/rung4/export_onnx.py            # -> yolov8n.onnx (next to the cached weights)
"""
import argparse

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--opset", type=int, default=12)
    args = ap.parse_args()

    model = YOLO(args.model)
    # dynamic=False: fixed shapes keep the TensorRT engine simpler + faster (we always feed 640x640).
    # simplify=True: fold constants / clean up the graph so the TRT parser has less to chew on.
    path = model.export(format="onnx", imgsz=args.imgsz, opset=args.opset, dynamic=False, simplify=True)
    print(f"ONNX written -> {path}")


if __name__ == "__main__":
    main()
