#!/usr/bin/env python3
"""M3 Rung 1 — Ultralytics-first YOLO inference on ONE saved image, on the Orin GPU.

This is the WORKING checkpoint: prove the whole stack (torch + CUDA from Rung 0 + a pretrained
COCO model) runs end-to-end and uses the GPU. It deliberately hides pre/post-processing inside
`model(img)`; `detect_manual.py` reconstructs those steps so you can explain each one (the actual
Rung 1 learning goal).

Run inside the L4T container:
    docker compose run --rm ros2-jetson \
        python3 /ros2_ws/rung1/detect_ultralytics.py --image /ros2_ws/rung1/sample.jpg

Watch the GPU in a second terminal on the host:  tegrastats     (or: jtop)
"""
import argparse

import torch
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True, help="path to one image (e.g. a saved camera frame)")
    ap.add_argument("--model", default="yolov8n.pt", help="pretrained COCO weights (yolo11n.pt works too)")
    ap.add_argument("--conf", type=float, default=0.25, help="confidence threshold")
    args = ap.parse_args()

    # 1) Pick the device. Rung 0 proved CUDA works; fail LOUD if it regressed (e.g. installing
    #    ultralytics quietly pulled a CPU torch) instead of silently running ~30x slower on the CPU.
    assert torch.cuda.is_available(), "CUDA not available — torch is not GPU-enabled (check Rung 0)"
    device = 0  # first CUDA device = the Orin iGPU
    print(f"torch {torch.__version__} | device: {torch.cuda.get_device_name(0)}")

    # 2) Load the pretrained model. First run downloads the weights (~6 MB for nano) to the
    #    ultralytics cache. COCO = 80 classes (person, car, dog, ...).
    model = YOLO(args.model)

    # 3) Inference. Ultralytics does letterbox -> BGR2RGB -> /255 -> CHW -> forward -> decode ->
    #    NMS internally. detect_manual.py is where all of that becomes visible.
    results = model(args.image, conf=args.conf, device=device, verbose=True)

    # 4) Print every detection: class name, confidence, xyxy box (in ORIGINAL image pixels).
    r = results[0]
    names = r.names
    print(f"\n{len(r.boxes)} detections (conf >= {args.conf}):")
    for b in r.boxes:
        cls = int(b.cls.item())
        conf = float(b.conf.item())
        x1, y1, x2, y2 = (round(v, 1) for v in b.xyxy[0].tolist())
        print(f"  {names[cls]:<15} {conf:.2f}  box=({x1}, {y1}, {x2}, {y2})")

    # 5) Save an annotated copy so you can eyeball it (the live Foxglove overlay is Rung 3).
    out = "/ros2_ws/rung1/annotated_ultralytics.jpg"
    r.save(filename=out)
    print(f"\nannotated image -> {out}")


if __name__ == "__main__":
    main()
