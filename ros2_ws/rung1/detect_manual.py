#!/usr/bin/env python3
"""M3 Rung 1 (learning) — reconstruct what Ultralytics does, step by step.

Same result as detect_ultralytics.py, but every stage is explicit so you can explain it:
  preprocess:  letterbox -> BGR2RGB -> /255 -> HWC2CHW -> batch -> GPU tensor
  forward:     raw model output, a (1, 84, 8400) grid tensor (NO objectness in YOLOv8/v11)
  postprocess: decode xywh -> xyxy, confidence threshold, NMS, scale back to original pixels

Cross-check: the boxes printed here should closely match detect_ultralytics.py's. If they do, you
understand the pipeline. This hand-rolled pre/post is exactly what gets reused in Rung 4 when the
torch forward pass is replaced by a TensorRT engine — the model changes, the wrapping doesn't.

    docker compose run --rm ros2-jetson \
        python3 /ros2_ws/rung1/detect_manual.py --image /ros2_ws/rung1/sample.jpg

NOTE: NMS here is class-agnostic for clarity; Ultralytics does it per-class, so a rare box may
differ. That's expected and a good thing to notice.
"""
import argparse

import cv2
import numpy as np
import torch
from ultralytics import YOLO

IMG_SIZE = 640  # the square input the model expects


def letterbox(img, new_size=IMG_SIZE, color=(114, 114, 114)):
    """Resize keeping aspect ratio, pad the remainder with gray to a square.

    WHY: the model needs a fixed square input, but squashing straight to 640x640 distorts objects
    (a tall person becomes short+fat -> worse detections). Letterboxing scales by the *limiting*
    dimension and pads the other, so geometry is preserved. We return the scale ratio + (left, top)
    padding so postprocessing can map boxes back to the ORIGINAL image.
    """
    h, w = img.shape[:2]
    ratio = min(new_size / h, new_size / w)            # scale by the tighter dimension
    nh, nw = round(h * ratio), round(w * ratio)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_w, pad_h = new_size - nw, new_size - nh        # leftover to fill
    left, top = pad_w // 2, pad_h // 2                 # center the image in the canvas
    out = cv2.copyMakeBorder(resized, top, pad_h - top, left, pad_w - left,
                             cv2.BORDER_CONSTANT, value=color)
    return out, ratio, (left, top)


def preprocess(img):
    """BGR HWC uint8  ->  normalized RGB CHW float32 batch tensor on the GPU."""
    lb, ratio, (left, top) = letterbox(img)
    rgb = lb[:, :, ::-1]                                # BGR->RGB (cv2 loads BGR; model trained RGB)
    chw = rgb.transpose(2, 0, 1)                        # HWC->CHW (torch wants channels first)
    arr = np.ascontiguousarray(chw, dtype=np.float32) / 255.0   # uint8 [0,255] -> float [0,1]
    tensor = torch.from_numpy(arr).unsqueeze(0)        # add batch dim -> (1,3,640,640)
    return tensor.to(0), ratio, (left, top)


def nms(boxes, scores, iou_thr=0.45):
    """Non-Max Suppression, hand-rolled so the algorithm is visible.

    The model emits many overlapping boxes for the same object. NMS keeps the highest-scoring box,
    drops every remaining box that overlaps it more than iou_thr (Intersection-over-Union), and
    repeats on what's left. boxes: (N,4) xyxy. Returns the indices to keep.
    """
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort(descending=True)            # process highest score first
    keep = []
    while order.numel() > 0:
        i = order[0].item()
        keep.append(i)
        if order.numel() == 1:
            break
        rest = order[1:]
        # IoU of box i against all remaining boxes (vectorized)
        xx1 = torch.maximum(x1[i], x1[rest]); yy1 = torch.maximum(y1[i], y1[rest])
        xx2 = torch.minimum(x2[i], x2[rest]); yy2 = torch.minimum(y2[i], y2[rest])
        inter = (xx2 - xx1).clamp(min=0) * (yy2 - yy1).clamp(min=0)
        iou = inter / (areas[i] + areas[rest] - inter)
        order = rest[iou <= iou_thr]                   # keep only boxes that DON'T overlap i much
    return keep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.45)
    args = ap.parse_args()

    assert torch.cuda.is_available(), "CUDA not available (check Rung 0)"
    print(f"torch {torch.__version__} | device: {torch.cuda.get_device_name(0)}")

    # Load the model, but grab the underlying torch nn.Module so we see the RAW output tensor,
    # not Ultralytics' already-decoded Results object.
    yolo = YOLO(args.model)
    net = yolo.model.to(0).eval()                       # the raw nn.Module
    names = yolo.names

    img = cv2.imread(args.image)
    assert img is not None, f"could not read {args.image}"
    orig_h, orig_w = img.shape[:2]

    x, ratio, (pad_left, pad_top) = preprocess(img)

    # FORWARD PASS -> raw grid tensor. For YOLOv8/v11 detect, inference output is (1, 84, 8400):
    #   84   = 4 box coords (cx, cy, w, h) + 80 COCO class scores  (note: NO objectness channel)
    #   8400 = total candidate boxes across 3 detection scales (80x80 + 40x40 + 20x20)
    with torch.no_grad():
        out = net(x)
    out = out[0] if isinstance(out, (list, tuple)) else out   # some builds wrap the tensor in a tuple
    pred = out[0].transpose(0, 1)                       # (84,8400) -> (8400,84): one row per box

    # DECODE: split coords vs class scores. YOLOv8/v11 confidence = max class score (no objectness).
    boxes_xywh = pred[:, :4]
    class_scores = pred[:, 4:]
    conf, cls = class_scores.max(dim=1)                 # best class + its score, per candidate box

    # confidence threshold FIRST (cheap) — drop the flood of low-score candidates before NMS
    m = conf > args.conf
    boxes_xywh, conf, cls = boxes_xywh[m], conf[m], cls[m]

    # xywh (center form) -> xyxy (corner form), still in the 640 letterboxed space
    cx, cy, w, h = boxes_xywh.unbind(1)
    boxes = torch.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], dim=1)

    keep = nms(boxes, conf, iou_thr=args.iou)
    boxes, conf, cls = boxes[keep], conf[keep], cls[keep]

    # UNDO the letterbox: subtract padding, divide by the scale ratio -> ORIGINAL image pixels
    boxes[:, [0, 2]] -= pad_left
    boxes[:, [1, 3]] -= pad_top
    boxes /= ratio
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clamp(0, orig_w)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clamp(0, orig_h)

    print(f"\n{len(boxes)} detections (conf >= {args.conf}, iou {args.iou}):")
    for b, c, k in zip(boxes.tolist(), conf.tolist(), cls.tolist()):
        x1, y1, x2, y2 = (round(v, 1) for v in b)
        print(f"  {names[int(k)]:<15} {c:.2f}  box=({x1}, {y1}, {x2}, {y2})")


if __name__ == "__main__":
    main()
