#!/usr/bin/env python3
"""TensorRT backend for yolo_detector (M3 Rung 4 → productized).

This is the Rung 1/Rung 4 pipeline, validated against the torch path on trucks.jpg (boxes matched to
sub-pixel, 5.8x faster forward pass), lifted out of the loose ros2_ws/rung1+rung4 scripts and into
the installed package so the live ROS node can use it without sys.path hacks. The learning scripts
stay as the teaching artifacts; this is the production copy.

Only imported when the node runs with backend:=trt, so the default (Ultralytics) path never pulls
`tensorrt`. Device memory is torch CUDA tensors handed to TensorRT by .data_ptr() — no pycuda.
Targets the TensorRT 10.x API (I/O by name, set_tensor_address + execute_async_v3).
"""
import cv2
import numpy as np
import torch
import tensorrt as trt

IMG_SIZE = 640

# COCO 80 classes, in model order (the engine carries ids, not names).
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush",
]


def letterbox(img, new_size=IMG_SIZE, color=(114, 114, 114)):
    """Aspect-preserving resize to a square canvas; returns (img, scale_ratio, (pad_left, pad_top))."""
    h, w = img.shape[:2]
    ratio = min(new_size / h, new_size / w)
    nh, nw = round(h * ratio), round(w * ratio)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_w, pad_h = new_size - nw, new_size - nh
    left, top = pad_w // 2, pad_h // 2
    out = cv2.copyMakeBorder(resized, top, pad_h - top, left, pad_w - left,
                             cv2.BORDER_CONSTANT, value=color)
    return out, ratio, (left, top)


def preprocess(img, device=0):
    """BGR HWC uint8 -> normalized RGB CHW float32 (1,3,640,640) GPU tensor. + ratio + pad for undo."""
    lb, ratio, (left, top) = letterbox(img)
    rgb = lb[:, :, ::-1]
    chw = rgb.transpose(2, 0, 1)
    arr = np.ascontiguousarray(chw, dtype=np.float32) / 255.0
    tensor = torch.from_numpy(arr).unsqueeze(0)
    return tensor.to(device), ratio, (left, top)


def nms(boxes, scores, iou_thr=0.45):
    """Class-agnostic NMS. boxes: (N,4) xyxy. Returns kept indices."""
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort(descending=True)
    keep = []
    while order.numel() > 0:
        i = order[0].item()
        keep.append(i)
        if order.numel() == 1:
            break
        rest = order[1:]
        xx1 = torch.maximum(x1[i], x1[rest]); yy1 = torch.maximum(y1[i], y1[rest])
        xx2 = torch.minimum(x2[i], x2[rest]); yy2 = torch.minimum(y2[i], y2[rest])
        inter = (xx2 - xx1).clamp(min=0) * (yy2 - yy1).clamp(min=0)
        iou = inter / (areas[i] + areas[rest] - inter)
        order = rest[iou <= iou_thr]
    return keep


def postprocess(out, ratio, pad, orig_w, orig_h, conf_thr=0.25, iou_thr=0.45):
    """(1,84,8400) grid -> (boxes xyxy in original pixels, conf, cls). Identical math to rung1."""
    pred = out[0].transpose(0, 1)                  # (8400,84)
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


def draw_boxes(frame, dets, names=COCO_NAMES):
    """dets: list of ((x1,y1,x2,y2), score, cls_id). Returns a fresh annotated BGR uint8 array."""
    img = frame.copy()                              # frombuffer view is read-only; copy to draw
    for (x1, y1, x2, y2), score, cls_id in dets:
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(img, p1, p2, (0, 255, 0), 2)
        label = f"{names[cls_id]} {score:.2f}"
        cv2.putText(img, label, (int(x1), max(12, int(y1) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return img


class TrtYolo:
    """Minimal TensorRT 10.x runner: one (1,3,640,640) input -> (1,84,8400) output."""

    def __init__(self, engine_path, device=0):
        self.device = device
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as rt:
            self.engine = rt.deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        self.stream = torch.cuda.Stream(device=device)

        self.in_name = self.out_name = None
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.in_name = name
            else:
                self.out_name = name
        self.in_shape = tuple(self.engine.get_tensor_shape(self.in_name))
        self.out_shape = tuple(self.engine.get_tensor_shape(self.out_name))
        self.out_buf = torch.empty(self.out_shape, dtype=torch.float32, device=device)

    def infer(self, input_tensor):
        x = input_tensor.contiguous()
        self.ctx.set_input_shape(self.in_name, self.in_shape)
        self.ctx.set_tensor_address(self.in_name, x.data_ptr())
        self.ctx.set_tensor_address(self.out_name, self.out_buf.data_ptr())
        self.ctx.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()
        return self.out_buf
