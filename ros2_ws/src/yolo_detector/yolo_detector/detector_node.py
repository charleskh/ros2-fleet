#!/usr/bin/env python3
"""yolo_detector node: /image_raw -> YOLO -> /detections (+ annotated /image_annotated).

Subscribes to the camera's sensor_msgs/Image (bgr8, from csi_camera), runs a pretrained YOLO model on
the Orin GPU, and publishes vision_msgs/Detection2DArray on /detections plus an annotated
sensor_msgs/Image on /image_annotated (boxes drawn, for Foxglove).

Two inference backends (parameter `backend`):
  ultralytics (default) — Ultralytics runs letterbox/normalize/forward/NMS internally (Rungs 2-3).
  trt                   — the TensorRT FP16 engine from Rung 4 (~5.8x faster forward pass), via
                          yolo_detector.trt_backend (the productized Rung1/Rung4 pipeline). Needs
                          engine_path to a .engine built ON this Orin (it's hardware-specific; build
                          with ros2_ws/rung4/, it's gitignored). Closes the M3 DoD: live detections
                          in Foxglove, on the GPU, via TensorRT.

WHY no cv_bridge: cv_bridge pulls OpenCV's GStreamer-linked build — the same one whose VideoCapture
double-frees on this L4T image (see csi_camera). We unpack the Image buffer into numpy by hand. The
cv2 drawing/imread path is fine; only cv2's GStreamer capture is cursed.

QoS: subscribe with sensor-data QoS (best-effort, keep-last) to MATCH the camera publisher. A
reliable subscriber will NOT connect to a best-effort publisher — established graph, zero frames.

Run (after `colcon build` + sourcing install/setup.bash), camera already publishing /image_raw:
    ros2 run yolo_detector detector_node                                       # torch/Ultralytics
    ros2 run yolo_detector detector_node --ros-args \
        -p backend:=trt -p engine_path:=/ros2_ws/rung4/yolov8n.fp16.engine     # TensorRT FP16
Verify: ros2 topic echo /detections --qos-reliability best_effort
"""
import time

import numpy as np
import rclpy  # noqa: E402
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# --- M6 Rung 1: Prometheus instrumentation (the app-metrics tier) ----------------------------------
# This is where "export our own metrics" becomes CODE (vs. Rung 0's off-the-shelf node-exporter).
# prometheus_client runs a tiny HTTP server in a background thread; caliban's Prometheus scrapes it.
# The metric objects live in the default registry (module-level = one set per process).
#
# Metric NAMING follows Prometheus convention: app prefix (detector_), base unit in the name
# (_seconds, _total). Histograms expose _bucket/_sum/_count so we get p50/p95 via histogram_quantile()
# AND a rate()-able _sum+_count — one instrument, both the distribution and the average.
from prometheus_client import Counter, Histogram, start_http_server

# Frames the node fully processed. FPS on the dashboard = rate(detector_frames_processed_total[1m]).
FRAMES = Counter("detector_frames_processed_total", "Frames processed to completion by the detector")

# Backend compute latency: frame-in -> detections-out (preprocess+infer+postprocess for trt; the
# single model() call for ultralytics). Labelled by backend so trt vs. ultralytics stay separable.
# Buckets span ~1ms..0.5s — TRT FP16 lives at the low end, ultralytics/CPU fallback higher.
INFER = Histogram(
    "detector_inference_latency_seconds", "Backend inference latency (frame -> detections)",
    ["backend"],
    buckets=(0.001, 0.0025, 0.005, 0.01, 0.025, 0.05, 0.075, 0.1, 0.25, 0.5),
)

# END-TO-END latency (the robotics-new bit): wall time from the camera's CAPTURE stamp
# (msg.header.stamp, set upstream by csi_camera) to the moment we publish detections. This measures
# latency ACROSS THE GRAPH using message timestamps — transport + queueing + compute — NOT one
# process's wall-clock. Valid because both nodes share hyperion's clock (no use_sim_time here).
E2E = Histogram(
    "detector_e2e_latency_seconds", "Capture-stamp -> detections-published latency (across the graph)",
    buckets=(0.005, 0.01, 0.025, 0.05, 0.075, 0.1, 0.15, 0.2, 0.3, 0.5, 1.0),
)

# Distribution of detections per frame — a cheap sanity signal (all-zero = model/feed broken;
# a shift tracks scene busyness). Histogram so the dashboard can show the spread, not just a mean.
DETS = Histogram(
    "detector_detections_per_frame", "Number of detections emitted per frame",
    buckets=(0, 1, 2, 3, 4, 5, 8, 13, 21, 50),
)


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("backend", "ultralytics")    # "ultralytics" | "trt"
        self.declare_parameter("engine_path", "")           # required when backend == "trt"
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)                 # NMS IoU (trt backend)
        self.declare_parameter("device", 0)                 # 0 = first CUDA device (Orin iGPU)
        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("detections_topic", "detections")
        self.declare_parameter("annotated_topic", "image_annotated")
        self.declare_parameter("publish_annotated", True)   # false = skip drawing (cheaper)
        self.declare_parameter("metrics_port", 9808)         # Prometheus scrape endpoint (host net)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.backend = g("backend")
        self.conf = float(g("conf"))
        self.iou = float(g("iou"))
        self.device = g("device")
        self.publish_annotated = bool(g("publish_annotated"))

        if self.backend == "trt":
            # Lazy import so the default path never pulls tensorrt.
            from yolo_detector import trt_backend as tb
            self.tb = tb
            engine = g("engine_path")
            if not engine:
                raise RuntimeError("backend:=trt requires -p engine_path:=/path/to/*.engine")
            self.trt = tb.TrtYolo(engine, device=self.device)
            self.names = tb.COCO_NAMES
            self.model = None
            self.get_logger().info(f"TensorRT backend | engine {engine} | in {self.trt.in_shape}")
        else:
            from ultralytics import YOLO
            self.model = YOLO(g("model"))
            self.names = self.model.names
            self.trt = None
            self.get_logger().info(f"Ultralytics backend | {g('model')} on device {self.device}")

        self.pub = self.create_publisher(Detection2DArray, g("detections_topic"), 10)
        # Annotated image uses sensor QoS (best-effort) like the camera — large buffer, and the
        # foxglove_bridge subscribes best-effort (see run_bridge.sh).
        self.ann_pub = (
            self.create_publisher(Image, g("annotated_topic"), qos_profile_sensor_data)
            if self.publish_annotated else None
        )
        self.sub = self.create_subscription(
            Image, g("image_topic"), self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Subscribed to '{g('image_topic')}' -> Detection2DArray on '{g('detections_topic')}'"
            + (f" + annotated Image on '{g('annotated_topic')}'" if self.ann_pub else "")
        )

        # Start the Prometheus endpoint LAST + defensively: monitoring must never take down the thing
        # it monitors, so a bind failure (port taken) is a warning, not a crash — perception runs on.
        port = int(g("metrics_port"))
        try:
            start_http_server(port)
            self.get_logger().info(f"Prometheus metrics on :{port}/metrics")
        except OSError as e:
            self.get_logger().warn(f"metrics endpoint disabled — bind :{port} failed: {e}")

    def on_image(self, msg: Image):
        if msg.encoding != "bgr8":
            self.get_logger().warn(f"expected bgr8, got '{msg.encoding}'", throttle_duration_sec=5.0)
            return

        # sensor_msgs/Image -> HxWx3 BGR numpy. Use msg.step (real row stride; may be padded beyond
        # width*3), slice off padding, reshape. (read-only view — copy before any in-place draw.)
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        frame = frame[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        h, w = msg.height, msg.width

        # Each backend yields a common list: ((x1,y1,x2,y2), score, cls_id). annotated may be None.
        # Time the whole frame->detections compute as "inference latency" (labelled by backend).
        t0 = time.perf_counter()
        if self.trt is not None:
            x, ratio, pad = self.tb.preprocess(frame, device=self.device)
            out = self.trt.infer(x)
            boxes, conf, cls = self.tb.postprocess(out, ratio, pad, w, h, self.conf, self.iou)
            dets = list(zip(boxes.tolist(), conf.tolist(), [int(c) for c in cls.tolist()]))
            annotated = self.tb.draw_boxes(frame, dets, self.names) if self.ann_pub else None
        else:
            r = self.model(frame, conf=self.conf, device=self.device, verbose=False)[0]
            dets = [(b.xyxy[0].tolist(), float(b.conf.item()), int(b.cls.item())) for b in r.boxes]
            annotated = r.plot() if self.ann_pub else None     # Ultralytics draws (reuses inference)
        INFER.labels(self.backend).observe(time.perf_counter() - t0)

        out_msg = Detection2DArray()
        out_msg.header = msg.header          # reuse the camera stamp + frame_id so overlays line up
        for (x1, y1, x2, y2), score, cls_id in dets:
            det = Detection2D()
            det.header = msg.header
            # vision_msgs wants box CENTER + size; YOLO gives corners (xyxy). Humble vision_msgs 4.x:
            # BoundingBox2D.center is a Pose2D whose .position is a Point2D (confirmed at runtime).
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.names[cls_id]
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            out_msg.detections.append(det)
        self.pub.publish(out_msg)

        # --- Rung 1 metrics: record AFTER publish so e2e captures the full path to output ---------
        FRAMES.inc()
        DETS.observe(len(dets))
        # End-to-end latency = now - capture stamp. Guard: skip if the camera didn't set a stamp
        # (sec == 0 -> "now - epoch" is a garbage ~1.7e9 s), and drop absurd values from a transient
        # clock skew so one bad sample can't wreck the histogram.
        stamp = msg.header.stamp
        if stamp.sec > 0:
            e2e = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
            if 0.0 <= e2e < 10.0:
                E2E.observe(e2e)

        if self.ann_pub is not None and annotated is not None:
            ann = Image()
            ann.header = msg.header
            ann.height = int(annotated.shape[0])
            ann.width = int(annotated.shape[1])
            ann.encoding = "bgr8"
            ann.is_bigendian = 0
            ann.step = ann.width * 3
            ann.data = annotated.tobytes()
            self.ann_pub.publish(ann)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():               # the SIGINT handler may have already shut down the context
            rclpy.shutdown()


if __name__ == "__main__":
    main()
