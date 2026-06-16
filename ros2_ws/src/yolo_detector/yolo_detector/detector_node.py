#!/usr/bin/env python3
"""M3 Rung 2 — yolo_detector node: /image_raw -> YOLO -> /detections.

Subscribes to the camera's sensor_msgs/Image (bgr8, from csi_camera), runs the pretrained YOLO model
on the Orin GPU, and publishes vision_msgs/Detection2DArray on /detections. This is the same model
and pipeline proved in Rung 1 (detect_ultralytics.py / detect_manual.py), now wired into the ROS
graph so detections flow live. Inference stays Ultralytics-first through Rung 3; Rung 4 swaps the
torch forward pass for a TensorRT engine while this node's plumbing stays put.

WHY no cv_bridge: cv_bridge pulls OpenCV's GStreamer-linked build — the same one whose
VideoCapture double-frees on this L4T image (see csi_camera). We unpack the Image buffer into a
numpy array by hand instead: three lines, zero extra dependency.

QoS: subscribe with sensor-data QoS (best-effort, keep-last) to MATCH the camera publisher. A
reliable subscriber will NOT connect to a best-effort publisher (incompatible QoS) — you'd get an
established graph but zero frames, the quietest possible failure.

Run (inside the L4T container, after `colcon build` + sourcing install/setup.bash), with the
csi_camera node already publishing /image_raw on domain 10:
    ros2 run yolo_detector detector_node
Verify:
    ros2 topic echo /detections --qos-reliability best_effort
"""
import numpy as np
import rclpy  # noqa: E402
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from ultralytics import YOLO


class YoloDetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("device", 0)          # 0 = first CUDA device (Orin iGPU)
        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("detections_topic", "detections")

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.conf = float(g("conf"))
        self.device = g("device")

        # Load once at startup; the first ever run downloads the weights to the ultralytics cache.
        self.model = YOLO(g("model"))
        self.names = self.model.names
        self.get_logger().info(f"Loaded {g('model')} on device {self.device}")

        self.pub = self.create_publisher(Detection2DArray, g("detections_topic"), 10)
        self.sub = self.create_subscription(
            Image, g("image_topic"), self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Subscribed to '{g('image_topic')}' -> Detection2DArray on '{g('detections_topic')}'"
        )

    def on_image(self, msg: Image):
        if msg.encoding != "bgr8":
            self.get_logger().warn(
                f"expected bgr8, got '{msg.encoding}'", throttle_duration_sec=5.0
            )
            return

        # sensor_msgs/Image -> HxWx3 BGR numpy. Use msg.step (the real row stride) because a row may
        # be padded beyond width*3 bytes; slice off the padding before reshaping to (H, W, 3).
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        frame = frame[:, : msg.width * 3].reshape(msg.height, msg.width, 3)

        # Ultralytics takes a BGR numpy array directly and runs letterbox/normalize/forward/NMS
        # internally (exactly what detect_manual.py reconstructs). device keeps it on the GPU.
        results = self.model(frame, conf=self.conf, device=self.device, verbose=False)
        r = results[0]

        out = Detection2DArray()
        out.header = msg.header          # reuse the camera's stamp + frame_id so Rung 3 overlays line up
        for b in r.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            det = Detection2D()
            det.header = msg.header
            # vision_msgs wants box CENTER + size; YOLO gives corners (xyxy). NOTE: this targets the
            # Humble vision_msgs (4.x) API — BoundingBox2D.center is a vision_msgs/Pose2D whose
            # .position is a vision_msgs/Point2D. If a build errors on these field names, check
            # `ros2 interface show vision_msgs/msg/BoundingBox2D` and adjust.
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.names[int(b.cls.item())]
            hyp.hypothesis.score = float(b.conf.item())
            det.results.append(hyp)
            out.detections.append(det)

        self.pub.publish(out)


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
