#!/usr/bin/env python3
"""Publish a Jetson CSI camera (IMX219 via Argus) as a ROS2 Image topic.

Opens ``nvarguscamerasrc`` through OpenCV's GStreamer backend, grabs BGR frames,
and publishes them as ``sensor_msgs/Image``. Reusable on any Jetson + CSI camera —
everything is parameterized, nothing Devastator-specific lives here (a Tier-2 fleet
capability, see datasmith docs/phase1-hyperion-perception.md).
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def gst_pipeline(sensor_id, cap_w, cap_h, fps, flip_method, disp_w, disp_h):
    """Build the nvarguscamerasrc -> BGR appsink pipeline OpenCV will read from.

    nvarguscamerasrc (Argus/ISP: debayer + auto-exposure) -> NVMM frames ->
    nvvidconv (GPU colour/format + optional flip) -> videoconvert -> BGR -> appsink.
    ``drop=true max-buffers=1`` keeps us on the *latest* frame (low latency) rather
    than queuing a backlog.
    """
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={cap_w},height={cap_h},framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw,width={disp_w},height={disp_h},format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=1"
    )


class CsiCameraNode(Node):
    def __init__(self):
        super().__init__("csi_camera")

        # Parameters — override at runtime without touching code, e.g.
        #   ros2 run csi_camera camera_node --ros-args -p flip_method:=2
        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("capture_width", 1920)
        self.declare_parameter("capture_height", 1080)
        self.declare_parameter("display_width", 1920)
        self.declare_parameter("display_height", 1080)
        self.declare_parameter("framerate", 30)
        self.declare_parameter("flip_method", 0)  # 0=none, 2=180deg (if cam is mounted upside down)
        self.declare_parameter("frame_id", "csi_camera")
        self.declare_parameter("topic", "image_raw")

        g = lambda n: self.get_parameter(n).value
        fps = g("framerate")
        disp_w, disp_h = g("display_width"), g("display_height")
        self.frame_id = g("frame_id")
        topic = g("topic")

        pipeline = gst_pipeline(
            g("sensor_id"), g("capture_width"), g("capture_height"),
            fps, g("flip_method"), disp_w, disp_h,
        )
        self.get_logger().info(f"Opening CSI camera:\n{pipeline}")

        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error(
                "Could not open the camera. Check: nvargus-daemon active on host, "
                "/tmp/argus_socket mounted, 'runtime: nvidia' set, and OpenCV built "
                "with GStreamer support."
            )
            raise RuntimeError("cv2.VideoCapture failed to open the CSI camera")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, 10)
        self.timer = self.create_timer(1.0 / float(fps), self.tick)
        self.get_logger().info(
            f"Publishing {disp_w}x{disp_h} BGR frames on '{topic}' at {fps} fps"
        )

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Frame grab failed", throttle_duration_sec=2.0)
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self):
        if getattr(self, "cap", None) is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CsiCameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        if node:
            node.get_logger().info(f"Shutting down: {exc}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
