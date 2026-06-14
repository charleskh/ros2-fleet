#!/usr/bin/env python3
"""Publish a Jetson CSI camera (IMX219 via Argus) as a ROS2 Image topic.

Frames are pulled via GStreamer's Python bindings (gi / GstApp appsink), NOT via
cv2.VideoCapture(CAP_GSTREAMER): OpenCV 4.8's GStreamer capture double-frees on this L4T image
when handling the nvargus NVMM pipeline. We map the appsink buffer into bytes and pack
sensor_msgs/Image by hand — no OpenCV in the process at all. Reusable on any Jetson + CSI camera
(a Tier-2 fleet capability; see datasmith docs/phase1-hyperion-perception.md).
"""
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp  # noqa: E402  (must follow require_version)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402


def gst_pipeline(sensor_id, cap_w, cap_h, flip_method, disp_w, disp_h):
    """nvarguscamerasrc (Argus/ISP) -> nvvidconv (GPU convert + optional flip) -> BGR -> appsink.

    appsink ``max-buffers=1 drop=true`` keeps us on the latest frame (low latency); ``sync=false``
    lets it deliver as fast as captured rather than throttling to the clock.

    We deliberately do NOT pin a framerate on the sensor caps: some resolutions only have a 60 fps
    sensor mode (e.g. 1280x720), and requesting 30 fps there matches no mode, so Argus delivers no
    frames at all. Let the sensor run at its native rate; our publish timer + ``drop=true`` set the
    effective output rate.
    """
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={cap_w},height={cap_h} ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw,width={disp_w},height={disp_h},format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink name=sink max-buffers=1 drop=true sync=false"
    )


class CsiCameraNode(Node):
    def __init__(self):
        super().__init__("csi_camera")

        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("capture_width", 1280)
        self.declare_parameter("capture_height", 720)
        self.declare_parameter("display_width", 1280)
        self.declare_parameter("display_height", 720)
        self.declare_parameter("framerate", 30)
        self.declare_parameter("flip_method", 0)  # 0=none, 2=180deg (cam mounted upside down)
        self.declare_parameter("frame_id", "csi_camera")
        self.declare_parameter("topic", "image_raw")

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        fps = g("framerate")
        self.frame_id = g("frame_id")
        topic = g("topic")

        Gst.init(None)
        pipeline_str = gst_pipeline(
            g("sensor_id"), g("capture_width"), g("capture_height"),
            g("flip_method"), g("display_width"), g("display_height"),
        )
        self.get_logger().info(f"Opening CSI camera:\n{pipeline_str}")

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("sink")
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to set the GStreamer pipeline to PLAYING")

        self.pull_timeout_ns = int(0.5 * Gst.SECOND)
        # Sensor QoS = best-effort, keep-last: drop stale frames rather than retransmit. Reliable
        # QoS on multi-MB images collapses over a LAN (retransmit storms). Subscribers must also
        # use best-effort to match (e.g. `ros2 topic hz <t> --qos-reliability best_effort`).
        self.pub = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / float(fps), self.tick)
        self.get_logger().info(f"Publishing BGR frames on '{topic}' at {fps} fps")

    def tick(self):
        sample = self.appsink.try_pull_sample(self.pull_timeout_ns)
        if sample is None:
            self.get_logger().warn("No frame from appsink", throttle_duration_sec=2.0)
            return

        struct = sample.get_caps().get_structure(0)
        w = struct.get_value("width")
        h = struct.get_value("height")

        buf = sample.get_buffer()
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            self.get_logger().warn("Failed to map GStreamer buffer", throttle_duration_sec=2.0)
            return
        try:
            data = bytes(mapinfo.data)   # copy the frame out before unmapping
            size = mapinfo.size
        finally:
            buf.unmap(mapinfo)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = h
        msg.width = w
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = size // h             # actual row stride (handles any GStreamer row padding)
        msg.data = data
        self.pub.publish(msg)

    def destroy_node(self):
        if getattr(self, "pipeline", None) is not None:
            self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CsiCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if node is not None:
            node.get_logger().error(f"Camera node error: {exc}")
        else:
            print(f"Camera node failed to start: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():           # the SIGINT handler may have already shut down the context
            rclpy.shutdown()


if __name__ == "__main__":
    main()
