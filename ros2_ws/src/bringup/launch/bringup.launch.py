# Phase-1 perception graph, as ONE launch file (M5). This is the node-graph orchestration unit:
# `ros2 launch bringup bringup.launch.py` starts all three nodes together, and the M5 compose service
# runs exactly this. It is the declarative form of the three manual M4 shells:
#   camera_node                         (csi_camera/run — the CSI camera -> /image_raw)
#   bash run_detector.sh                (yolo_detector/detector_node backend:=trt -> /detections, /image_annotated)
#   bash run_bridge.sh                  (foxglove_bridge, best-effort QoS -> ws://<host>:8765)
# The individual run_*.sh scripts stay for single-node triage; this file is how the graph ships.
#
# WHY a launch file and not one Docker container per node: the container is the DEPLOYMENT unit, the
# launch file is the GRAPH unit (docs Concept G/J). Keeping the graph in one deployable lets the nodes
# share params/QoS/lifecycle and keeps the door open to a composable (zero-copy) container later.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch arguments: override at the CLI, e.g.
    #   ros2 launch bringup bringup.launch.py backend:=ultralytics
    #   ros2 launch bringup bringup.launch.py engine_path:=/ros2_ws/rung4/other.engine
    backend = LaunchConfiguration("backend")
    engine_path = LaunchConfiguration("engine_path")
    flip_method = LaunchConfiguration("flip_method")

    args = [
        DeclareLaunchArgument(
            "backend", default_value="trt",
            description="Detector backend: 'trt' (TensorRT FP16 engine, the live M3 path) or 'ultralytics'.",
        ),
        DeclareLaunchArgument(
            "engine_path", default_value="/ros2_ws/rung4/yolov8n.fp16.engine",
            description="Path to the Orin-specific TensorRT engine (required when backend:=trt). "
                        "Built by rung4/ and validated by the container entrypoint, not built here.",
        ),
        DeclareLaunchArgument(
            "flip_method", default_value="0",
            description="CSI camera rotation: 0=none, 2=180deg (if the camera is mounted upside down).",
        ),
    ]

    # --- The three nodes of the graph ---------------------------------------------------------
    camera = Node(
        package="csi_camera",
        executable="camera_node",
        name="csi_camera",
        parameters=[{"flip_method": flip_method}],
        output="screen",
    )

    detector = Node(
        package="yolo_detector",
        executable="detector_node",
        name="yolo_detector",
        parameters=[{"backend": backend, "engine_path": engine_path}],
        output="screen",
    )

    # foxglove_bridge must subscribe BEST-EFFORT to match the camera's sensor QoS, or the Image panel
    # stays black (reliable subscriber + best-effort publisher = no data). Same param as run_bridge.sh.
    bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{"best_effort_qos_topic_whitelist": [".*"]}],
        output="screen",
    )

    return LaunchDescription(args + [camera, detector, bridge])
