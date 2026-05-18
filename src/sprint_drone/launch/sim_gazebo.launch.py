"""
sim_gazebo.launch.py — Gazebo Harmonic + ROS-GZ bridge launch.

Starts:
  1. Gazebo Harmonic with sprint_grid_world.sdf
  2. ros_gz_bridge — bridges camera/IMU Gz topics to ROS2
  3. ros_gz_image  — handles compressed image transport (optional)

Prerequisites (install once):
  sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-image

Usage:
  ros2 launch sprint_drone sim_gazebo.launch.py
  ros2 launch sprint_drone sim_gazebo.launch.py headless:=true show_camera:=false
  ros2 launch sprint_drone sim_gazebo.launch.py use_px4_sitl:=true

Notes:
  - Standalone mode (default): uses the sprint_camera_rig model embedded in
    the world file for downward camera images.
  - PX4 SITL mode (use_px4_sitl:=true): disable the embedded camera rig and
    rely on the x500 drone model's camera. Adjust bridge.yaml gz_topic_name
    to match the PX4 sensor topic.

ArUco texture setup (run once before first launch):
  python3 tools/gen_aruco_markers.py

Random marker placement (optional, run before each test):
  python3 tools/spawn_markers.py --seed <N> --write
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory("sprint_drone"))
    default_world_file = str(share_dir / "worlds" / "sprint_grid_world.sdf")
    default_bridge_config = str(share_dir / "config" / "bridge.yaml")
    models_dir = str(share_dir / "models")

    use_px4_sitl = LaunchConfiguration("use_px4_sitl", default="false")
    headless = LaunchConfiguration("headless", default="false")
    world_file = LaunchConfiguration("world_file")
    bridge_config = LaunchConfiguration("bridge_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_px4_sitl",
            default_value="false",
            description=(
                "false = standalone mode with embedded camera rig; "
                "true = PX4 SITL mode (expects x500 model to carry camera)"
            ),
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=default_world_file,
            description="SDF world file used in standalone mode.",
        ),
        DeclareLaunchArgument(
            "bridge_config",
            default_value=default_bridge_config,
            description="ros_gz_bridge YAML mapping file.",
        ),
        DeclareLaunchArgument(
            "show_camera",
            default_value="true",
            description="Launch rqt_image_view to display the downward camera feed.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo server only without GUI. Useful for report/CI data capture.",
        ),

        # ── Make Gazebo find our model directory ─────────────────────
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=models_dir,
        ),

        # ── Gazebo Harmonic ──────────────────────────────────────────
        # Run server + GUI together by default. The headless path runs the same
        # world server-only so report data can be captured on machines without
        # a display.
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world_file],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", use_px4_sitl, "' == 'false' and '", headless, "' == 'false'"
            ])),
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "-s", world_file],
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", use_px4_sitl, "' == 'false' and '", headless, "' == 'true'"
            ])),
        ),
        # In PX4 SITL mode, PX4's make target launches Gazebo itself.
        # Only start the bridge — do not launch Gazebo here.

        # ── ROS-GZ bridge ────────────────────────────────────────────
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config}],
        ),

        # ── Downward-camera viewer ───────────────────────────────────
        # Shows /drone/camera/down/image_raw in a separate window.
        # Pass show_camera:=false to suppress (e.g. in headless CI).
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="camera_view",
            arguments=["/drone/camera/down/image_raw"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("show_camera")),
        ),
    ])
