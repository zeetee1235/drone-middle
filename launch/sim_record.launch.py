from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_output = str(
        Path.cwd()
        / "bags"
        / f"sprint_drone_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    output = LaunchConfiguration("output")

    return LaunchDescription([
        DeclareLaunchArgument(
            "output",
            default_value=default_output,
            description="rosbag2 output directory",
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record",
                "--all-topics",
                "--exclude-topics",
                "/drone/camera/down/image_raw",
                "/parameter_events",
                "/rosout",
                "/events/write_split",
                "-o", output,
            ],
            output="screen",
        ),
    ])
