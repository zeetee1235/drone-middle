from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory("sprint_drone"))
    params = str(share_dir / "config" / "ros_params.yaml")

    return LaunchDescription([
        Node(
            package="sprint_planner",
            executable="sprint_planner_node",
            name="sprint_planner",
            output="screen",
            parameters=[params],
        ),
    ])
