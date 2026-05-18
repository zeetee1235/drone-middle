from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory("sprint_drone"))
    params = str(share_dir / "config" / "ros_params.yaml")

    use_sim_pose = LaunchConfiguration("use_sim_pose")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_pose",
            default_value="false",
            description="Use simulated pose from gz_drone_sim instead of visual odometry",
        ),
        Node(
            package="grid_detector",
            executable="grid_detector_node",
            name="grid_detector",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="aruco_tracker",
            executable="aruco_tracker_node",
            name="aruco_tracker",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="visual_odometry",
            executable="visual_odometry_node",
            name="visual_odometry",
            output="screen",
            parameters=[params],
            condition=UnlessCondition(use_sim_pose),
        ),
        Node(
            package="vertiport_detector",
            executable="vertiport_detector_node",
            name="vertiport_detector",
            output="screen",
            parameters=[params],
        ),
    ])
