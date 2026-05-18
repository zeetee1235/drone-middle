from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="grid_detector",
            executable="grid_detector_node",
            name="grid_detector",
            output="screen",
            parameters=["config/ros_params.yaml"],
        ),
        Node(
            package="aruco_tracker",
            executable="aruco_tracker_node",
            name="aruco_tracker",
            output="screen",
            parameters=["config/ros_params.yaml"],
        ),
        Node(
            package="visual_odometry",
            executable="visual_odometry_node",
            name="visual_odometry",
            output="screen",
            parameters=["config/ros_params.yaml"],
        ),
        Node(
            package="vertiport_detector",
            executable="vertiport_detector_node",
            name="vertiport_detector",
            output="screen",
            parameters=["config/ros_params.yaml"],
        ),
    ])
