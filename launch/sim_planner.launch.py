from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sprint_planner",
            executable="sprint_planner_node",
            name="sprint_planner",
            output="screen",
            parameters=["config/ros_params.yaml"],
        ),
    ])
