from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource("launch/sim_gazebo.launch.py")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource("launch/sim_perception.launch.py")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource("launch/sim_planner.launch.py")),
        Node(
            package="mission_manager",
            executable="mission_manager_node",
            name="mission_manager",
            output="screen",
            parameters=["config/ros_params.yaml"],
        ),
        Node(
            package="px4_offboard_control",
            executable="px4_offboard_control_node",
            name="px4_offboard_control",
            output="screen",
            parameters=["config/ros_params.yaml"],
        ),
    ])
