from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    repo_root = Path(__file__).resolve().parents[1]
    px4_bridge_config = str(repo_root / "src" / "sprint_drone" / "config" / "bridge_px4.yaml")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(repo_root / "launch" / "sim_gazebo.launch.py")),
            launch_arguments={
                "use_px4_sitl": "true",
                "bridge_config": px4_bridge_config,
            }.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(repo_root / "launch" / "sim_perception.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(repo_root / "launch" / "sim_planner.launch.py"))),
        Node(
            package="mission_manager",
            executable="mission_manager_node",
            name="mission_manager",
            output="screen",
            parameters=[str(repo_root / "src" / "sprint_drone" / "config" / "ros_params.yaml")],
        ),
        Node(
            package="px4_offboard_control",
            executable="px4_offboard_control_node",
            name="px4_offboard_control",
            output="screen",
            parameters=[str(repo_root / "src" / "sprint_drone" / "config" / "ros_params.yaml")],
        ),
    ])
