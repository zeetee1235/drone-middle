from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    repo_root = Path(__file__).resolve().parents[1]
    default_world_file = str(repo_root / "src" / "sprint_drone" / "worlds" / "sprint_grid_world.sdf")
    default_bridge_config = str(repo_root / "src" / "sprint_drone" / "config" / "bridge.yaml")
    models_dir = str(repo_root / "src" / "sprint_drone" / "models")
    use_px4_sitl = LaunchConfiguration("use_px4_sitl", default="false")
    world_file = LaunchConfiguration("world_file")
    bridge_config = LaunchConfiguration("bridge_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_px4_sitl",
            default_value="false",
            description="false launches Gazebo directly; true expects PX4 SITL to launch Gazebo",
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
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=models_dir),
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world_file],
            output="screen",
            condition=UnlessCondition(use_px4_sitl),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config}],
        ),
    ])
