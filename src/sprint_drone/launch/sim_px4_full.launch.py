from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share_dir = Path(get_package_share_directory("sprint_drone"))
    px4_bridge_config = str(share_dir / "config" / "bridge_px4.yaml")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share_dir / "launch" / "sim_gazebo.launch.py")),
            launch_arguments={
                "use_px4_sitl": "true",
                "bridge_config": px4_bridge_config,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share_dir / "launch" / "sim_perception.launch.py"))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share_dir / "launch" / "sim_planner.launch.py"))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share_dir / "launch" / "sim_mission.launch.py"))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share_dir / "launch" / "sim_control.launch.py"))
        ),
    ])
