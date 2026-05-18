from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share_dir = Path(get_package_share_directory("sprint_drone"))
    headless = LaunchConfiguration("headless")
    show_camera = LaunchConfiguration("show_camera")
    use_sim_pose = LaunchConfiguration("use_sim_pose")

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo server-only for headless report/CI capture.",
        ),
        DeclareLaunchArgument(
            "show_camera",
            default_value="true",
            description="Launch rqt_image_view for the downward camera feed.",
        ),
        DeclareLaunchArgument(
            "use_sim_pose",
            default_value="false",
            description="Use gz_drone_sim pose instead of visual_odometry.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share_dir / "launch" / "sim_gazebo.launch.py")),
            launch_arguments={
                "headless": headless,
                "show_camera": show_camera,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share_dir / "launch" / "sim_perception.launch.py")),
            launch_arguments={"use_sim_pose": use_sim_pose}.items(),
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
