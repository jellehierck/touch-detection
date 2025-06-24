from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """Bring up RViz for the Touch Detection experiment."""
    # ---------- Define common paths ----------

    default_rviz2_config_path = str(
        Path(get_package_share_directory("touch_detection_bringup")) / "rviz2" / "touch_detection.rviz"
    )

    # ---------- Declare launch arguments ----------

    launch_args = []

    rviz_config_path_arg = DeclareLaunchArgument(
        "rviz_config_path",
        default_value=default_rviz2_config_path,
        description="(string) Which RViz2 configuration file to use. Does nothing if use_rviz is false.",
    )
    launch_args.append(rviz_config_path_arg)

    # ---------- Declare executables ----------

    # RViz2 for visualization of the robot
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", LaunchConfiguration(rviz_config_path_arg.name)],
    )

    # ---------- Construct launch description ----------

    return LaunchDescription(
        [
            *launch_args,  # Unpack the launch arguments
            rviz2_node,
        ]
    )
