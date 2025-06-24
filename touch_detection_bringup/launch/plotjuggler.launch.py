from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """Bring up plotjuggler."""
    # ---------- Define common paths ----------

    default_plotjuggler_layout_path = str(
        Path(get_package_share_directory("touch_detection_bringup")) / "plotjuggler" / "touch_detection.xml"
    )

    # ---------- Declare launch arguments ----------

    launch_args = []

    # Specify the layout file for plotjuggler
    plotjuggler_layout_path_arg = DeclareLaunchArgument(
        "plotjuggler_layout_path",
        default_value=TextSubstitution(text=default_plotjuggler_layout_path),
        description="(string) Path to the layout specification for Plotjuggler.",
    )
    launch_args.append(plotjuggler_layout_path_arg)

    # ---------- Declare executables ----------

    # Controller spawner executable
    linear_velocity_controller_spawner = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=[
            "--layout",
            LaunchConfiguration(plotjuggler_layout_path_arg.name),
            "--buffer_size",
            "30",
            "--nosplash",
        ],
        output="screen",
    )

    # ---------- Construct launch description ----------

    return LaunchDescription(
        [
            *launch_args,  # Unpack the launch arguments
            linear_velocity_controller_spawner,
        ]
    )
