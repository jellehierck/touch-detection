from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Bring up the Franka robot for the hand-eye calibration."""
    # ---------- Define common paths ----------

    default_linear_velocity_controller_config_path = str(
        Path(get_package_share_directory("linear_velocity_controller")) / "config" / "linear_velocity_controller.yaml"
    )

    # ---------- Declare launch arguments ----------

    launch_args = []

    # Specify the parameter configuration file path for linear_velocity_controller
    linear_velocity_controller_config_path_arg = DeclareLaunchArgument(
        "linear_velocity_controller_config_path",
        default_value=TextSubstitution(text=default_linear_velocity_controller_config_path),
        description="(string) Path to the parameter configuration file for linear_velocity_controller.",
    )
    launch_args.append(linear_velocity_controller_config_path_arg)

    # ---------- Declare executables ----------

    # Controller spawner executable
    linear_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "linear_velocity_controller",
            # "--inactive",  # Optional: when enabled, load and configure the controller but do not activate
            "--param-file",
            LaunchConfiguration(linear_velocity_controller_config_path_arg.name),
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
