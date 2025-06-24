from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

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

# Specify the parameter configuration file path for linear_velocity_controller
linear_velocity_controller_inactive_arg = DeclareLaunchArgument(
    "linear_velocity_controller_inactive",
    default_value="false",
    description="(bool) Whether to load and configure the controller but not activate immediately.",
    choices=["true", "false"],
)
launch_args.append(linear_velocity_controller_inactive_arg)


def spawn_linear_velocity_controller(context: LaunchContext, *_args, **_kwargs) -> list[Node]:
    """Opaque function to spawn the linear velocity controller."""
    spawner_args = [
        "linear_velocity_controller",
        "--param-file",
        LaunchConfiguration(linear_velocity_controller_config_path_arg.name),
    ]

    set_inactive = LaunchConfiguration(linear_velocity_controller_inactive_arg.name).perform(context)
    if set_inactive == "true":
        spawner_args.append("--inactive")

    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=spawner_args,
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Bring up the Franka robot for the hand-eye calibration."""
    # ---------- Declare executables ----------

    # Controller spawner executable
    linear_velocity_controller_spawner = OpaqueFunction(function=spawn_linear_velocity_controller)

    # ---------- Construct launch description ----------

    return LaunchDescription(
        [
            *launch_args,  # Unpack the launch arguments
            linear_velocity_controller_spawner,
        ]
    )
