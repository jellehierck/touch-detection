from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Bring up the Franka robot for the hand-eye calibration."""
    # ---------- Define common paths ----------

    default_controller_manager_config_path = str(
        Path(get_package_share_directory("linear_velocity_controller")) / "config" / "controller_manager.yaml"
    )

    # ---------- Declare launch arguments ----------

    launch_args: list[DeclareLaunchArgument] = []

    # Specify the parameter configuration file path for the controller manager
    controller_manager_config_path_arg = DeclareLaunchArgument(
        "controller_manager_config_path",
        default_value=TextSubstitution(text=default_controller_manager_config_path),
        description="(string) Path to the parameter configuration file for the controller manager.",
    )
    launch_args.append(controller_manager_config_path_arg)

    # ---------- Declare executables ----------

    # Controller manager node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration(controller_manager_config_path_arg.name),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        # When the controller manager crashes, the entire launch file will shut down
        on_exit=Shutdown(),
    )

    # Joint state broadcaster (to update the robot state based on state measurements)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # ---------- Construct launch description ----------

    return LaunchDescription(
        [
            *launch_args,  # Unpack the launch arguments
            controller_manager,
            joint_state_broadcaster,
        ]
    )
