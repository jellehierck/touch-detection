from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description() -> LaunchDescription:
    """Bring up the Franka robot for the hand-eye calibration."""
    # ---------- Define common paths ----------

    # ---------- Declare launch arguments ----------

    launch_args = []

    # ---------- Declare executables ----------

    # linear_velocity_controller spawner
    linear_velocity_controller_launch = GroupAction(
        actions=[
            # Include the controller manager nodes
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        str(Path(get_package_share_directory("linear_velocity_controller")) / "launch" / "linear_velocity_controller.launch.py"),
                    ]
                ),
            ),
        ],
        forwarding=True,  # Make the launch arguments from the current launch file available inside this GroupAction
        scoped=True  # Any changes to launch arguments inside this GroupAction do not "spill" outside of the GroupAction
    )


    # Controller manager
    controller_manager_launch = GroupAction(
        actions=[
            # Define remappings which are applied to all nodes within this GroupAction
            SetRemap("joint_states", "franka/joint_states"),
            SetRemap("controller_manager/robot_description", "robot_description"),

            # Include the controller manager nodes
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        str(Path(get_package_share_directory("linear_velocity_controller")) / "launch" / "controller_manager.launch.py"),
                    ]
                ),
            ),
        ],
        forwarding=True,  # Make the launch arguments from the current launch file available inside this GroupAction
        scoped=True  # Any changes to launch arguments inside this GroupAction do not "spill" outside of the GroupAction
    )

    # Robot description
    robot_description_launch = GroupAction(
        actions=[
            # Include the controller manager nodes
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        str(Path(get_package_share_directory("linear_velocity_controller")) / "launch" / "robot_description.launch.py"),
                    ]
                ),
                launch_arguments={
                    # Due to a bug in ros2_launch, required launch arguments are passed explicitly as a workaround
                    # See https://github.com/ros2/launch/issues/749
                    "robot_ip": LaunchConfiguration("robot_ip"),
                }.items(),
            ),
        ],
        forwarding=True,  # Make the launch arguments from the current launch file available inside this GroupAction
        scoped=True  # Any changes to launch arguments inside this GroupAction do not "spill" outside of the GroupAction
    )

    # ---------- Construct launch description ----------

    return LaunchDescription(
        [
            *launch_args,  # Unpack the launch arguments
            controller_manager_launch,
            robot_description_launch,
            linear_velocity_controller_launch,
        ]
    )
