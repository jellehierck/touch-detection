from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """Bring up the Franka robot for the hand-eye calibration."""
    # ---------- Define common paths ----------

    # ---------- Declare launch arguments ----------

    launch_args = []

    use_plotjuggler_arg = DeclareLaunchArgument(
        "use_plotjuggler",
        default_value="true",
        description="(bool) Whether to start PlotJuggler.",
        choices=["true", "false"],
    )
    launch_args.append(use_plotjuggler_arg)

    # Redeclare the robot ip argument because we also need it for other nodes (therefore we cannot just include the
    # argument from Franka's launch file).
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        description="(string) Hostname or IP address of the robot.",
    )
    launch_args.append(robot_ip_arg)

    # Redeclare the arm ID argument because we want to add a default option.
    arm_id_arg = DeclareLaunchArgument(
        "arm_id",
        default_value="fr3",
        description="(string) ID of the type of arm used.",
        choices=["fer", "fr3", "fp3"],
    )
    launch_args.append(arm_id_arg)

    # Redeclare the rviz argument because we want to control rviz in this launch file instead of Franka's launch file.
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="(bool) Whether to start RViz2.",
        choices=["true", "false"],
    )
    launch_args.append(use_rviz_arg)

    # ---------- Declare executables ----------

    # Include the Franka bringup file to start all state reading nodes but not any controllers. This will allow manual
    # movement of the robot.

    # Note: we wrap this in a GroupAction to make sure that any overrides to launch arguments in the included launch
    # file do not also override the launch arguments in the outer launch file.
    # More specifically: this is necessary because we override argument "use_rviz" only in the included launch file. If
    # scoped=False, the overridden "use_rviz" value would also be set in the outer launch file.
    franka_bringup_base = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        str(Path(get_package_share_directory("franka_bringup")) / "launch" / "franka.launch.py"),
                    ]
                ),
                launch_arguments={
                    # We override some arguments in the Franka launch file
                    "robot_ip": LaunchConfiguration(robot_ip_arg.name),
                    "arm_id": LaunchConfiguration(arm_id_arg.name),
                    "use_rviz": "false",  # Do not start RViz from the Franka file, we use our own configuration
                }.items(),
            ),
        ],
        # Changes to launch configurations set inside the GroupAction do not spill into the rest of the launch file.
        # This prevents launch arguments with the same name to be overridden in the outer launch file.
        scoped=True,
        # We want to forward all launch configurations from the outer file into the included file.
        forwarding=True,
    )

    # linear_velocity_controller spawner
    linear_velocity_controller_launch = GroupAction(
        actions=[
            # Include the controller manager nodes
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        str(
                            Path(get_package_share_directory("linear_velocity_controller"))
                            / "launch"
                            / "linear_velocity_controller.launch.py"
                        ),
                    ]
                ),
                launch_arguments={
                    "linear_velocity_controller_inactive": "true",
                }.items(),
            ),
        ],
        forwarding=True,  # Make the launch arguments from the current launch file available inside this GroupAction
        scoped=True,  # Any changes to launch arguments inside this GroupAction do not "spill" outside the GroupAction
    )

    # PlotJuggler for data visuazliaton
    plotjuggler_launch = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        str(
                            Path(get_package_share_directory("touch_detection_bringup"))
                            / "launch"
                            / "plotjuggler.launch.py"
                        ),
                    ]
                ),
            ),
        ],
        # Changes to launch configurations set inside the GroupAction do not spill into the rest of the launch file.
        # This prevents launch arguments with the same name to be overridden in the outer launch file.
        scoped=True,
        # We want to forward all launch configurations from the outer file into the included file.
        forwarding=True,
        # Only start this launch file if the parameter is set
        condition=IfCondition(LaunchConfiguration(use_plotjuggler_arg.name)),
    )

    # RViz2 for visualization of the robot. We launch this instead of from Franka's launch file since we want our own
    # configuration.
    rviz_launch = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        str(Path(get_package_share_directory("touch_detection_bringup")) / "launch" / "rviz.launch.py"),
                    ]
                ),
            ),
        ],
        # Changes to launch configurations set inside the GroupAction do not spill into the rest of the launch file.
        # This prevents launch arguments with the same name to be overridden in the outer launch file.
        scoped=True,
        # We want to forward all launch configurations from the outer file into the included file.
        forwarding=True,
        # Only start this launch file if the parameter is set
        condition=IfCondition(LaunchConfiguration(use_rviz_arg.name)),
    )

    # ---------- Construct launch description ----------

    return LaunchDescription(
        [
            *launch_args,  # Unpack the launch arguments
            franka_bringup_base,
            linear_velocity_controller_launch,
            plotjuggler_launch,
            rviz_launch,
        ]
    )
