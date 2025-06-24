###########################################
# Dockerfile Summary
###########################################
# Purpose: Build a ROS2 Humble container with MoveIt2, libfranka v0.13.6 and franka_ros2 v0.1.15
# Context: ROS2 robotic simulation and manipulation (Unity/MoveIt2/Gazebo integration).
#
# Major Components:
# - Base image: osrf/ros:humble-desktop
# - ROS2 Humble installation with MoveIt2, controllers, and visualization tools
# - Python tools for ROS2 linters and code quality
# - Manual build and installation of libfranka 0.13.6
# - Manual build and installation of franka_ros2 v0.1.15 and franka_description 0.4.0
# - Switch to a non-root user for development security
###########################################

# Use full ROS 2 Humble desktop image
FROM osrf/ros:humble-desktop

# Use barebones ROS 2 Humble image
# FROM ros:humble

# Execute all commands with bash instead of sh (the default)
SHELL ["/bin/bash", "-c"]

# Install basic dependencies and sudo
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    curl \
    gdb \
    git \
    nano \
    xauth \
    openssh-client \
    bash-completion \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-argcomplete \
    python3-venv \
    ros-$ROS_DISTRO-plotjuggler-ros \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Arguments for custom user
ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

# Set permissions for all files in the user's home folder
RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME

# Switch to non-root user
USER $USERNAME

# Setup Franka ROS2 workspace
ENV FRANKA_WS=/home/$USERNAME/franka_ros2_ws

# Clone franka_ros2 v0.1.15
WORKDIR $FRANKA_WS
COPY dependencies.repos ./dependencies.repos
RUN mkdir src \
    && vcs import src < dependencies.repos --recursive --skip-existing \
    # Remove dependencies.repos file again as we no longer need it
    && rm dependencies.repos

# Remove all folders except franka_msgs
WORKDIR $FRANKA_WS/src
RUN mv ./franka_ros2/franka_msgs/ ./franka_msgs/ \
    && rm -rf ./franka_ros2/

# Build all franka_ros2 stuff
WORKDIR $FRANKA_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Extend the default ROS entrypoint script (included in ROS base image) with a statement to source the Franka workspace
# Example taken from https://hub.docker.com/_/ros/
RUN sudo sed --in-place \
    --expression '$isource "$FRANKA_WS/install/setup.bash" --' \
    /ros_entrypoint.sh

# Add the Franka ROS 2 workspace to the .bashrc so that every shell has it opened by default
# Note that this is only necessary for docker exec -it ... commands, otherwise the ENTRYPOINT script already takes care of this  
RUN echo "source $FRANKA_WS/install/setup.bash" >> /home/$USERNAME/.bashrc

# ---------------------------------------------------------------------------------------------------------------

# # Set the workspace where the touch_detection_robot packages will be stored and built
# ENV TOUCH_DETECTION_WS=/home/$USERNAME/touch_detection_ws

# # Prepare a Python virtual environment for packages which need that later
# WORKDIR $TOUCH_DETECTION_WS
# RUN source /opt/ros/$ROS_DISTRO/setup.bash \
#     && python3 -m venv .venv --system-site-packages --symlinks \
#     && touch .venv/COLCON_IGNORE

# # Copy this package's source packages to the src/ folder
# WORKDIR $TOUCH_DETECTION_WS/src
# # TODO: If we need any code running in this folder, copy the folder here

# # Install all missing Python dependencies as specified in their requirements.txt files
# WORKDIR $TOUCH_DETECTION_WS
# RUN source .venv/bin/activate \
#     && find ./src -name "requirement*.txt" -type f -exec pip3 install -r '{}' ';'

# # # Install all ROS dependencies too
# # RUN source /opt/ros/$ROS_DISTRO/setup.bash \
# #     && source $FRANKA_WS/install/setup.bash \
# #     # Install rosdep dependencies from the imported repositories
# #     && sudo apt-get update \
# #     && rosdep update \
# #     && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
# #     # Remove cache to reduce image size
# #     && sudo apt-get clean \
# #     && sudo rm -rf /var/lib/apt/lists/*

# # Build all packages with the virtual environment sourced
# WORKDIR $TOUCH_DETECTION_WS
# RUN source /opt/ros/$ROS_DISTRO/setup.bash \
#     && source $FRANKA_WS/install/setup.bash \
#     && source .venv/bin/activate \
#     # If more packages need the virtual environment build, add them at the end of the following line
#     && python -m colcon build --symlink-install 
#         # --packages-up-to \
#         #     # TODO: Add packages to build

# # Extend the default ROS entrypoint script so that the new workspace is also sourced by default
# RUN sudo sed --in-place \
#     --expression '$isource "$TOUCH_DETECTION_WS/install/setup.bash" --' \
#     /ros_entrypoint.sh

# # Add the touch detection workspace to the .bashrc so that every shell has it opened by default
# # Note that this is only necessary for docker exec -it ... commands, otherwise the ENTRYPOINT script already takes care of this  
# RUN echo "source $TOUCH_DETECTION_WS/install/setup.bash" >> /home/$USERNAME/.bashrc

# Set the landing workspace when the container is started
ENV ROS2_WS=/home/$USERNAME/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS
