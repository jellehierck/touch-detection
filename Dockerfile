###########################################
# Dockerfile Summary
###########################################
# Purpose: Build a ROS 2 Humble container with the touch_detection_robot packages and all dependencies
# Context: ROS 2 touch detection under clock synchronization for the Nakama Robotics Lab
# Author: Jelle Hierck
#
# Major components:
# - Base image: some image which derives from ros:humble and builds Franka ROS 2 (e.g. franka_ros2:v0.1.15), set by BASE_IMAGE build argument 
# - Creates new workspace /touch_detection_ws
# - Installs franka_buttons_ros2 (https://github.com/jellehierck/franka_buttons_ros2)
# - Installs touch_detection_robot (TODO: Add link to repository)
# - Automatically sources new workspace whenever container is started
###########################################

# Obtain the base image that this image will be built on
# TODO: Perhaps we do not make this a build argument but instead hard-code the base image version
ARG BASE_IMAGE=franka_ros2:v0.1.15
FROM $BASE_IMAGE

# Execute allcommands with bash
SHELL ["/bin/bash", "-c"]

ARG USERNAME=user
USER $USERNAME

# Install APT dependencies as cache layer
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        python3-venv \
        ros-$ROS_DISTRO-plotjuggler-ros

# Set the workspace where the touch_detection_robot packages will be stored and built
ENV TOUCH_DETECTION_WS=/home/$USERNAME/touch_detection_ws

# Prepare a Python virtual environment for packages which need that later
WORKDIR $TOUCH_DETECTION_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && python3 -m venv .venv --system-site-packages --symlinks \
    && touch .venv/COLCON_IGNORE

# Import missing dependencies into the workspace src folder
WORKDIR $TOUCH_DETECTION_WS
COPY dependencies.repos dependencies.repos
RUN mkdir -p src \
    && vcs import src < dependencies.repos --recursive --skip-existing \
    # Remove dependencies.repos file as we don't need it anymore
    && rm dependencies.repos

# Install the dependencies of the repositories we just imported
WORKDIR $TOUCH_DETECTION_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source $FRANKA_WS/install/setup.bash \
    # Install rosdep dependencies from the imported repositories
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    # Remove cache to reduce image size
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

# Remove the built-in ruckig as we do not want interference with the newly built one
RUN sudo apt-get remove ros-humble-ruckig -y

# Build the dependency packages using the virtual environment
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source $FRANKA_WS/install/setup.bash \
    && source .venv/bin/activate \
    && python -m colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Copy this package's source packages to the src/ folder
WORKDIR $TOUCH_DETECTION_WS/src
COPY touch_detection_bringup/ touch_detection_bringup/
COPY linear_velocity_controller/ linear_velocity_controller/
COPY linear_velocity_controller_interfaces/ linear_velocity_controller_interfaces/

# Install all missing Python dependencies as specified in their requirements.txt files
WORKDIR $TOUCH_DETECTION_WS
RUN source .venv/bin/activate \
    && find ./src -name "requirement*.txt" -type f -exec pip3 install -r '{}' ';'

# Install all ROS dependencies too
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source $FRANKA_WS/install/setup.bash \
    # Install rosdep dependencies from the imported repositories
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    # Remove cache to reduce image size
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

# Build all packages with the virtual environment sourced
WORKDIR $TOUCH_DETECTION_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source $FRANKA_WS/install/setup.bash \
    && source .venv/bin/activate \
    # If more packages need the virtual environment build, add them at the end of the following line
    && python -m colcon build --symlink-install --packages-up-to \
        linear_velocity_controller \
        touch_detection_bringup

# Extend the default ROS entrypoint script so that the new workspace is also sourced by default
RUN sudo sed --in-place \
    --expression '$isource "$TOUCH_DETECTION_WS/install/setup.bash" --' \
    /ros_entrypoint.sh

# Add the Franka ROS 2 workspace to the .bashrc so that every shell has it opened by default
# Note that this is only necessary for docker exec -it ... commands, otherwise the ENTRYPOINT script already takes care of this  
RUN echo "source $TOUCH_DETECTION_WS/install/setup.bash" >> /home/$USERNAME/.bashrc

# Set the landing workspace when the container is started
ENV ROS2_WS=/home/$USERNAME/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS
