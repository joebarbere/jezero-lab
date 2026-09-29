# Phase 0: upstream osr_gazebo as published — ROS 2 Humble + Gazebo Classic 11.
# Build context is the repo root (see containers/build.sh).
FROM docker.io/osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Packages listed in osr_gazebo/README.md, plus teleop.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-humble-controller-manager \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-joint-trajectory-controller \
        ros-humble-robot-state-publisher \
        ros-humble-ros2-controllers \
        ros-humble-rviz2 \
        ros-humble-teleop-twist-keyboard \
        ros-humble-trajectory-msgs \
        ros-humble-velocity-controllers \
        ros-humble-xacro \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Copy the workspace: the osr-rover-code submodule plus osr_gz. osr_gazebo ships a
# COLCON_IGNORE upstream; drop it here, in the image, so the upstream package can
# still be run side by side with osr_gz for comparison.
WORKDIR /osr_ws
COPY ros_ws/src src
RUN rm -f src/osr-rover-code/ROS/osr_gazebo/COLCON_IGNORE

RUN apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src --rosdistro humble -y \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/humble/setup.sh && colcon build

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /osr_ws/install/setup.bash' >> /root/.bashrc
