# Phase 1: ROS 2 Jazzy + Gazebo Harmonic, running osr_gz.
# Build context is the repo root (see containers/build.sh).
FROM docker.io/osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros-gz \
        ros-jazzy-ros2-controllers \
        ros-jazzy-ros2controlcli \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-xacro \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Upstream osr_gazebo keeps its COLCON_IGNORE here: it needs Gazebo Classic,
# which Jazzy doesn't have. osr_gz replaces it.
WORKDIR /osr_ws
COPY ros_ws/src src

# rosdep doesn't honour COLCON_IGNORE, so name the packages rather than
# scanning all of src (osr_gazebo's gazebo_ros key can't resolve on Jazzy).
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --ignore-src --rosdistro jazzy -y --from-paths \
        src/osr_gz \
        src/osr-rover-code/ROS/osr_bringup \
        src/osr-rover-code/ROS/osr_control \
        src/osr-rover-code/ROS/osr_interfaces \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/jazzy/setup.sh && colcon build

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /osr_ws/install/setup.bash' >> /root/.bashrc
