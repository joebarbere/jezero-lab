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
        gdal-bin \
        python3-gdal \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic's Python bindings (gz.sim8, gz.transport13) for the Gym
# environment. ROS's gz vendor packages don't ship them, so they come from the
# OSRF repo, which installs a second, system copy of Harmonic under /usr. The
# ROS image's LD_LIBRARY_PATH points at the vendor copy, so jezero_env runs with
# LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu (see jezero_env/run.sh) and never
# mixes the two.
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        python3-gz-sim8 \
        libgz-sim8-plugins \
        python3-gz-transport13 \
        libgz-physics7-dartsim \
        python3-pip \
    && pip3 install --break-system-packages --no-cache-dir gymnasium==1.2.0 \
    && rm -rf /var/lib/apt/lists/* \
    # gz-sim asks for "gz-physics-dartsim-plugin", i.e. the unversioned alias
    # libgz-physics-dartsim-plugin.so, which only the -dev package provides.
    # Link it (as ROS's vendor copy does) rather than pull in the headers.
    && cd /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins \
    && ln -s libgz-physics7-dartsim-plugin.so.7 libgz-physics7-dartsim-plugin.so \
    && ln -s libgz-physics7-dartsim-plugin.so libgz-physics-dartsim-plugin.so

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
