# ============================================================================
# Stage 1: Base - ROS2 Humble + Dependencies
# ============================================================================
FROM ros:humble-ros-base-jammy AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# From documentation: Dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-turtle-tf2-py \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joy-linux \
    ros-${ROS_DISTRO}-urg-node \
    ros-${ROS_DISTRO}-urg-node-msgs \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    ros-${ROS_DISTRO}-ros2bag \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-smach \
    ros-${ROS_DISTRO}-smach-ros \
    ros-${ROS_DISTRO}-py-trees \
    ros-${ROS_DISTRO}-py-trees-ros \
    ros-${ROS_DISTRO}-py-trees-ros-interfaces \
    xcb \
    && rm -rf /var/lib/apt/lists/*

# From documentation: pip install
RUN pip3 install --no-cache-dir --upgrade sphinx docutils transforms3d numpy matplotlib

# ============================================================================
# Stage 2: Institution - HBRS-AMR packages pre-built
# ============================================================================
FROM base AS institution

WORKDIR /root/ros2_ws/src

# From documentation: Clone Robile and robile_description first
RUN git clone -b ros2 https://github.com/HBRS-AMR/Robile.git \
    && git clone -b ros2 https://github.com/HBRS-AMR/robile_description.git

# Build first batch (robile_gazebo depends on robile_description)
WORKDIR /root/ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

# From documentation: Clone remaining packages
WORKDIR /root/ros2_ws/src
RUN git clone -b ros2 https://github.com/HBRS-AMR/robile_gazebo.git \
    && git clone -b ros2 https://github.com/HBRS-AMR/robile_navigation.git \
    && git clone https://github.com/HBRS-AMR/robile_interfaces.git \
    && git clone -b ros2 https://github.com/ros/executive_smach.git

# Build all packages
WORKDIR /root/ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

# ============================================================================
# Stage 3: Development - GUI tools
# ============================================================================
FROM institution AS dev

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-rviz2 \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /root/ros2_ws/install/setup.bash' >> /root/.bashrc \
    && echo '[ -f /root/ws/install/setup.bash ] && source /root/ws/install/setup.bash' >> /root/.bashrc

ENV GAZEBO_MODEL_PATH=/root/ros2_ws/src/robile_gazebo/models

WORKDIR /root/ws
CMD ["bash"]

# ============================================================================
# Stage 4: Production/CI - Minimal, no GUI
# ============================================================================
FROM institution AS prod

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /root/ros2_ws/install/setup.bash' >> /root/.bashrc \
    && echo '[ -f /root/ws/install/setup.bash ] && source /root/ws/install/setup.bash' >> /root/.bashrc

WORKDIR /root/ws
CMD ["bash"]