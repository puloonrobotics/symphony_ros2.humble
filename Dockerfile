FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
  curl \
  gnupg2 \
  lsb-release \
  sudo \
  software-properties-common \
  wget \
  && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble + MoveIt / ros2_control / Gazebo bridge deps
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y ros-dev-tools \
  && apt-get install -y --no-install-recommends \
  ros-humble-desktop \
  ros-humble-xacro \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-moveit-visual-tools \
  ros-humble-moveit-resources \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-planners-ompl \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-ros-perception \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-state-publisher-gui \
  ros-humble-joint-trajectory-controller \
  ros-humble-rviz-visual-tools \
  ros-humble-geometric-shapes \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gz \
  python3-argcomplete \
  python3-rosdep \
  && rosdep init \
  && rosdep update \
  && apt-get install -y \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  && colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
  && colcon mixin update default \
  && apt-get install -y python3-vcstool \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
  && apt-get install -y curl lsb-release gnupg \
  && curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update \
  && apt-get install -y gz-fortress \
  && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

SHELL ["/bin/bash", "-c"]

# symphony_driver needs system Puloon RTPRI (find_package). Install SDK into the
# image or mount a prefix and set CMAKE_PREFIX_PATH before colcon build.
RUN mkdir -p /root/symphony_ws/src
COPY ./src /root/symphony_ws/src/

# Without Puloon RTPRI on CMAKE_PREFIX_PATH, skip symphony_driver.
# After installing the SDK: colcon build --packages-select symphony_driver
RUN cd /root/symphony_ws \
  && . /opt/ros/humble/setup.bash \
  && colcon build --packages-skip symphony_driver

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ENV DEBIAN_FRONTEND=
ENV GZ_IP=127.0.0.1
