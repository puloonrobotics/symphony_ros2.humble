# Symphony ROS2

## Overview

ROS2(Humble) packages for controlling **PULOON Robotics's Symphony** collaborative robots (cobots): drivers, descriptions, controllers, MoveIt, and Gazebo.

## Installation

### Puloon RTPRI

`symphony_driver` needs the Puloon RTPRI SDK (`find_package(puloon_rtpri)`).

If the SDK is not on the default CMake path, add its prefix to `CMAKE_PREFIX_PATH` before building. A local install in this tree lives at `.rtpri-install`. Do not bake the SDK into the Docker image.

### apt

```bash
sudo apt install -y python3-rosdep python3-colcon-common-extensions python3-colcon-mixin python3-vcstool
sudo rosdep init   # once
rosdep update
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

### ROS packages

```bash
sudo apt install -y \
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
  ros-humble-forward-command-controller \
  ros-humble-force-torque-sensor-broadcaster \
  ros-humble-pose-broadcaster \
  ros-humble-rviz-visual-tools \
  ros-humble-geometric-shapes \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gz
```

### Build

```bash
source /opt/ros/humble/setup.bash
cd /path/to/symphony_ros2
export CMAKE_PREFIX_PATH="$PWD/.rtpri-install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"
source install/setup.bash
```

Skip the `CMAKE_PREFIX_PATH` line when RTPRI is already installed on the default search path.

## Usage

Do not run a real-robot bringup and Gazebo at the same time.

### Real robot

```bash
ros2 launch symphony_driver symphony_bringup.launch.py robot_ip:=192.168.0.234 symphony_type:=symphony5
```

The robot should be in External mode. Change `robot_ip` for your network. Only one bringup may be running.

### Description (RViz)

```bash
ros2 launch symphony_description symphony_display.launch.py symphony_type:=symphony5
```

### MoveIt

```bash
ros2 launch symphony_moveit symphony_moveit.launch.py symphony_type:=symphony5
```

### Gazebo

```bash
ros2 launch symphony_gazebo symphony_gazebo.launch.py symphony_type:=symphony5
```

This launch is the robot only. For the plate and blocks:

```bash
ros2 launch symphony_pnp pnp.launch.py symphony_type:=symphony5
```

### MoveIt + Gazebo

```bash
ros2 launch symphony_moveit symphony_moveit_gazebo.launch.py symphony_type:=symphony5
```

### MoveIt on the real robot

This launch starts bringup itself. Do not start `symphony_bringup.launch.py` as well.

```bash
ros2 launch symphony_moveit symphony_moveit_real.launch.py robot_ip:=192.168.0.234 symphony_type:=symphony5
```

## Examples

Start bringup first, then:

```bash
ros2 run symphony_driver servoq_example.py
```

Other scripts are in `src/symphony_driver/examples`. Do not use the example launch files while bringup is already running.

## Docker

```bash
cd /path/to/symphony_ros2
docker build -t symphony-ros2 .
xhost +local:docker
docker run -d --name symphony-ros2 -it \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -v "$PWD/.rtpri-install":/opt/rtpri:ro \
  -e DISPLAY=$DISPLAY \
  -e CMAKE_PREFIX_PATH=/opt/rtpri \
  -e QT_X11_NO_MITSHM=1 \
  --net host \
  symphony-ros2 \
  /bin/bash
docker exec -it symphony-ros2 bash
```

```bash
source /opt/ros/humble/setup.bash
cd /root/symphony_ws
colcon build --packages-select symphony_driver --cmake-args -DCMAKE_PREFIX_PATH=/opt/rtpri
source install/setup.bash
```

