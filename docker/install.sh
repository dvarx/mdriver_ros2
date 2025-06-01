#!/bin/bash

#install ROS2 Jazzy
export DEBIAN_FRONTEND=noninteractive
#install tzdata non-interactively
apt update && apt install sudo curl git -y
apt-get install tzdata -y
apt update && apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
apt install -y software-properties-common
add-apt-repository universe -y
apt update && apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
#install ros dev tools
echo "Installing ROS2 dev tools"
apt update && apt install -y ros-dev-tools
apt-get update
apt install -y ros-jazzy-ros-base

#create ROS2 workspace and check out sourcecode
source /opt/ros/jazzy/setup.bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/dvarx/mdriver_ros2.git

#install ROS2 dependencies
rosdep init
rosdep update
rosdep install --from-paths ./src --ignore-src -r -y
source /opt/ros/jazzy/setup.bash

#build the package
colcon build --packages-select mdriver