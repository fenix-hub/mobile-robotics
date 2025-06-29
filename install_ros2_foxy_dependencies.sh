#!/bin/bash
# install_ros2_foxy_dependencies.sh
# This script installs all required apt packages for the Ackermann ROS2 Foxy project

set -e

# Update package index
sudo apt update

# Install ROS 2 Foxy base packages
sudo apt install -y ros-foxy-desktop

# ROS 2 build tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# Gazebo Classic (for ROS2 Foxy)
sudo apt install -y gazebo11 libgazebo11-dev ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-ros2-control

# Navigation2 stack and dependencies
sudo apt install -y ros-foxy-nav2-bringup ros-foxy-slam-toolbox ros-foxy-nav2-map-server ros-foxy-nav2-amcl

# Teleop and visualization
sudo apt install -y ros-foxy-teleop-twist-keyboard ros-foxy-rviz2

# Control and message packages
sudo apt install -y ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui

# Sensor messages and drivers
sudo apt install -y ros-foxy-sensor-msgs ros-foxy-image-transport ros-foxy-cv-bridge

# Other useful tools
sudo apt install -y git wget curl

# Initialize rosdep
sudo rosdep init || true
rosdep update

echo "All required apt packages for the Ackermann ROS2 Foxy project have been installed."

