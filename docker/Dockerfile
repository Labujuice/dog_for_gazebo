FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# 系統依賴
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-clean \
    doxygen libeigen3-dev \
    libgl1-mesa-dri libglx-mesa0 libgl1 libegl1 \
    joystick jstest-gtk evtest \
    git cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

# Pinocchio (quad-sdk 動力學依賴)
RUN apt-get update && apt-get install -y \
    ros-jazzy-pinocchio \
    && rm -rf /var/lib/apt/lists/*

ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

WORKDIR /ros2_ws
