FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# 系統與 ROS 2 核心依賴
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
    ros-jazzy-topic-tools \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-grid-map \
    ros-jazzy-grid-map-core \
    ros-jazzy-grid-map-ros \
    ros-jazzy-grid-map-pcl \
    ros-jazzy-grid-map-msgs \
    ros-jazzy-nav2-msgs \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-eigen \
    ros-jazzy-filters \
    ros-jazzy-pinocchio \
    coinor-libipopt-dev \
    libglfw3-dev \
    libpcl-dev \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-clean \
    doxygen libeigen3-dev \
    libgl1-mesa-dri libglx-mesa0 libgl1 libegl1 \
    joystick jstest-gtk evtest \
    git cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

WORKDIR /ros2_ws
