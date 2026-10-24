#!/bin/bash
# Install ROS2 Jazzy + the ROS packages the racecar driver depends on.
set -eo pipefail

# Bootstrap tools needed to add the ROS2 apt repo.
sudo apt-get update
sudo apt-get install -y -qq curl wget gnupg ca-certificates lsb-release

# The ROS apt source package. The newest release is looked up on GitHub; a
# failed lookup (rate limit, outage) uses the version known to work.
ROS_APT_SOURCE_FALLBACK=1.2.0
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    ROS_APT_SOURCE_VERSION=$(curl -fsS --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F 'tag_name' | head -1 | awk -F'"' '{print $4}' || true)
    if [ -z "$ROS_APT_SOURCE_VERSION" ]; then
        echo "GitHub lookup failed; using ros-apt-source ${ROS_APT_SOURCE_FALLBACK}."
        ROS_APT_SOURCE_VERSION="$ROS_APT_SOURCE_FALLBACK"
    fi
    DEB="/tmp/ros2-apt-source.deb"
    wget -q --tries=3 "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(lsb_release -cs)_all.deb" -O "$DEB"
    sudo dpkg -i "$DEB"
fi

sudo apt-get update

# Runtime: ros-base + the message/driver packages used by the racecar driver.
# Dev:     ros-dev-tools (ament_cmake et al.), colcon, rosdep.
# SLAM:    slam_toolbox for mapping; nav2 AMCL, map_server and lifecycle_manager
#          for localization on a saved map.
# Diag:    rviz2, rqt, foxglove_bridge for visualization on a remote laptop.
sudo apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-joy \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-tf2-tools \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-topic-tools \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-amcl \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-rqt-graph \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rqt-image-view \
    ros-jazzy-foxglove-bridge \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    nvme-cli \
    gdisk
