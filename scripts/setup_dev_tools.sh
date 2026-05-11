#!/bin/bash
# Build tools, Python hardware libraries, and CLI utilities used across phases.
set -e

sudo apt-get install -y \
    build-essential cmake pkg-config ccache \
    python3-pip python3-venv python3-dev python3-setuptools python3-wheel \
    python3-smbus python3-serial python3-spidev python3-rpi.gpio \
    python3-yaml python3-opencv python3-numpy python3-scipy \
    git vim nano htop tmux screen tree jq bc less curl wget \
    v4l-utils i2c-tools usbutils lshw lsof pciutils \
    net-tools iputils-ping traceroute dnsutils \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
