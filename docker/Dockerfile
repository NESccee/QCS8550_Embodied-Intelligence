# ==============================================================================
# Dockerfile for Degeneracy-Aware Multi-Sensor Fusion SLAM & Dynamic Navigation
# Based on Ubuntu 22.04 LTS, ROS 2 Humble Desktop, and Gazebo Fortress
# ==============================================================================

FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install build dependencies, Eigen3, GTSAM, OpenCV, and LaTeX
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    libopencv-dev \
    libgtest-dev \
    texlive-latex-base \
    texlive-fonts-recommended \
    texlive-fonts-extra \
    texlive-latex-extra \
    texlive-science \
    latexmk \
    python3-matplotlib \
    python3-numpy \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws
COPY . /ros2_ws/src/SmartCar_Gazebo_ws

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws/src/SmartCar_Gazebo_ws && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/src/SmartCar_Gazebo_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash", "/ros2_ws/src/SmartCar_Gazebo_ws/scripts/reproduce_benchmarks.sh"]

