# Use official Ubuntu 22.04 base image
FROM ubuntu:22.04

# Set noninteractive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# Env setup
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Install system dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    git \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update

# Install dependencies
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    wget \ 
    software-properties-common \ 
    python3-pip \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    libpython3-dev \
    ros-humble-rclpy \
    ros-humble-ros-base \
    ros-humble-motion-capture-tracking \
    ros-dev-tools \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install -r requirements.txt

# Set up workspace
WORKDIR /ros_ws/src

# Initialize rosdep
RUN rosdep init && rosdep update

# Clone driver code
# RUN git clone https://github.com/alex-j-wang/crazyswarm2.git .
# RUN git submodule update --init --recursive
COPY . .

# Run install script and pass in the architecture
# RUN ARCH=$(dpkg --print-architecture) && echo "Building driver with $ARCH" && /ros_ws/src/install_spot_ros2.sh --$ARCH

# Build packages with Colcon
WORKDIR /ros_ws/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug