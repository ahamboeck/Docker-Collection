# Base image
FROM nvcr.io/nvidia/isaac-sim:4.5.0

ENV DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    wget \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN apt-get update && apt-get install -y locales \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && apt-get install -y software-properties-common \
 && add-apt-repository universe \
 && apt-get update && apt-get install -y gnupg2 curl lsb-release

# Remove conflicting sury.org packages and downgrade libbrotli1
RUN rm -f /etc/apt/sources.list.d/*sury* && \
    apt-get update && \
    apt-get install -y --allow-downgrades libbrotli1=1.0.9-2build6 && \
    apt-mark hold libbrotli1

# Install ROS2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /etc/apt/trusted.gpg.d/ros.gpg
RUN echo "deb [arch=amd64 signed-by=/etc/apt/trusted.gpg.d/ros.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# Install ROS2 Humble ros-base and dependencies
RUN apt-get update && apt-get install -y \
    libfreetype6-dev \
    libfontconfig1-dev \
    libxft-dev \
    ros-humble-desktop \
    ros-dev-tools \
    ros-humble-rviz2 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Environment variables
ENV ACCEPT_EULA=Y
ENV PRIVACY_CONSENT=Y
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

# Volumes
VOLUME ["/isaac-sim/kit/cache", "/root/.cache/ov", "/root/.cache/pip", "/root/.cache/nvidia/GLCache", "/root/.nv/ComputeCache", "/root/.nvidia-omniverse/logs", "/root/.local/share/ov/data", "/root/Documents"]

# Source ROS2 in bash shell
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Default CMD to launch Isaac Sim with GUI
CMD ["./runapp.sh", "-v"]
