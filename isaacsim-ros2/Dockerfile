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
    ros-humble-desktop-full \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \
    ros-humble-plotjuggler-ros \
    ros-humble-robot-state-publisher \
    ros-humble-ros2bag \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rqt-tf-tree \
    ros-humble-slam-toolbox \
    ros-humble-twist-mux \
    ros-humble-usb-cam \
    ros-humble-xacro \
    ros-humble-ament-clang-format \
    ros-humble-ament-clang-tidy \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-dev-tools \
    ros-humble-rviz2 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# --- Build rmw_zenoh from source ---
# Create workspace and clone rmw_zenoh
RUN mkdir -p /root/ws_rmw_zenoh/src && \
    git clone --branch humble https://github.com/ros2/rmw_zenoh.git /root/ws_rmw_zenoh/src/rmw_zenoh

# Install dependencies
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep install --from-paths /root/ws_rmw_zenoh/src --ignore-src --rosdistro humble -y

# Build rmw_zenoh
RUN . /opt/ros/humble/setup.sh && \
    cd /root/ws_rmw_zenoh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace in .bashrc (after sourcing ROS)
RUN echo "source /root/ws_rmw_zenoh/install/setup.bash" >> /root/.bashrc

# Environment variables
ENV ACCEPT_EULA=Y
ENV PRIVACY_CONSENT=Y
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

# Add build argument for RMW implementation (default: rmw_cyclonedds_cpp)
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}

# Volumes
VOLUME ["/isaac-sim/kit/cache", "/root/.cache/ov", "/root/.cache/pip", "/root/.cache/nvidia/GLCache", "/root/.nv/ComputeCache", "/root/.nvidia-omniverse/logs", "/root/.local/share/ov/data", "/root/Documents"]

# Copy Cyclone DDS XML config (must be present in build context)
COPY cyclonedds.xml /root/cyclonedds.xml

# Copy Zenoh config files (must be present in build context)
COPY MY_ZENOH_ROUTER_CONFIG.json5 /root/MY_ZENOH_ROUTER_CONFIG.json5
COPY MY_ZENOH_SESSION_CONFIG.json5 /root/MY_ZENOH_SESSION_CONFIG.json5

# Set Cyclone DDS or Zenoh config file if selected
RUN echo 'if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then export CYCLONEDDS_URI=file:///root/cyclonedds.xml; fi' >> ~/.bashrc && \
    echo 'if [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ]; then export ZENOH_ROUTER_CONFIG_URI=/root/MY_ZENOH_ROUTER_CONFIG.json5; export ZENOH_SESSION_CONFIG=/root/MY_ZENOH_SESSION_CONFIG.json5; fi' >> ~/.bashrc

# Source ROS2 in bash shell and set RMW in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" >> ~/.bashrc

# Default CMD to launch Isaac Sim with GUI
CMD ["./runapp.sh", "-v"]
