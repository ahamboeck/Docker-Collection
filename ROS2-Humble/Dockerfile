FROM ubuntu:22.04

ARG WORKSPACE=dev_ws
# Add build argument for RMW implementation (default: rmw_cyclonedds_cpp)
ARG RMW_IMPLEMENTATION=rmw_zenoh_cpp

WORKDIR /root/$WORKSPACE

# Set noninteractive mode for APT
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV QT_X11_NO_MITSHM=1
ENV EDITOR=nano
ENV XDG_RUNTIME_DIR=/tmp
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    wget \
    software-properties-common

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble and additional packages
RUN apt-get update && apt-get install -y \
    cmake \
    gazebo \
    git \
    libglu1-mesa-dev \
    nano \
    python3-pip \
    python3-pydantic \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-argcomplete \
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
    ros-humble-rviz2 \
    ros-dev-tools \
    vim \
    tcpdump \
    iputils-ping \
    iputils-tracepath \
    traceroute \
    netcat \
    net-tools \
    bash-completion \
    gdb \
    valgrind \
    clang-format \
    python3-autopep8 \
    xorg-dev

# Initialize rosdep
RUN rosdep init && rosdep update

RUN pip3 install setuptools==58.2.0

RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN git config --global --add safe.directory '*'

# Copy Cyclone DDS XML config (must be present in build context)
COPY cyclonedds.xml /root/cyclonedds.xml

# Copy Zenoh config files (must be present in build context)
COPY MY_ZENOH_ROUTER_CONFIG.json5 /root/MY_ZENOH_ROUTER_CONFIG.json5
COPY MY_ZENOH_SESSION_CONFIG.json5 /root/MY_ZENOH_SESSION_CONFIG.json5

# --- Build rmw_zenoh from source ---
# Create workspace and clone rmw_zenoh
RUN mkdir -p /root/ws_rmw_zenoh/src && \
    git clone --branch humble https://github.com/ros2/rmw_zenoh.git /root/ws_rmw_zenoh/src/rmw_zenoh

# Install dependencies
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths /root/ws_rmw_zenoh/src/ --ignore-src --rosdistro humble -y

# Build rmw_zenoh
RUN . /opt/ros/humble/setup.sh && \
    cd /root/ws_rmw_zenoh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup ROS environment and RMW implementation in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc && \
    echo "source /root/ws_rmw_zenoh/install/setup.bash" >> /root/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}' >> /root/.bashrc && \
    echo 'if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedx_cpp" ]; then export CYCLONEDDS_URI=file:///root/cyclonedds.xml; fi' >> /root/.bashrc && \
    echo 'if [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ]; then export ZENOH_ROUTER_CONFIG_URI=/root/MY_ZENOH_ROUTER_CONFIG.json5; export ZENOH_SESSION_CONFIG=/root/MY_ZENOH_SESSION_CONFIG.json5; fi' >> /root/.bashrc && \
    echo 'alias rosdi="rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y"' >> /root/.bashrc && \
    echo 'alias cbuild="colcon build --symlink-install"' >> /root/.bashrc && \
    echo 'alias zenoh="export RMW_IMPLEMENTATION=rmw_zenoh_cpp"' >> /root/.bashrc && \
    echo 'eval \"$(register-python-argcomplete3 ros2)\"' >> /root/.bashrc && \
    echo 'eval \"$(register-python-argcomplete3 colcon)\"' >> /root/.bashrc

WORKDIR /root/${WORKSPACE}/ros2_ws

CMD ["/bin/bash"]