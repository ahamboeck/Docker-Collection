FROM ubuntu:22.04

ARG WORKSPACE=dev_ws

WORKDIR /root/$WORKSPACE

# Set noninteractive mode for APT
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV QT_X11_NO_MITSHM=1
ENV EDITOR=nano
ENV XDG_RUNTIME_DIR=/tmp

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
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rqt-tf-tree \
    ros-humble-slam-toolbox \
    ros-humble-twist-mux \
    ros-humble-usb-cam \
    ros-humble-xacro \
    ros-humble-ament-clang-format \
    ros-humble-ament-clang-tidy \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    vim \
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

# Setup ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc

# Add useful aliases
RUN echo 'alias rosdi="rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y"' >> /root/.bashrc
RUN echo 'alias cbuild="colcon build --symlink-install"' >> /root/.bashrc
RUN echo 'alias zenoh="export RMW_IMPLEMENTATION=rmw_zenoh_cpp"' >> /root/.bashrc
RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /root/.bashrc

# Enable argument completion
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.bashrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.bashrc

WORKDIR /root/${WORKSPACE}/ros2_ws

CMD ["/bin/bash"]