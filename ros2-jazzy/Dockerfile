# ROS 2 Jazzy on Ubuntu 24.04 — minimal base, no pip installs
FROM ubuntu:24.04

ARG WORKSPACE=dev_ws
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # switch at runtime if you like

ENV DEBIAN_FRONTEND=noninteractive \
    WORKSPACE=${WORKSPACE} \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} \
    NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
    NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics \
    QT_X11_NO_MITSHM=1 \
    EDITOR=nano

# Keep /tmp sane and give XDG a private dir (helps apt + some GUI tools)
RUN chmod 1777 /tmp && mkdir -p /tmp/xdg && chmod 700 /tmp/xdg
ENV XDG_RUNTIME_DIR=/tmp/xdg

# Base tools (no pip here)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl wget gnupg2 lsb-release \
      build-essential cmake git nano bash-completion \
      mesa-utils libgl1-mesa-dri libglu1-mesa-dev xorg-dev \
      tcpdump iputils-ping iputils-tracepath traceroute netcat-openbsd net-tools gdb valgrind clang-format \
      vim trash-cli \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 apt repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list

# Core ROS 2 bits (keep it tight; add more as you actually need)
RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
      ros-jazzy-desktop \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-rmw-zenoh-cpp \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
      ros-jazzy-slam-toolbox \
      ros-jazzy-depth-image-proc ros-jazzy-image-geometry ros-jazzy-pointcloud-to-laserscan \
      ros-jazzy-rviz2 \
      ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher \
      ros-jazzy-robot-localization \
      ros-jazzy-ros2bag ros-jazzy-rosbag2-storage-default-plugins \
      ros-jazzy-rqt-tf-tree \
      ros-jazzy-xacro \
      ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# rosdep minimal init; you can `rosdep update` at runtime
RUN rosdep init || true && rosdep update || true

# Git safety for bind mounts
RUN git config --global --add safe.directory '*'

# Optional config files (provide them or remove these lines)
COPY cyclonedds.xml /root/cyclonedds.xml
# If you don’t use Zenoh, you can delete these two:
COPY MY_ZENOH_ROUTER_CONFIG.json5 /root/MY_ZENOH_ROUTER_CONFIG.json5
COPY MY_ZENOH_SESSION_CONFIG.json5 /root/MY_ZENOH_SESSION_CONFIG.json5

# Shell convenience
RUN echo 'source /opt/ros/jazzy/setup.bash'                                   >> /root/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}'                     >> /root/.bashrc && \
    echo 'if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then export CYCLONEDDS_URI=file:///root/cyclonedds.xml; fi' >> /root/.bashrc && \
    echo 'eval "$(register-python-argcomplete ros2)"'                          >> /root/.bashrc && \
    echo 'eval "$(register-python-argcomplete colcon)"'                        >> /root/.bashrc && \
    echo 'export RCUTILS_COLORIZED_OUTPUT=1'                                   >> /root/.bashrc && \
    echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}'                            >> /root/.bashrc && \
    echo 'mkdir -p /root/${WORKSPACE} && cd /root/${WORKSPACE}'                >> /root/.bashrc

# Workspace
WORKDIR /root/${WORKSPACE}

CMD ["/bin/bash"]
