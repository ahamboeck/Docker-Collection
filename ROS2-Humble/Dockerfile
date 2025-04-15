FROM osrf/ros:humble-desktop-full

ARG WORKSPACE=humble_dev_ws

WORKDIR /root/$WORKSPACE

# Set noninteractive mode for APT
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV QT_X11_NO_MITSHM=1
ENV EDITOR=nano
ENV XDG_RUNTIME_DIR=/tmp

RUN apt-get update && apt-get install -y \
    cmake \
    curl \
    gazebo \
    libglu1-mesa-dev \
    nano \
    python3-pip \
    python3-pydantic \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \
    ros-humble-plotjuggler-ros \
    ros-humble-robot-state-publisher \
    ros-humble-ros2bag \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rqt-tf-tree \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-twist-mux \
    ros-humble-usb-cam \
    ros-humble-xacro \
    ros-humble-ament-clang-format \
    ros-humble-ament-clang-tidy \
    ruby-dev \
    rviz \
    tmux \
    wget \
    xorg-dev \
    zsh \
    gdb \
    valgrind \
    clang-format \
    python3-autopep8 \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py

RUN pip3 install setuptools==58.2.0

RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions

RUN gem install tmuxinator && \
    wget https://raw.githubusercontent.com/tmuxinator/tmuxinator/master/completion/tmuxinator.zsh -O /usr/local/share/zsh/site-functions/_tmuxinator

RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN git config --global --add safe.directory '*'

RUN echo "export DISABLE_AUTO_TITLE=true" >> /root/.zshrc
RUN echo 'LC_NUMERIC="en_US.UTF-8"' >> /root/.zshrc
RUN echo "source /opt/ros/humble/setup.zsh" >> /root/.zshrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.zshrc

RUN echo 'alias rosdi="rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y"' >> /root/.zshrc
RUN echo 'alias cbuild="colcon build --symlink-install"' >> /root/.zshrc
RUN echo 'alias ssetup="source ./install/local_setup.zsh"' >> /root/.zshrc
RUN echo 'alias zenoh="export RMW_IMPLEMENTATION=rmw_zenoh_cpp"' >> /root/.zshrc
RUN echo 'alias cyclone="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"' >> /root/.zshrc
RUN echo 'alias fastdds="export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"' >> /root/.zshrc
RUN echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> /root/.zshrc

RUN echo "autoload -U bashcompinit" >> /root/.zshrc
RUN echo "bashcompinit" >> /root/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc

# Keep root as the user and stay in /root
WORKDIR /root

CMD [ "tmuxinator", "start", "-p", "/root/.session.yml" ]