# ROS 2 Jazzy on Ubuntu 24.04 — Full AI & Robotics Stack
FROM ubuntu:24.04

ARG WORKSPACE=dev_ws
ARG RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENV DEBIAN_FRONTEND=noninteractive \
    WORKSPACE=${WORKSPACE} \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
    QT_X11_NO_MITSHM=1 \
    EDITOR=nano \
    # Fix for SSL mismatches in Pip/Conda wheels
    LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libssl.so.3 \
    # Ensures Rerun uses the X11-friendly GL backend
    WGPU_BACKEND=gl

# 1. System Setup
RUN chmod 1777 /tmp && mkdir -p /tmp/xdg && chmod 700 /tmp/xdg
ENV XDG_RUNTIME_DIR=/tmp/xdg

# 2. Base Tools + Modern GUI Libraries (Noble specific)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl wget gnupg2 lsb-release \
      build-essential cmake git nano bash-completion \
      mesa-utils libgl1-mesa-dri libglu1-mesa-dev xorg-dev \
      # Added for Rerun/Qt6 stability in Ubuntu 24.04:
      libxcb-cursor0 libxkbcommon-x11-0 libfontconfig1 libegl1 libvulkan1 \
      # Your original debug/network tools:
      tcpdump iputils-ping iputils-tracepath traceroute netcat-openbsd net-tools \
      gdb valgrind clang-format vim trash-cli python3-pip python3-dev \
    && rm -rf /var/lib/apt/lists/*

# 3. ROS 2 Repo Setup
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list

# 4. Core ROS 2 Bits (Your original package list)
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

RUN apt-get remove -y python3-matplotlib

# 5. Blackwell-Optimized AI Stack (DualMap Requirements)
# Added --ignore-installed to bypass the Debian kiwisolver/numpy stubs
RUN pip install --break-system-packages --ignore-installed \
    "numpy<2.0" \
    "setuptools<80" \
    "matplotlib<3.10" \
    torch torchvision torchaudio \
    supervision==0.25.1 \
    rerun-sdk==0.22.1 \
    ultralytics==8.3.103 \
    open3d faiss-cpu tyro open_clip_torch wandb h5py openai hydra-core \
    distinctipy dill imageio natsort kornia record3d==1.4.1 pyliblzfse \
    pypng tabulate pympler plyfile \
    git+https://github.com/apple/ml-mobileclip.git \
    git+https://github.com/ultralytics/CLIP.git \
    langchain \
    uvicorn \
    fastapi \
    langchain-ollama

RUN rosdep init || true && rosdep update || true
RUN git config --global --add safe.directory '*'

# 6. Configs & Convenience (Your original shell logic)
COPY cyclonedds.xml /root/cyclonedds.xml
COPY MY_ZENOH_ROUTER_CONFIG.json5 /root/MY_ZENOH_ROUTER_CONFIG.json5
COPY MY_ZENOH_SESSION_CONFIG.json5 /root/MY_ZENOH_SESSION_CONFIG.json5

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}' >> /root/.bashrc && \
    echo 'if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then export CYCLONEDDS_URI=file:///root/cyclonedds.xml; fi' >> /root/.bashrc && \
    echo 'eval "$(register-python-argcomplete ros2)"' >> /root/.bashrc && \
    echo 'eval "$(register-python-argcomplete colcon)"' >> /root/.bashrc && \
    echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> /root/.bashrc && \
    echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}' >> /root/.bashrc && \
    echo 'export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libssl.so.3' >> /root/.bashrc && \
    echo 'export WGPU_BACKEND=gl' >> /root/.bashrc && \
    echo 'mkdir -p /root/${WORKSPACE} && cd /root/${WORKSPACE}' >> /root/.bashrc

WORKDIR /root/${WORKSPACE}
CMD ["/bin/bash"]