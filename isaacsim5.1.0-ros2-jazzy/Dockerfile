# Isaac Sim 5.1.0 + ROS 2 Jazzy (Ubuntu 24.04)
# Note: ROS apt repo uses HTTP here to avoid TLS cert-name-mismatch issues in some networks.
FROM ubuntu:24.04

ARG ISAACSIM_VERSION=5.1.0

ENV DEBIAN_FRONTEND=noninteractive \
    OMNI_KIT_ACCEPT_EULA=YES \
    ACCEPT_EULA=Y \
    PRIVACY_CONSENT=Y \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    TZ=Etc/UTC

# Base OS + graphics/Vulkan + tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl wget git unzip bzip2 locales tzdata sudo \
    software-properties-common gnupg2 gpg \
    libgl1 libegl1 libvulkan1 vulkan-tools \
    libx11-6 libxext6 libxrender1 libxrandr2 \
    libxi6 libxfixes3 libxcursor1 libxkbcommon0 libxcomposite1 libxdamage1 libxinerama1 \
    libxt6 libxmu6 libglu1-mesa \
    libnss3 libdrm2 libgbm1 libxshmfence1 libasound2t64 xdg-user-dirs zenity \
 && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8

# Grant the ubuntu user passwordless sudo
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Python 3.11 venv (Ubuntu 24.04 default is 3.12)
RUN add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && apt-get install -y --no-install-recommends \
      python3.11 python3.11-venv python3.11-dev && \
    rm -rf /var/lib/apt/lists/*

RUN python3.11 -m venv /opt/py311 && /opt/py311/bin/python -m ensurepip --upgrade
ENV PATH=/opt/py311/bin:$PATH

# Isaac Sim (pip)
RUN python -m pip install --no-cache-dir -U pip setuptools wheel && \
    python -m pip install --no-cache-dir "isaacsim[all,extscache]==${ISAACSIM_VERSION}" && \
    python -m pip install --no-cache-dir -U certifi

ENV VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

# Prefer system CA bundle (for Kit/Carb/Requests)
ENV SSL_CERT_DIR=/etc/ssl/certs \
    SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt \
    REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt

# ROS 2 Jazzy via apt (for CLI tools/external nodes)
# IMPORTANT: using http://packages.ros.org to bypass TLS name-mismatch issues.
RUN add-apt-repository -y universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list' && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base ros-dev-tools python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy ROS_VERSION=2 ROS_PYTHON_VERSION=3

# Cyclone DDS config
COPY cyclonedds.xml /etc/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# Isaac Sim internal ROS2 bridge libs (Python 3.11)
ENV ISAACSIM_BRIDGE_LIB=/opt/py311/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib
ENV LD_LIBRARY_PATH="${ISAACSIM_BRIDGE_LIB}:${LD_LIBRARY_PATH}"

# Create directories used by your bind mounts and make them writable by UID/GID 1000
RUN mkdir -p /home/ubuntu /home/ubuntu/Documents && \
    touch /home/ubuntu/.Xauthority && \
    mkdir -p /isaac-sim/.cache /isaac-sim/.nv/ComputeCache \
             /isaac-sim/.nvidia-omniverse/logs /isaac-sim/.nvidia-omniverse/config \
             /isaac-sim/.local/share/ov/data /isaac-sim/.local/share/ov/pkg && \
    chown -R 1000:1000 /home/ubuntu /isaac-sim
ENV HOME=/home/ubuntu \
    XAUTHORITY=/home/ubuntu/.Xauthority

# Run as "host-like" user (UID/GID 1000)
USER 1000:1000
WORKDIR /home/ubuntu

CMD ["bash", "-lc", "isaacsim --help"]
