# Isaac Sim 5.0.0 + ROS 2 Jazzy via apt (Ubuntu 24.04)
# Isaac Sim runs with its internal Python 3.11 ROS bridge (rclpy), NOT the system ROS Python.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    OMNI_KIT_ACCEPT_EULA=YES \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# --- Base OS + graphics/Vulkan + tools ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl wget git unzip bzip2 locales tzdata sudo \
    libgl1 libegl1 libvulkan1 vulkan-tools \
    libx11-6 libxext6 libxrender1 libxrandr2 \
    libxi6 libxfixes3 libxcursor1 libxkbcommon0 libxcomposite1 libxdamage1 libxinerama1 \
    libnss3 libdrm2 libgbm1 libxshmfence1 libasound2t64 xdg-user-dirs \
    fonts-dejavu-core fonts-noto-core vim nano less \
 && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8

# --- Python 3.11 venv for Isaac Sim (Ubuntu 24.04 defaults to 3.12 system Python) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
      software-properties-common gnupg2 curl ca-certificates && \
    add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && apt-get install -y --no-install-recommends \
      python3.11 python3.11-venv python3.11-dev && \
    rm -rf /var/lib/apt/lists/*

RUN python3.11 -m venv /opt/py311
ENV PATH=/opt/py311/bin:$PATH

# --- Isaac Sim 5.0.0 (pip) ---
ARG ISAACSIM_VERSION=5.0.0
RUN python -m pip install --no-cache-dir -U pip setuptools wheel && \
    python -m pip install --no-cache-dir "isaacsim[all,extscache]==${ISAACSIM_VERSION}" \

# --- Ensure a modern cert bundle (keep certifi for fallback/testing) ---
RUN python -m pip install --no-cache-dir -U certifi
#
# Prefer the system CA bundle so corporate/private CAs installed via
# `update-ca-certificates` are honored automatically.
ENV SSL_CERT_DIR=/etc/ssl/certs \
    SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt \
    REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt

# --- ROS 2 Jazzy via apt (system Python 3.12) ---
# We will NOT source this when launching Isaac; it's only for CLI tools and external nodes.
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common gnupg2 curl && \
    add-apt-repository -y universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null && \
    sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list' && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-dev-tools python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy ROS_VERSION=2 ROS_PYTHON_VERSION=3

# --- Cyclone DDS config: use YOUR file from the build context ---
# Put cyclonedds.xml next to this Dockerfile
COPY cyclonedds.xml /etc/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# --- Isaac Sim internal ROS 2 bridge (Python 3.11) wiring ---
# We do NOT source /opt/ros/jazzy here. Instead, point Isaac to its internal bridge libs.
ENV ISAACSIM_BRIDGE_LIB=/opt/py311/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib
ENV LD_LIBRARY_PATH="${ISAACSIM_BRIDGE_LIB}"

# --- Convenience in root shells (but *do not* source system ROS automatically) ---
RUN echo 'export PATH=/opt/py311/bin:$PATH' >> /root/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.bashrc && \
    echo 'export CYCLONEDDS_URI=file:///etc/cyclonedds.xml' >> /root/.bashrc && \
    echo '# To use ros2 CLI manually, run: source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

# --- Non-root dev user with passwordless sudo ---
RUN useradd -m -u 1010 -s /bin/bash isaac && \
    mkdir -p /home/isaac/.cache/ov /home/isaac/.nvidia-omniverse/logs /home/isaac/Documents && \
    chown -R isaac:isaac /home/isaac && \
    usermod -aG sudo isaac && \
    echo "isaac ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    cp /root/.bashrc /home/isaac/.bashrc && chown isaac:isaac /home/isaac/.bashrc

USER isaac
WORKDIR /home/isaac

# --- Cache volumes ---
VOLUME ["/home/isaac/.cache/ov", "/home/isaac/.nvidia-omniverse/logs", "/home/isaac/Documents"]

# --- Launch helpers ---
# These DO NOT source /opt/ros/jazzy to avoid Python-mismatch issues.
# They also export CA bundle vars so Kit/Carb HTTPS works reliably.
RUN echo '#!/usr/bin/env bash'                                   >  /home/isaac/isaac-gui && \
    echo 'set -euo pipefail'                                     >> /home/isaac/isaac-gui && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'          >> /home/isaac/isaac-gui && \
    echo 'export CYCLONEDDS_URI=file:///etc/cyclonedds.xml'      >> /home/isaac/isaac-gui && \
    echo '# Use system CA bundle (works with corp roots); override at runtime if needed' >> /home/isaac/isaac-gui && \
    echo 'export SSL_CERT_DIR=/etc/ssl/certs'                    >> /home/isaac/isaac-gui && \
    echo 'export SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt' >> /home/isaac/isaac-gui && \
    echo 'export REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt' >> /home/isaac/isaac-gui && \
    echo 'export LD_LIBRARY_PATH="'"$ISAACSIM_BRIDGE_LIB"':${LD_LIBRARY_PATH:-}"' >> /home/isaac/isaac-gui && \
    echo 'exec isaacsim --/exts/omni.kit.widget.cache_indicator/enabled=false isaacsim.exp.full "$@"' >> /home/isaac/isaac-gui && \
    \
    echo '#!/usr/bin/env bash'                                   >  /home/isaac/isaac-headless && \
    echo 'set -euo pipefail'                                     >> /home/isaac/isaac-headless && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'          >> /home/isaac/isaac-headless && \
    echo 'export CYCLONEDDS_URI=file:///etc/cyclonedds.xml'      >> /home/isaac/isaac-headless && \
    echo 'export SSL_CERT_DIR=/etc/ssl/certs'                    >> /home/isaac/isaac-headless && \
    echo 'export SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt' >> /home/isaac/isaac-headless && \
    echo 'export REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt' >> /home/isaac/isaac-headless && \
    echo 'export LD_LIBRARY_PATH="'"$ISAACSIM_BRIDGE_LIB"':${LD_LIBRARY_PATH:-}"' >> /home/isaac/isaac-headless && \
    echo 'exec isaacsim --/exts/omni.kit.widget.cache_indicator/enabled=false isaacsim.exp.full.streaming --no-window "$@"' >> /home/isaac/isaac-headless && \
    chmod +x /home/isaac/isaac-gui /home/isaac/isaac-headless

# Default: show Isaac Sim CLI help
CMD ["bash", "-lc", "isaacsim --help"]
