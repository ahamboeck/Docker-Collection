#!/usr/bin/env bash
set -euo pipefail

echo "Host UID=$(id -u) GID=$(id -g)"

# Minimal (still permissive); safer would be xhost +si:localuser:$(whoami)
xhost +local: >/dev/null

docker run --name isaac_sim_ros2 \
    --init \
    -it --gpus all \
    --rm \
    --runtime=nvidia \
    --network=host \
    --privileged \
    -e DISPLAY \
    -e HOME=/home/ubuntu \
    -e XAUTHORITY=/home/ubuntu/.Xauthority \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$HOME/.Xauthority":/home/ubuntu/.Xauthority:ro \
    -v "$HOME/docker/isaac-sim/cache/kit":/isaac-sim/kit/cache:rw \
    -v "$HOME/docker/isaac-sim/assets":/isaac-sim/assets:rw \
    -v "$HOME/docker/isaac-sim/cache/main":/isaac-sim/.cache:rw \
    -v "$HOME/docker/isaac-sim/cache/computecache":/isaac-sim/.nv/ComputeCache:rw \
    -v "$HOME/docker/isaac-sim/logs":/isaac-sim/.nvidia-omniverse/logs:rw \
    -v "$HOME/docker/isaac-sim/config":/isaac-sim/.nvidia-omniverse/config:rw \
    -v "$HOME/docker/isaac-sim/data":/isaac-sim/.local/share/ov/data:rw \
    -v "$HOME/docker/isaac-sim/pkg":/isaac-sim/.local/share/ov/pkg:rw \
    -v "$HOME/docker/isaac-sim/documents":/home/ubuntu/Documents:rw \
    -v "$HOME/tmp":/home/ubuntu/tmp:rw \
    -v "$HOME/local_git/dev":/home/ubuntu/local_git/dev:rw \
    isaac_sim_ros2:5.1.0-jazzy \
    bash
