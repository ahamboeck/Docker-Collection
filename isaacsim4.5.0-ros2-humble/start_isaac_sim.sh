#!/bin/bash

xhost +

docker run --name isaac_sim_ros2 \
    --entrypoint bash \
    -it --gpus all \
    --rm \
    --network=host \
    --privileged \
    -e "ACCEPT_EULA=Y" \
    -e "PRIVACY_CONSENT=Y" \
    -e DISPLAY \
    -v /dev/input/js0:/dev/input/js0 \
    -v /dev/input/js1:/dev/input/js1 \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    -v ~/local_git/dev/:/root/local_git/dev/ \
    isaac_sim_ros2:4.5.0-humble
