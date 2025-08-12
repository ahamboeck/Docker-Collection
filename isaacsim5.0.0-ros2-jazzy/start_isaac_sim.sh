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
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
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
    -v ~/tmp:/home/isaac/tmp:rw \
    -v ~/local_git/dev/:/home/isaac/local_git/dev/ \
    -v /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:ro \
    -v /usr/share/vulkan/implicit_layer.d:/usr/share/vulkan/implicit_layer.d:ro \
    -v /usr/share/glvnd/egl_vendor.d:/usr/share/glvnd/egl_vendor.d:ro \
    isaac_sim_ros2:5.0.0-jazzy
