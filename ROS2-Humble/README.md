## ROS2 Humble

### Building

```shell
./run.sh -w dev_ws -i [YOUR_IMAGE_NAME:TAG] -b
```

### Running

- Update `.tmux.conf` if you need to enable additional `tmux` features
- Update `.session.yml` to customize Tmuxinator UI

Note that the above configs are mapped as volumes to docker image.

```shell
./run.sh -w dev_ws -i [YOUR_IMAGE_NAME:TAG] -r
```

### The following repo was used as a guidance to create this repo

[text](https://github.com/sskorol/ros2-humble-docker-dev-template)