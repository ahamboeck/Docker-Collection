## ROS2 Humble with Zenoh Support

This Docker image provides ROS2 Humble with both CycloneDX and Zenoh RMW implementations.

### Building

Build the Docker image using docker compose (uses `.env` file for configuration):

```shell
docker compose build
```

### Running

Start and bash into the container:

```shell
docker compose up -d
docker compose exec ros2 bash
```

or

```shell
docker exec -it ros2-humble-dev bash
```

### Starting a Zenoh Router

Inside the container, start a Zenoh router:

```shell
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Configuration

- Edit `.env` to customize workspace path and image name/tag
- RMW implementation defaults to `rmw_zenoh_cpp` but can be switched at runtime
- Zenoh and CycloneDDS configuration files are included in the image