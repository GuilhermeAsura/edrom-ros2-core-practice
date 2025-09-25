#!/bin/bash
# Script para executar o container

# Configurações do container
CONTAINER_NAME="edrom_ros2_container"
IMAGE_NAME="edrom_image"
WORKSPACE_DIR="$HOME/Desktop/ROS2/workspace"
DISPLAY_SETTING=$DISPLAY

# Executar o container
docker run -e "TERM=xterm-256color" -t --rm -it \
  --name "$CONTAINER_NAME" \
  --user ros \
  --network=host \
  --ipc=host \
  -v "$WORKSPACE_DIR:/my_workspace" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --env=DISPLAY="$DISPLAY_SETTING" \
  -v /dev:/dev \
  --device-cgroup-rule='c *:* rmw' \
  "$IMAGE_NAME"
