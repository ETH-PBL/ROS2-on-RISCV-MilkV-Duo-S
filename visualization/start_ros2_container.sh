#!/bin/bash

CONTAINER_NAME="ros2_drone_viz"

# Check if container already exists
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Container exists. Starting it..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    echo "Creating new container..."
    xhost +local:docker
    
    docker run -it \
        --name $CONTAINER_NAME \
        --network host \
        --env DISPLAY=$DISPLAY \
        --env QT_X11_NO_MITSHM=1 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --volume ~/drone_viz_ws:/root/drone_viz_ws:rw \
        --device /dev/dri:/dev/dri \
        osrf/ros:humble-desktop-full \
        bash
fi
