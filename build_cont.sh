#!/bin/bash

xhost +local:docker

docker run -it -e DISPLAY=$DISPLAY \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e ROS_DOMAIN_ID=15 \
    --net host \
    --shm-size=6G \
    --volume $(pwd):/mujoco \
    --name mujoco mujoco \
    -c ". /opt/ros/humble/setup.bash; cd /mujoco; colcon build; bash runlaunch.sh"
