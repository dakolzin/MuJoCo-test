#!/bin/sh

CONTAINER_NAME="mujoco"
COMMANDS="cd src/ && . /opt/ros/humble/setup.bash && colcon build && . install/setup.bash && cd .."

docker exec -it "$CONTAINER_NAME" /bin/bash -c "$COMMANDS && exec /bin/bash"
