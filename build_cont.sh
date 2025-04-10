#!/bin/bash

# Разрешаем доступ к X серверу
xhost +local:

# Генерируем случайный ROS_DOMAIN_ID
RANDOM_DOMAIN_ID=$(( RANDOM % 250 ))
echo "Используется ROS_DOMAIN_ID: $RANDOM_DOMAIN_ID"

# Определяем, нужно ли подключать GPU
USE_GPU=false
if [ "$1" == "--gpu" ]; then
    USE_GPU=true
fi

GPU_OPTION=""
if [ "$USE_GPU" = true ]; then
    GPU_OPTION="--gpus all"
fi

# Запускаем контейнер с нужными параметрами
docker run -it \
    $GPU_OPTION \
    -e DISPLAY=$DISPLAY \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --env ROS_DOMAIN_ID="$RANDOM_DOMAIN_ID" \
    --net host \
    --shm-size=6G \
    --volume "$(pwd):/mujoco" \
    --name mujoco \
    mujoco
