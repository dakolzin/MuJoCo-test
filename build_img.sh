#!/bin/bash

BASE_IMAGE=${1:-osrf/ros:humble-desktop-full}

docker build --build-arg BASE_IMAGE="$BASE_IMAGE" -t mujoco .
