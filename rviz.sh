#!/bin/bash

cd src/

colcon build

. install/setup.bash

ros2 launch my_tf_broadcaster view.launch.py

cd ..