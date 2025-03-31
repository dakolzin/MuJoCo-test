#!/bin/bash

cd src/

colcon build

. install/setup.bash

ros2 run my_tf_broadcaster exp

cd ..