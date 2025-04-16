#!/bin/bash

colcon build --symlink-install

source ./install/setup.bash

ros2 launch communicate_2025_aatest launch.py 