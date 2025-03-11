#!/bin/bash
# This script is responsible to launch the rebars segmentation node

# author [Lars Dethlefsen] - lars.dethlefsen@gmail.com
# version 0.1
# date 2025-01-27

# Copyright (c) 2025 - DTU. All rights reserved.

echo "WP5.3 - IRR Vision - Rebars Segmentation"
source /ros_entrypoint.sh
set +e
echo "Setting up ROS environment............."
source ~/catkin_ws/devel/setup.bash --extend
echo "Launching Rebars Segmentation ............."

roslaunch rebarsegmenation rebarsegmenation.launch
