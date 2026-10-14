#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
set -u
ros2 pkg prefix isaac_ros_cuvslam
dpkg-query -W ros-lyrical-isaac-ros-cuvslam
python3 synthetic_cuvslam_check.py
