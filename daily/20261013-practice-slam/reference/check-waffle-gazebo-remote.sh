#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
cd "$(dirname "$0")"
source install/setup.bash
source gazebo_install/setup.bash
export ROS_DOMAIN_ID=176
export GZ_PARTITION=waffle_course_check_176
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 check_waffle_gazebo.py
