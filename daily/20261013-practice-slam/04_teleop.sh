#!/bin/bash
# 04 - Gazebo 로봇 조종 (키보드 텔레옵)
# 실행: bash 04_teleop.sh
# 조작: i/j/k/l/, + u/o/m/. (이 터미널에 포커스 두고 키 입력)
set -e
source /opt/ros/lyrical/setup.bash

export ROS_DOMAIN_ID=9

echo "teleop_twist_keyboard 실행..."
echo "포커스가 이 터미널에 있어야 키가 먹습니다."
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
