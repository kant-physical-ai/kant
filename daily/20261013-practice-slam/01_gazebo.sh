#!/bin/bash
# 01 - Gazebo (gz sim) + TurtleBot3 스폰
# 실행: bash 01_gazebo.sh [turtlebot3_world|turtlebot3_house|empty_world]
# 기본: turtlebot3_world (SLAM 매핑용으로 벽이 많아서 좋음)
set -e
source /opt/ros/lyrical/setup.bash

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=9

WORLD=${1:-turtlebot3_world}
# WORLD.launch.py 형태로 변환 (예: turtlebot3_world -> turtlebot3_world.launch.py)
LAUNCH_FILE="${WORLD}.launch.py"

echo "TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"
echo "WORLD=${WORLD} (${LAUNCH_FILE})"
echo "사용 가능한 월드: turtlebot3_world, turtlebot3_house, empty_world"
echo ""

ros2 launch turtlebot3_gazebo "${LAUNCH_FILE}"
