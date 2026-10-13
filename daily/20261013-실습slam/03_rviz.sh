#!/bin/bash
# 03 - RViz2 로 SLAM 지도 보기
# 실행: bash 03_rviz.sh
set -e
source /opt/ros/lyrical/setup.bash

export ROS_DOMAIN_ID=9

RVIZ_CONFIG="/opt/ros/lyrical/share/slam_toolbox/config/slam_toolbox_default.rviz"

if [ -f "${RVIZ_CONFIG}" ]; then
  echo "rviz2 실행 (slam_toolbox 기본 config)..."
  ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true
else
  echo "rviz2 실행 (기본)..."
  ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
fi
