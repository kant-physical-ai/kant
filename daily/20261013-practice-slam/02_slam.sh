#!/bin/bash
# 02 - SLAM (지도 제작) - cartographer 대신 slam_toolbox 사용
# 이유: ros-lyrical-cartographer-ros (노드 패키지)가 lyrical에 apt 릴리즈 없음.
#       lib(cartographer) + msgs만 있음. 표준 대체재인 slam_toolbox 사용.
# 실행: bash 02_slam.sh
set -e
source /opt/ros/lyrical/setup.bash

export ROS_DOMAIN_ID=9

echo "slam_toolbox online_async_launch 실행 (use_sim_time=true)..."
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/opt/ros/lyrical/share/slam_toolbox/config/mapper_params_online_async.yaml \
  use_sim_time:=true
