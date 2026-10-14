#!/bin/bash
# 00 - ROS2 Lyrical SLAM 실습용 패키지 설치
# 실행: bash 00_install.sh  (비밀번호 입력 필요)
set -e
source /opt/ros/lyrical/setup.bash

echo "[1/3] apt update..."
sudo apt update

echo "[2/3] 필수 패키지 설치..."
sudo apt install -y \
  ros-lyrical-turtlebot3-simulations \
  ros-lyrical-turtlebot3-gazebo \
  ros-lyrical-ros-gz-sim \
  ros-lyrical-ros-gz-bridge \
  ros-lyrical-slam-toolbox \
  ros-lyrical-nav2-map-server \
  ros-lyrical-nav2-rviz-plugins \
  ros-lyrical-teleop-twist-keyboard \
  ros-lyrical-cartographer \
  ros-lyrical-cartographer-ros-msgs

echo "[3/3] 설치 확인..."
ros2 pkg list | grep -E "turtlebot3_gazebo|slam_toolbox|nav2_map_server|teleop_twist_keyboard"

echo ""
echo "완료. 다음으로 01_gazebo.sh 부터 실행하세요."
echo "Cartographer를 쓸 경우 추가로: bash 00_build_cartographer.sh (소스 빌드, 10~20분)"
