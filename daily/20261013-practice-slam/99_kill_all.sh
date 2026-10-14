#!/bin/bash
# 99 - 실습에서 띄운 것 전부 종료 (Gazebo 화면 없는 서버까지)
# 실행: bash 99_kill_all.sh
# 이후 01_gazebo.sh 부터 다시 실행
source /opt/ros/lyrical/setup.bash

export ROS_DOMAIN_ID=9

# Gazebo 서버/GUI, 브리지, SLAM, RViz, 조종
PATTERNS='gz-sim-main|ros_gz|cartographer|slam_toolbox|rviz2|teleop_twist_keyboard'

echo "종료 대상:"
pgrep -af "${PATTERNS}" || echo "  (없음)"

# 1차: 정상 종료 요청 (SIGTERM)
pkill -f "${PATTERNS}"

# 최대 5초 기다림
for i in 1 2 3 4 5; do
  pgrep -f "${PATTERNS}" > /dev/null || break
  sleep 1
done

# 2차: 남은 것 강제 종료 (SIGKILL)
if pgrep -f "${PATTERNS}" > /dev/null; then
  echo "안 꺼진 프로세스 강제 종료:"
  pgrep -af "${PATTERNS}"
  pkill -9 -f "${PATTERNS}"
  sleep 1
fi

# ros2 node list 에 남는 옛날 노드 캐시 비우기
ros2 daemon stop > /dev/null 2>&1

echo ""
echo "남은 프로세스:"
pgrep -af "${PATTERNS}" || echo "  (없음) 전부 종료됨"
