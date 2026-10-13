#!/bin/bash
# 02b - Cartographer 실행 (TurtleBot3 + Gazebo용)
# 전제: bash 00_build_cartographer.sh 완료 + 01_gazebo.sh 실행 중
# 실행: bash 02_cartographer.sh
set -e
source /opt/ros/lyrical/setup.bash

export ROS_DOMAIN_ID=9

# cartographer_ws 자동 탐색
for ws in "$HOME/cartographer_ws" "/home/pank/cartographer_ws"; do
  if [ -f "$ws/install/setup.bash" ]; then
    source "$ws/install/setup.bash"
    echo "sourced: $ws"
    break
  fi
done

DIR="$(cd "$(dirname "$0")" && pwd)"

if ! ros2 pkg list 2>/dev/null | grep -qx cartographer_ros; then
  echo "[에러] cartographer_ros 패키지가 없습니다. (msgs만 있고 노드는 없음)"
  echo "먼저: bash ${DIR}/00_build_cartographer.sh"
  exit 1
fi

echo "Cartographer TB3 실행 (config=${DIR}/config/turtlebot3_lds_2d.lua)..."
ros2 launch "${DIR}/launch/cartographer_tb3.launch.py"
