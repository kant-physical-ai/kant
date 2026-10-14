#!/bin/bash
# Cartographer 풀코스: Gazebo + Cartographer + RViz + Teleop
# 실행: bash run_cartographer.sh [turtlebot3_world|turtlebot3_house|empty_world]
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
WORLD=${1:-turtlebot3_world}

if ! ros2 pkg list 2>/dev/null | grep -q turtlebot3_gazebo; then
  echo "[경고] turtlebot3_gazebo 없음. 먼저: bash ${DIR}/00_install.sh"
  exit 1
fi
if ! ros2 pkg list 2>/dev/null | grep -qx cartographer_ros; then
  echo "[경고] cartographer_ros 없음 (msgs만 있음). 먼저: bash ${DIR}/00_build_cartographer.sh"
  echo "(빌드 후 이 터미널에서도: source ~/cartographer_ws/install/setup.bash)"
  exit 1
fi

gnome-terminal --tab --title="1-Gazebo" -- bash -c "bash '${DIR}/01_gazebo.sh' '${WORLD}'; exec bash" \
               --tab --title="2-Carto" -- bash -c "sleep 8; bash '${DIR}/02_cartographer.sh'; exec bash" \
               --tab --title="3-RViz" -- bash -c "sleep 11; bash '${DIR}/03_rviz.sh'; exec bash" \
               --tab --title="4-Teleop" -- bash -c "sleep 14; bash '${DIR}/04_teleop.sh'; exec bash"

echo ""
echo "4개 탭 실행됨. 매핑 끝나면:"
echo "  bash ${DIR}/05_save_map.sh ./carto_map   # /map 토픽 저장 (occupancy_grid_node 발행)"
