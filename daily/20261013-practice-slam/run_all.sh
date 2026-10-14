#!/bin/bash
# 전체 실행: 터미널 4개를 자동으로 띄움
# 실행: bash run_all.sh [turtlebot3_world|turtlebot3_house|empty_world]
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
WORLD=${1:-turtlebot3_world}

echo "실습 디렉토리: ${DIR}"
echo "월드: ${WORLD}"
echo "순서: 1.Gazebo -> 2.SLAM -> 3.RViz -> 4.Teleop (각 3초 간격)"
echo ""

# 패키지 설치 여부 체크
if ! ros2 pkg list 2>/dev/null | grep -q turtlebot3_gazebo; then
  echo "[경고] turtlebot3_gazebo 가 없습니다. 먼저 아래를 실행하세요:"
  echo "  bash ${DIR}/00_install.sh"
  exit 1
fi

gnome-terminal --tab --title="1-Gazebo" -- bash -c "bash '${DIR}/01_gazebo.sh' '${WORLD}'; exec bash" \
               --tab --title="2-SLAM" -- bash -c "sleep 8; bash '${DIR}/02_slam.sh'; exec bash" \
               --tab --title="3-RViz" -- bash -c "sleep 11; bash '${DIR}/03_rviz.sh'; exec bash" \
               --tab --title="4-Teleop" -- bash -c "sleep 14; bash '${DIR}/04_teleop.sh'; exec bash"

echo ""
echo "4개 탭 실행됨. 매핑 끝나면 별도 터미널에서:"
echo "  bash ${DIR}/05_save_map.sh"
