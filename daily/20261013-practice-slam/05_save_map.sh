#!/bin/bash
# 05 - 매핑된 지도 저장 (pgm + yaml)
# 실행: bash 05_save_map.sh [저장경로_prefix]
# 예: bash 05_save_map.sh ~/maps/tb3_house
set -e
source /opt/ros/lyrical/setup.bash

export ROS_DOMAIN_ID=9

MAP_PREFIX=${1:-$(pwd)/tb3_map}

mkdir -p "$(dirname "${MAP_PREFIX}")"

echo "지도 저장: ${MAP_PREFIX}.pgm / ${MAP_PREFIX}.yaml"
ros2 run nav2_map_server map_saver_cli -f "${MAP_PREFIX}" --ros-args -p use_sim_time:=true

echo ""
echo "저장 확인:"
ls -lh "${MAP_PREFIX}.pgm" "${MAP_PREFIX}.yaml"
