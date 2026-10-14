#!/usr/bin/env bash
set -euo pipefail
[[ "${ROS_DISTRO:-}" == lyrical ]] || { echo 'Activate Lyrical first'; exit 1; }
mode="${1:?Usage: bash preflight.sh cartographer|cuvslam}"
case "$mode" in
 cartographer) topics=(/clock /scan /odom /joint_states);;
 cuvslam) topics=(/clock /waffle/left/image_rect /waffle/right/image_rect /waffle/left/camera_info /waffle/right/camera_info /joint_states);;
 *) echo 'Unknown mode'; exit 1;;
esac
for topic in "${topics[@]}"; do
 echo "Checking $topic"
 timeout 15 ros2 topic echo "$topic" --once --qos-reliability best_effort >/dev/null
done
echo 'Messages received. Now check synchronization, calibration and TF using README acceptance steps.'
