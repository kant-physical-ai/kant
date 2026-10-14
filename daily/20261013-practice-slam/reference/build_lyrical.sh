#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ ! -f /opt/ros/lyrical/setup.bash ]]; then
    echo 'ROS 2 Lyrical이 없습니다. README의 Ubuntu 24.04 NVIDIA Buildfarm 설치 안내를 먼저 따르세요.' >&2
    exit 1
  fi
  # ROS setup scripts may reference unset variables while this script uses nounset.
  set +u
  source /opt/ros/lyrical/setup.bash
  set -u
fi
[[ "$ROS_DISTRO" == lyrical ]] || { echo 'Lyrical is required' >&2; exit 1; }
command -v vcs >/dev/null
command -v rosdep >/dev/null
command -v colcon >/dev/null
mkdir -p src
vcs import src < sources.repos
# Keep offline-only gflags out of the shared library used by every executable.
patch_file="$PWD/patches/cartographer-offline-flags.patch"
if git -C src/cartographer_ros apply --check "$patch_file"; then
  git -C src/cartographer_ros apply "$patch_file"
elif git -C src/cartographer_ros apply --reverse --check "$patch_file"; then
  echo 'Cartographer flag patch is already applied'
else
  echo 'Pinned Cartographer source differs from the reviewed patch' >&2
  exit 1
fi
# Build only the description and SLAM dependency closure, not Jazzy hardware drivers.
rosdep install --from-paths src/cartographer src/cartographer_ros \
  src/perception_pcl/pcl_conversions src/turtlebot3/turtlebot3_description \
  --ignore-src --rosdistro lyrical -y
if [[ "${1:-cartographer}" == cuvslam ]]; then
  ros2 pkg prefix isaac_ros_cuvslam >/dev/null
fi
colcon build --base-paths src --symlink-install --packages-up-to waffle_slam_course \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
echo 'Next: source install/setup.bash'
