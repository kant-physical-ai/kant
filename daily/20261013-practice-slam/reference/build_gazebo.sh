#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/lyrical/setup.bash
source install/setup.bash
set -u
commit=45633014a14e8f438495b532a723e4ad45cbbd31
if [[ ! -d tb3_simulation_source ]]; then
  git clone --branch jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git tb3_simulation_source
  git -C tb3_simulation_source checkout "$commit"
fi
test "$(git -C tb3_simulation_source rev-parse HEAD)" = "$commit"
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix ros_gz_image
colcon --log-base gazebo_log build \
  --base-paths tb3_simulation_source/turtlebot3_gazebo \
  --build-base gazebo_build --install-base gazebo_install \
  --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
