#!/bin/bash
# 00b - Cartographer ROS 소스 빌드 (lyrical apt에 cartographer_ros 바이너리 없음)
# 실행: bash 00_build_cartographer.sh  (비밀번호 필요, 10~20분 소요)
set -e
source /opt/ros/lyrical/setup.bash

WS="${CARTO_WS:-$HOME/cartographer_ws}"

echo "[1/5] apt 의존성 설치..."
# 주의: ros-lyrical-pcl-conversions는 현재 libpcl1.15를 요구해서 noble(1.14)에서 설치 불가
# (상위 패키징 버그). 대신 libpcl-dev(1.14) + pcl_conversions 소스빌드로 우회.
sudo apt update
sudo apt install -y \
  ros-lyrical-cartographer \
  ros-lyrical-cartographer-ros-msgs \
  ros-lyrical-tf2-eigen \
  libpcl-dev \
  libabsl-dev libboost-iostreams-dev libcairo2-dev libceres-dev \
  libeigen3-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev \
  libprotobuf-dev protobuf-compiler \
  python3-rosdep python3-vcstool git build-essential cmake

echo "[2/5] rosdep 초기화 확인..."
sudo rosdep init 2>/dev/null || true
rosdep update

echo "[3/5] 소스 클론: ${WS}/src"
mkdir -p "${WS}/src"
if [ ! -d "${WS}/src/cartographer_ros" ]; then
  git clone -b ros2 https://github.com/ros2/cartographer_ros.git "${WS}/src/cartographer_ros"
else
  echo "이미 있음, pull..."
  git -C "${WS}/src/cartographer_ros" pull --ff-only || true
fi
# pcl_conversions 소스빌드 (apt 버전이 libpcl1.15 요구로 깨져있어서 우회)
# 주의: ros2/pcl_conversions 저장소는 폐기됨(README만 남음). 실제 코드는 ros-perception/perception_pcl에 있음.
if [ ! -f "${WS}/src/pcl_conversions/package.xml" ]; then
  rm -rf "${WS}/src/pcl_conversions"
fi
if [ ! -d "${WS}/src/perception_pcl" ]; then
  git clone https://github.com/ros-perception/perception_pcl.git "${WS}/src/perception_pcl"
else
  echo "perception_pcl 이미 있음, pull..."
  git -C "${WS}/src/perception_pcl" pull --ff-only || true
fi

echo "[4/5] rosdep install..."
rosdep install --from-paths "${WS}/src" --ignore-src -r -y

echo "[4.5/5] gflags 중복 패치 (node_main.cpp)..."
# cartographer_node는 node_main.cpp + shared lib(offline_node.cpp 포함)로 링크되는데
# 양쪽에 같은 gflags DEFINE이 있어 시작 즉시 abort함. 겹치는 5개는 DECLARE로 전환.
python3 - "${WS}/src/cartographer_ros/cartographer_ros/src/node_main.cpp" <<'EOF'
import sys
p = sys.argv[1]
s = open(p).read()
if 'DECLARE_bool(collect_metrics);' in s:
    print('이미 패치됨, skip')
    sys.exit(0)
s = s.replace('''DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");''',
'''// NOTE(lyrical-patch): also DEFINE'd in offline_node.cpp (linked via the
// shared library). Duplicate DEFINEs make gflags abort, so DECLARE here.
DECLARE_bool(collect_metrics);
DECLARE_string(configuration_directory);''')
s = s.replace('''DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");''',
'''DECLARE_string(load_state_filename);
DECLARE_bool(load_frozen_state);''')
s = s.replace('''DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");''',
'''DECLARE_string(save_state_filename);''')
open(p, 'w').write(s)
print('패치 적용됨')
EOF

echo "[5/5] colcon build..."
cd "${WS}"
# 이전 실패 잔재 정리 (실패했거나 미처리된 패키지만)
rm -rf "${WS}/build/cartographer_ros" "${WS}/build/cartographer_rviz"
colcon build --symlink-install --packages-select pcl_conversions cartographer_ros_msgs cartographer_ros cartographer_rviz

echo ""
echo "완료. 사용법:"
echo "  source ${WS}/install/setup.bash"
echo "  bash 02_cartographer.sh"
