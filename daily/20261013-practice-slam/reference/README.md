# Waffle Lyrical SLAM
https://github.com/SeongminJaden/waffle-lyrical-slam

Ubuntu 24.04 · ROS 2 Lyrical · TurtleBot3 Waffle · Cartographer · cuVSLAM 강의 예제.
Gazebo 기본 월드는 사용자 STL에서 변환한 **Twin Closed Maze B**다.
미로 크기는 4.464 × 6.364m, 높이는 0.390m다.

## Gazebo 빠른 실행

Ubuntu 24.04에 [NVIDIA Buildfarm의 ROS 2 Lyrical](https://nvidia-isaac-ros.github.io/getting_started/isaac_ros_buildfarm_cdn.html)을 먼저 설치한다.
명령은 Ubuntu 터미널에서 실행한다.

```bash
git clone https://github.com/SeongminJaden/waffle-lyrical-slam.git "$HOME/waffle-lyrical-slam"
cd "$HOME/waffle-lyrical-slam"
source /opt/ros/lyrical/setup.bash
sudo apt-get update
sudo apt-get install -y git build-essential cmake python3-numpy python3-vcstool \
  python3-colcon-common-extensions python3-rosdep \
  ros-lyrical-robot-state-publisher ros-lyrical-xacro ros-lyrical-tf2-ros \
  ros-lyrical-rviz2 ros-lyrical-nav2-map-server ros-lyrical-teleop-twist-keyboard \
  ros-lyrical-ros-gz-sim ros-lyrical-ros-gz-bridge ros-lyrical-ros-gz-image
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi
rosdep update
bash build_lyrical.sh
bash build_gazebo.sh
source install/setup.bash
source gazebo_install/setup.bash
export ROS_DOMAIN_ID=174
export GZ_PARTITION=waffle_course
ros2 launch waffle_slam_course gazebo.launch.py gui:=true
```

`build_lyrical.sh`는 ROS 환경이 비어 있으면 `/opt/ros/lyrical/setup.bash`를
자동으로 불러온다. 파일 자체가 없다면 위 Buildfarm 안내에 따라 Lyrical을 먼저
설치한다. 다른 ROS 배포판을 불러온 터미널에서는 새 터미널을 열어 실행한다.
빌드 스크립트의 source는 부모 터미널에 남지 않으므로 빌드 후 launch를 실행할
터미널에서는 위 예시처럼 `source /opt/ros/lyrical/setup.bash`와 workspace source를 수행한다.

새 터미널에서 SLAM을 실행한다.

```bash
source /opt/ros/lyrical/setup.bash
source ~/waffle-lyrical-slam/install/setup.bash
export ROS_DOMAIN_ID=174
ros2 launch waffle_slam_course cartographer.launch.py
```

다른 터미널에서 키보드로 조종한다. `i` 전진, `,` 후진, `j/l` 회전,
`k` 또는 Space 정지다. 키에서 손을 떼는 것만으로 자동 정지하지 않는다.

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=174
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint -p use_sim_time:=true \
  -p speed:=0.1 -p turn:=0.2
```

기존 Gazebo 서버가 남아 있으면 새 GUI가 이전 월드에 연결될 수 있다.
이전 실습의 서버와 GUI를 종료한 뒤 실행한다. 다른 프로젝트의 Gazebo를 함께
실행할 경우 서로 다른 `GZ_PARTITION`과 `ROS_DOMAIN_ID`를 사용한다.

## 지도 저장

Gazebo와 Cartographer를 실행하고 키보드로 필요한 통로를 탐색한다.
정지 키 `k`를 누른 뒤 **Cartographer를 종료하기 전에** 새 터미널에서 저장한다.
아래 `WORKSPACE`는 실제 빌드한 폴더로 지정한다. 이전 실습 폴더에서 빌드했다면
그 폴더를 사용한다. Gazebo의 월드 STL과 SLAM이 만드는 점유 격자 지도는 서로 다르다.

```bash
source /opt/ros/lyrical/setup.bash
WORKSPACE="$HOME/waffle-lyrical-slam"
source "$WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=174

ros2 topic info /map
mkdir -p "$HOME/slam_maps"
MAP_PREFIX="$HOME/slam_maps/maze_$(date +%Y%m%d_%H%M%S)"
ros2 run nav2_map_server map_saver_cli \
  -f "$MAP_PREFIX" --ros-args -p use_sim_time:=true
ls -lh "${MAP_PREFIX}.pgm" "${MAP_PREFIX}.yaml"
```

- `.pgm`: 관측한 장애물·자유 공간·미관측 영역을 표시한 지도 이미지.
- `.yaml`: 이미지 경로, 해상도, 지도 원점 등 Nav2가 읽을 설정.

두 파일을 함께 보관한다. 파일명이 출력되고 저장 명령이 성공했는지 확인한다.
`/map`이 없으면 Cartographer가 실행 중인지, 모든 터미널의 domain이 174인지
확인한다. 탐색하지 않은 영역까지 자동으로 채워지지는 않는다.

Cartographer 상태도 보관하려면 **위와 같은 터미널에서**, Cartographer가 살아 있을 때 실행한다.
Nav2의 AMCL 실행에는 PGM/YAML을 사용하며 `.pbstream`을 직접 넣지 않는다.

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '${MAP_PREFIX}.pbstream', include_unfinished_submaps: true}"
ls -lh "${MAP_PREFIX}.pbstream"
```

## 저장한 지도에서 Nav2 내비게이션

아래는 **Gazebo Waffle + 2D 점유 격자 + AMCL** 경로다. cuVSLAM의 sparse 지도나
`.pbstream`을 Nav2의 `map` 인자로 사용하지 않는다. cuVSLAM+nvblox의 3D 환경에서
Nav2를 연결하는 절차는 별도 구성이다. 이 내비게이션 절차의 실제 목표 도달 주행은 아직 검증하지 않았다.

### 1. Nav2 설치와 Waffle 설정 준비

Ubuntu 24.04 NVIDIA Buildfarm의 Lyrical 환경에서 실행한다.
확인한 `nav2_bringup` 패키지는 1.5.0이다. 설치한 버전의 기본 YAML을 가져와
Waffle에 필요한 프레임·속도·메시지 설정만 변경한다.

```bash
source /opt/ros/lyrical/setup.bash
sudo apt-get update
sudo apt-get install -y ros-lyrical-navigation2 ros-lyrical-nav2-bringup python3-yaml
mkdir -p "$HOME/slam_maps"
python3 - <<'PY'
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory

source = Path(get_package_share_directory('nav2_bringup')) / 'params/nav2_params.yaml'
params = yaml.safe_load(source.read_text())
for name in ['controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor']:
    params[name]['ros__parameters']['enable_stamped_cmd_vel'] = True
params['amcl']['ros__parameters']['base_frame_id'] = 'base_footprint'
params['amcl']['ros__parameters']['scan_topic'] = '/scan'
params['bt_navigator']['ros__parameters']['robot_base_frame'] = 'base_footprint'
params['behavior_server']['ros__parameters']['robot_base_frame'] = 'base_footprint'
for name in ['local_costmap', 'global_costmap']:
    costmap = params[name][name]['ros__parameters']
    costmap['robot_base_frame'] = 'base_footprint'
    costmap['robot_radius'] = 0.22
    costmap['inflation_layer']['inflation_radius'] = 0.30
follow = params['controller_server']['ros__parameters']['FollowPath']
assert follow['plugin'] == 'nav2_mppi_controller::MPPIController', 'Check the installed Nav2 controller configuration'
follow.update(vx_max=0.1, vx_min=-0.1, vy_max=0.0, wz_max=0.2)
smoother = params['velocity_smoother']['ros__parameters']
smoother['max_velocity'] = [0.1, 0.0, 0.2]
smoother['min_velocity'] = [-0.1, 0.0, -0.2]
params['behavior_server']['ros__parameters']['max_rotational_vel'] = 0.2
params['behavior_server']['ros__parameters']['min_rotational_vel'] = 0.1
target = Path.home() / 'slam_maps/nav2_waffle.yaml'
target.write_text(yaml.safe_dump(params, sort_keys=False))
print(target)
PY
```

반경 0.22m, inflation 0.30m는 이 시뮬레이션의 시작 설정이며,
실물 장비의 치수·제동 거리를 측정한 값은 아니다.

### 2. SLAM에서 저장 지도 사용으로 전환

Gazebo는 실행한 상태로 둔다. Cartographer와 cuVSLAM의 launch를 Ctrl+C로 종료하고,
키보드 텔레옵도 중지한다. `map→odom`은 AMCL만 발행하며,
`odom→base_footprint`는 Gazebo가 발행한다. SLAM과 AMCL을 동시에 실행하지 않는다.

새 터미널에서 `MAP_YAML`을 **직접 저장한 YAML의 실제 경로**로 변경한다.

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=174
MAP_YAML="$HOME/slam_maps/maze_YYYYMMDD_HHMMSS.yaml"
test -f "$MAP_YAML" || { echo "MAP_YAML을 실제 저장 파일로 변경하세요"; exit 1; }
ros2 launch nav2_bringup bringup_launch.py \
  map:="$MAP_YAML" params_file:="$HOME/slam_maps/nav2_waffle.yaml" \
  use_sim_time:=true autostart:=true slam:=false \
  use_localization:=true use_composition:=false \
  use_keepout_zones:=false use_speed_zones:=false
```

영역 마스크 파일을 준비하지 않았으므로 keepout/speed zone은 끈다.
Nav2가 `active` 상태가 되기 전에 목표 주행을 시작하지 않는다.

### 3. RViz에서 초기 위치와 목표 지정

별도 터미널에서 실행한다.

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=174
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true
```

1. RViz의 Fixed Frame을 `map`으로 확인하고 저장 지도가 표시되는지 확인한다.
2. **2D Pose Estimate**로 지도 위 현재 로봇의 위치와 방향을 지정한다.
   저장 지도의 원점은 SLAM 좌표계이므로 Gazebo의 spawn 좌표를 그대로 입력하지 않는다.
3. LiDAR 점과 지도 벽이 겹치는지 확인한다. 어긋나면 초기 위치를 다시 지정한다.
4. **Nav2 Goal**로 지도에서 이미 관측된 자유 공간에 가까운 목표와 방향을 지정한다.
5. 경로·local/global costmap을 보고 이동을 확인한다. 미관측 영역이나 벽 안을 목표로 지정하지 않는다.

종료는 RViz/Nav2 터미널의 Ctrl+C로 수행한다. 목표 실행 중 키보드 teleop을
같이 발행하지 않는다. 이동을 중단하려면 RViz Navigation 패널에서 목표를 취소한다.

### 4. 실행 상태와 문제 확인

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=174
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic info /cmd_vel --verbose
ros2 action list -t
```

- `/cmd_vel`은 Gazebo bridge와 동일한 `geometry_msgs/msg/TwistStamped`여야 한다.
- `/scan`, `/odom`, `/clock`, TF가 없으면 먼저 Gazebo와 ROS domain을 확인한다.
- `/map`은 map_server, `map→odom`은 AMCL이 담당한다. 이전 SLAM 노드가 남아 있으면 종료한다.
- lifecycle이 `inactive`이거나 설정 오류가 있으면 Nav2 로그에서 첫 오류를 확인한다.
- 경로가 없으면 미관측 영역, 잘못된 초기 위치, 로봇 반경·inflation과 통로 폭을 확인한다.
  통과시키기 위해 실제 로봇보다 footprint를 작게 설정하지 않는다.

참고: [Nav2 로봇 설정 가이드](https://docs.nav2.org/setup_guides/index.html),
[Nav2 bringup launch](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/launch/bringup_launch.py).
실행 옵션은 설치한 Lyrical 패키지의 `bringup_launch.py`를 기준으로 확인한다.

## 검증과 범위

- [실행 가이드](실행가이드.md): 지도 저장, 키보드 텔레옵, cuVSLAM/Isaac Sim 실행.
- [검증 기록](VALIDATION.md): 실제 Linux 시험 결과와 아직 확인하지 않은 항목.
- `validation/`: 해당 시험의 구조화된 결과. 새 clone에서 자동 재검증한 결과는 아니다.
- 미로 LiDAR·직진·회전·정지·물리 벽 충돌·지도 저장을 확인했다.
- Isaac Sim 6.1의 작은 stereo 장면 → cuVSLAM GPU 추적을 확인했다.
- Isaac Sim stereo 장면은 Gazebo 미로/Waffle 물리 주행 장면과 별개다.
- 충돌 기둥은 60mm 바닥 외곽을 적용한 근사다. 미로 전체 완주·지도 재위치 추정·전체 Nav2는 별도 검증 대상이다.

cuVSLAM에는 NVIDIA GPU와 Isaac ROS 5.0 환경이 필요하다. `Dockerfile.cuvslam`은
Isaac ROS 5.0/Lyrical 및 NVIDIA APT 저장소가 구성된 **사용자 제공 베이스 이미지**를 확장한다.
검증 장비에서 사용한 베이스 이미지와 Isaac Sim 본체는 이 저장소에 포함하지 않는다.

```bash
# BASE_IMAGE에 준비한 Isaac ROS 5.0/Lyrical 베이스 이미지를 지정한다.
docker build --build-arg ISAAC_ROS_BASE="$BASE_IMAGE" \
  -f Dockerfile.cuvslam -t waffle-cuvslam:5.0-lyrical .
```

Isaac Sim은 별도 Python 3.12 가상환경 `isaacsim-venv/`에 6.1.0.0을 설치한다.
호스트 Lyrical rclpy를 쓰는 경우 venv에 `empy==3.3.4`, `lark==1.1.9`도 필요하다.
공식 설치 절차는 [Isaac Sim Python 설치](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html)를 따른다.
Lyrical CameraInfo ABI 우회는 제공된 평행 pinhole stereo 장면에 한정된다.

---

# TurtleBot3 Waffle / ROS 2 Lyrical 강의 예제

대상: Ubuntu 24.04 amd64, NVIDIA Isaac ROS 5.0의 Lyrical 환경, Isaac Sim 6.1.
2026-10-13 기준 소스를 sources.repos에 commit SHA로 고정했다.
이 파일은 실행을 위한 포팅 구성이다. 빌드·검증은 지정된 원격 Ubuntu 노트북에서 수행한다. 검증 범위와 남은 조건은 함께 제공한 검증기록을 확인한다. Windows에는 ROS를 설치하지 않는다.

Linux 노트북의 터미널별 실행 순서는 `실행가이드.md`를 먼저 확인한다.
추가한 Gazebo Waffle 예제는 `build_gazebo.sh`로 공식 Jazzy 소스를 Lyrical의
ros_gz/Gazebo 10에 빌드한다. Waffle의 `/cmd_vel` 형식은 이 Gazebo 예제에서
TwistStamped이며 아래 Isaac Sim 장면의 Twist 설정과 구분한다.

## 1. 호환 전략

- TurtleBot3 공식 브랜치는 main/humble/jazzy/noetic이며 Lyrical 전용 브랜치는 없다. Jazzy의 **turtlebot3_description만** Lyrical에서 빌드한다. Jazzy 바이너리나 Gazebo 플러그인을 복사하지 않는다.
- Cartographer와 cartographer_ros는 ROS 배포 목록에 Lyrical 릴리스가 있다. 공식 Ubuntu 대상은 26.04이므로 Ubuntu 24.04에서는 ROS 2 포크의 소스를 현재 Lyrical ABI로 빌드한다.
- Ubuntu 24.04/Lyrical은 NVIDIA Buildfarm 환경을 먼저 구성한다. 일반 ROS apt 저장소에서 Noble용 Lyrical이 제공된다고 가정하지 않는다.
- Isaac Sim native Lyrical bridge는 전용 빌드가 없는 실험적 경로다. 실제 CameraInfo 배열의 ABI 불일치를 확인해 제공 장면에서는 Lyrical rclpy 발행기로 우회했다. ROS 환경을 source한 셸에서 Sim을 실행한다. 다른 배포판의 rclpy/librcl을 섞지 않는다. 자세한 수정·검증 범위는 VALIDATION.md를 확인한다.
- Noble 기본 PCL은 1.14다. NVIDIA 저장소의 pcl_conversions 바이너리가 PCL 1.15를 요구하므로 perception_pcl의 pcl_conversions도 소스로 빌드한다. pcl_ros 전체를 가져와 실행하지 않는다.
- Cartographer ROS의 offline_node.cpp에 정의된 gflags가 공유 라이브러리와 online 실행 파일에 중복 등록되는 문제가 실제 실행에서 확인되었다. 제공 패치는 offline_node.cpp를 offline 실행 파일에만 넣어 모든 online 실행 파일의 중복 등록을 제거한다. 원본 저장소나 기존 워크스페이스에는 적용하지 않는다.

## 2. 준비 / 빌드

공식 Isaac ROS 5.0 Getting Started에 따라 GPU 드라이버, NVIDIA 저장소 및 개발 환경을 설치한다. `isaac-ros activate` 후 `echo $ROS_DISTRO`가 lyrical인지 확인한다. 그 환경에 다음을 설치한다.

```bash
sudo apt-get update
sudo apt-get install ros-lyrical-isaac-ros-cuvslam ros-lyrical-robot-state-publisher ros-lyrical-tf2-ros ros-lyrical-xacro ros-lyrical-rviz2 ros-lyrical-nav2-map-server python3-vcstool python3-colcon-common-extensions python3-rosdep
bash build_lyrical.sh
source install/setup.bash
```

rosdep을 처음 쓰면 `sudo rosdep init`을 한 번 실행하고 `rosdep update`를 실행한다. NVIDIA 저장소에서 의존 패키지를 찾지 못하면 저장소 설정을 먼저 확인한다. rosdep 실패가 있으면 빌드를 진행하기 전에 해소한다. 시스템을 Jazzy로 덮어쓰지 않는다. Cartographer만 준비할 때는 apt 목록에서 isaac_ros_cuvslam을 빼고 CPU 의존성부터 설치한다. cuVSLAM 준비 여부까지 검사하려면 `bash build_lyrical.sh cuvslam`을 실행한다.

## 3. Isaac Sim 6.1의 Waffle 장면

1. `ros2 run xacro xacro $(ros2 pkg prefix turtlebot3_description --share)/urdf/turtlebot3_waffle.urdf > /tmp/waffle.urdf`로 namespace 변수를 먼저 펼친다. `/tmp/waffle.urdf`를 URDF Importer로 가져온다. 패키지 경로가 해석되도록 터미널에서 workspace를 source한다. 고정 베이스를 해제하고 articulation root와 충돌체를 확인한다.
2. 차동구동 wheel joint: `wheel_left_joint`, `wheel_right_joint`. 고정한 URDF의 반지름 0.033m, 바퀴 중심 y=±0.144m에서 얻은 간격 0.288m를 사용하고, import 후 축·회전 부호를 확인한다. `/cmd_vel`은 geometry_msgs/Twist로 구독한다.
3. ROS 2 bridge와 Simulation Time을 활성화하고 `/clock`, `/joint_states`, `/odom`, `/scan`을 발행한다. `/scan`은 sensor_msgs/LaserScan, frame_id=`base_scan`, 수평 360도, URDF 라이다 위치를 사용한다. 센서 주기·거리 한계는 실제 장면 설정값과 Lua를 일치시킨다.
4. `ros2 launch waffle_slam_course model.launch.py`를 실행한다. joint_states는 Sim만 발행한다. robot_state_publisher가 URDF의 base_footprint→base_link 및 센서 고정 TF를 담당한다. Sim 전체 트리를 중복 발행하지 않는다.
5. Waffle의 기존 단안 카메라만으로 stereo cuVSLAM을 실행하지 않는다. 가상 stereo rig를 추가한다. **수업용 장면 가정**: 왼쪽 광학 원점은 base_link 기준 (0.12, 0.0475, 0.28)m, 오른쪽은 (0.12, -0.0475, 0.28)m, 평행 광축, baseline=0.095m. 실제 Waffle 장착 치수라고 주장하지 않는다.
6. 두 카메라를 동일 tick에 촬영한다. 640×480/30Hz 예시, 동일한 초점거리, 왜곡 0인 rectified 영상. 광학 frame은 `waffle_left_optical`, `waffle_right_optical`이다. 광학 z=전방, x=오른쪽, y=아래. base_link 기준 RPY=(-π/2,0,-π/2)로 고정 TF를 발행한다. 이 rig의 고정 TF는 Sim 또는 별도 static_transform_publisher 한 곳만 담당한다.
7. Publish ROS 2 Camera Helper로 좌/우 영상을 구성한다. 이 6.1/Lyrical 조합의 기본 CameraInfoHelper는 K/R/P 배열이 밀리므로 제공 장면의 `lyrical_camera_info.py`를 사용한다. 이 발행기는 **실제 USD 카메라 설정**에서 보정값을 계산하고 Image와 동일한 stamp/frame_id를 사용한다. 다른 센서에는 해당 장면의 해상도·내부/외부 보정을 다시 적용한다. 오른쪽 P[3]=-fx×0.095m인지 확인한다. 같은 영상을 두 토픽으로 복제하면 stereo가 아니다. 다른 ROS 메시지의 ABI까지 수정한 것은 아니다.

| 토픽 | 형식 | 역할 |
|---|---|---|
| /clock | rosgraph_msgs/Clock | 모든 노드 use_sim_time=true |
| /scan | sensor_msgs/LaserScan | Cartographer 2D |
| /odom | nav_msgs/Odometry | Cartographer wheel odometry |
| /joint_states | sensor_msgs/JointState | robot_state_publisher |
| /waffle/left/image_rect | sensor_msgs/Image | rectified 왼쪽 영상 |
| /waffle/right/image_rect | sensor_msgs/Image | rectified 오른쪽 영상 |
| /waffle/left/camera_info | sensor_msgs/CameraInfo | 왼쪽 K/D/R/P |
| /waffle/right/camera_info | sensor_msgs/CameraInfo | 오른쪽 K/D/R/P |

TF 발행자 선택이 중요하다. **Cartographer 모드**: Sim이 odom→base_footprint, Cartographer가 map→odom. **cuVSLAM 모드**: Sim의 odom TF를 끄고 cuVSLAM이 두 변환을 모두 발행한다. 두 SLAM launch를 동시에 실행하지 않는다. Sim의 ground truth odom을 cuVSLAM 추정 결과로 기록하지 않는다.

## 4. Cartographer 2D 비교 예제

```bash
bash preflight.sh cartographer
ros2 launch waffle_slam_course cartographer.launch.py
rviz2 --ros-args -p use_sim_time:=true
```

RViz Fixed Frame=map, Map=/map, LaserScan=/scan. 3m 사각 경로를 천천히 한 바퀴 돌고 시작 지점에서 루프 폐합을 관찰한다. `waffle_2d.lua`는 ROBOTIS Apache-2.0 설정에서 tracking_frame을 base_footprint로 바꿨다. IMU를 쓰지 않는 2D 모드다. 이것은 Cartographer 3D 예제가 아니다.

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/waffle_2d" --ros-args -p use_sim_time:=true
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$PWD/maps/waffle.pbstream', include_unfinished_submaps: true}"
```

PGM/YAML은 점유 격자다. pbstream은 SLAM 상태다. 저장 서비스 이름은 `ros2 service list -t`로 확인한다. 후보 start pose=(0,0,0), 속도 0.1m/s는 시연 가정이며 결과 수치가 아니다.

## 5. cuVSLAM stereo 3D 위치 추정

```bash
# 먼저 Sim의 odom→base_footprint TF 발행을 끈다.
bash preflight.sh cuvslam
ros2 launch waffle_slam_course cuvslam.launch.py
rviz2 --ros-args -p use_sim_time:=true
ros2 topic echo /visual_slam/status
```

Fixed Frame=map, TF와 `/visual_slam/tracking/slam_path`, `/visual_slam/tracking/vo_path`를 관찰한다. 2D 바닥 로봇이어도 카메라 위치 추정은 6DoF다. sparse landmarks와 dense collision map을 구분한다.

```bash
MAP_DIR=$(mktemp -d "$HOME/waffle_cuvslam_XXXXXX")
ros2 service call /visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/FilePath "{file_path: '$MAP_DIR'}"
echo "$MAP_DIR"
```

저장 성공과 파일 생성을 확인한다. 공식 튜토리얼의 재위치 추정 순서대로
cuVSLAM launch를 종료하고 다시 실행한 뒤, 같은 지도 경로와 근처 위치 힌트를
전달한다. 아래 서비스의 실제 재위치 추정은 아직 별도 검증하지 않았다.

```bash
ros2 service call /visual_slam/localize_in_map isaac_ros_visual_slam_interfaces/srv/LocalizeInMap "{map_folder_path: '$MAP_DIR', pose_hint: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

같은 셸에서 MAP_DIR를 유지한다. 다른 셸이면 실제 경로를 다시 지정한다. 힌트 (0,0,0)은 지도 시작점 근처에서만 쓴다. 저장마다 새 빈 디렉터리를 사용한다.

## 6. nvblox 연결은 다음 실습의 범위

37강 핵심 예제는 위 두 launch다. 41강에서 depth+cuVSLAM 추정 pose를 nvblox에 연결한다. nvblox는 거리장/메시를 만들며 cuVSLAM을 대체하지 않는다. `/static_map_slice`는 DistanceMapSlice이며 map_saver의 OccupancyGrid 입력과 다르다. 본 패키지는 nvblox/전체 Nav2 bringup을 자동 실행하지 않는다. 지상 Waffle은 Nav2용 2D 슬라이스를 사용한다.

## 7. 수업 전 인수검사

1. `ros2 run tf2_ros tf2_echo map base_footprint`와 `tf2_echo base_footprint waffle_left_optical`의 변환이 유효하다. base_link/base_footprint를 혼용하지 않는다.
2. `/clock`이 전진하고 노드의 use_sim_time이 true다. 영상 좌/우 stamp 차이는 5ms 이하를 목표로 확인한다. CameraInfo K와 baseline, optical axes를 실측/장면 설정과 대조한다.
3. 직진 명령으로 Waffle이 전방으로 움직이고, 양의 angular.z로 반시계 회전한다. 멈춤 명령이 즉시 반영된다. 외부 실물 모터는 연결하지 않는다.
4. Cartographer /map이 증가하고 루프 경로를 재방문해 벽 중복을 확인한다. cuVSLAM tracking/status, 6DoF pose, save/localize service 성공을 확인한다.
5. 텍스처 있는 방/흰 벽/빠른 회전 조건을 같은 경로에서 비교한다. 실패 횟수, 루프 전후 위치 오차, 영상 주기, 설정과 commit SHA를 report.md에 기록한다.
6. 추후 Nav2 연결 시 Lyrical 기본 TwistStamped와 Sim Twist 구독을 맞춘다. 사용하는 Nav2 서버의 enable_stamped_cmd_vel=false 설정 또는 명시적 변환 노드 중 하나를 택한다.

## 출처

- https://github.com/ROBOTIS-GIT/turtlebot3/tree/jazzy
- https://raw.githubusercontent.com/ros/rosdistro/master/lyrical/distribution.yaml
- https://github.com/ros2/cartographer/tree/ros2
- https://github.com/ros2/cartographer_ros/tree/ros2
- https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html
- https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/visual_slam/cuvslam/tutorial_isaac_sim.html
- https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html

ROBOTIS Lua 원본 저작권/Apache-2.0 고지를 보존했다. 다른 파일은 강의용 새 구성이다.
