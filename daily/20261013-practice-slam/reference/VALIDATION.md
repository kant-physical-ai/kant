# 원격 검증 기록

2026-10-13, Ubuntu 24.04 / ROS 2 Lyrical에서 실행했다.
실제 Isaac Sim 영상과 합성 입력 검사를 구분한다.

| 검사 | 결과 | 범위 |
|---|---|---|
| Cartographer 소스 빌드 | 통과 | pcl_conversions 소스 및 offline gflags 분리 패치 적용 |
| Cartographer 핵심 CTest 4개 | 통과 | probability grid, points processor, range collator, rigid transform |
| Cartographer 합성 방 입력 | 통과 | 132×132셀, 0.05m, 관측 11,116셀, 점유 266셀 |
| Cartographer 지도 저장 | 통과 | 실제 PGM/YAML 생성 |
| Waffle 모델 해석 | 통과 | 조인트 12개, 메시 5개, 바퀴 조인트와 메시 경로 확인 |
| cuVSLAM 패키지·launch·Python 의존성 | 통과 | Isaac ROS 5.0, Lyrical |
| cuVSLAM GPU 추적 | 통과 | 합성 평면 스테레오 입력, 유효한 odometry 179개 |
| cuVSLAM 지도 저장 | 통과 | FilePath 서비스 성공 및 data.mdb 생성 |
| 기존 OpenArm 환경 | 통과 | 기존 패키지 경로와 sim.launch.py 인자 해석 확인 |
| 기존 OpenArm Gazebo 서버 | 부분 통과 | 독립 domain 177, /clock 263개, /joint_states 43개; 컨트롤러 5개 active, 종료 bridge 오류 있음 |
| Waffle Gazebo Lyrical 빌드 | 통과 | 공식 Jazzy commit 45633014, 설치된 Gazebo 10.5 ABI로 빌드 |
| Waffle Gazebo + Cartographer | 통과 | LiDAR 145개, odometry 721개, 지도 15개, 111×114셀 |
| Waffle Gazebo 주행·정지·저장 | 통과 | 직진 0.7766m, 양의 yaw 0.9655rad, 정지 및 PGM/YAML 저장 |
| 키보드 텔레옵 | 통과 | teleop_twist_keyboard 2.4.1, 독립 domain 178에서 PTY i/j/k 입력과 TwistStamped 전진·회전·정지, Ctrl+C 종료 확인 |
| Isaac Sim 6.1 시작·bridge 활성화 | 통과 | CPU 스레드 4, 단일 GPU, native Lyrical rclpy 로드 |
| Isaac Sim 스테레오 → cuVSLAM | 통과 | 실제 렌더링 120쌍, odometry 130개, 추정 x 변화 0.3832m |
| Sim stereo 보정·시계·타임스탬프 | 통과 | fx/fy=600, baseline=0.095m, 시계 전진, 중복·역순 이미지 stamp 0건 |
| Waffle Isaac Sim 물리 주행 | 미검증 | Gazebo 주행 검증과 별도 |
| cuVSLAM 저장 지도 재위치 추정 | 미검증 | 지도 저장 성공과 별도 검사 |

## GPU와 재부팅

사용자 승인 후 재부팅했다. 커널은 7.0.0-38-generic, NVIDIA 드라이버는
595.99.02이며 NVML 버전 불일치가 해소됐다. GPU는 RTX 3050 Laptop,
VRAM 4096MiB, 시스템 RAM은 약 16GiB다.

Isaac Sim 공식 최소 사양보다 낮은 하드웨어다. 작은 장면 검사를 통과해도
대형 장면·장시간 동시 실행이나 수업 전체 성능을 보장하지 않는다.
https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html

기존 OpenArm Gazebo 서버는 별도 domain 177 및 독립 GZ_PARTITION에서
/clock, /joint_states와 컨트롤러 5개 active를 확인했다. 관절 궤적 명령과
GUI는 검사하지 않았다. 정상 준비 후 종료해도 parameter_bridge SIGSEGV가
기록됐다. 첫 검사에서는 조기 종료에 spawner 지연 및 잔여 gz-sim-main도
발생해 검사에서 시작한 해당 PID만 정리했다. 시작·토픽·컨트롤러 준비 성공과
정상 종료는 구분한다.
이 검사는 Waffle Gazebo 주행 검사가 아니다.

## 합성 cuVSLAM 검사

cuVSLAM 라이브러리는 15.0.0+33da606, ROS 패키지는
5.0.0-0noble.20260918011121000이다. 로그에서 `Use use_gpu: true`와
tracker 초기화 성공을 확인했다. 시험 입력은 640×480 mono8, 10Hz다.
기본 30Hz 기준의 image_jitter_threshold_ms=34 경고가 발생했으므로
이 결과를 30Hz 실시간 성능 측정으로 사용하지 않는다.

마지막 위치는 약 (0.00005, -0.30307, -0.00183)m였다.
생성한 입력의 추적·메시지·저장 경로를 확인한 결과이며, 실제 카메라
정확도·루프 폐합·Waffle 주행 성능을 측정한 결과는 아니다.

## 재현용 검사 파일

- `synthetic_cartographer_check.py`: 합성 LiDAR → Cartographer → 지도 저장
- `prepare_waffle_urdf.py`: xacro 처리, 메시 경로 확인, import용 URDF 준비
- `synthetic_cuvslam_check.py`: 합성 stereo → GPU 추적 → 지도 저장
- `isaacsim_stereo_bridge_check.py`: 작은 실제 Sim stereo/clock 발행 장면
- `observe_sim_cuvslam.py`: Sim 영상·보정값·odometry 검사

뒤의 두 스크립트도 실제 실행했다. Dockerfile.cuvslam은
이 원격 머신에 기존 설치된 openarm-isaac-cumotion:5.0 이미지를 기반으로
한 검증용 파일이다. 일반 실습 환경에서는 공식 Isaac ROS 5.0 환경을
먼저 준비하고 launch 파일을 실행한다.

## Isaac Sim / Lyrical 호환 수정

Isaac Sim 6.1의 Lyrical backend는 전용 빌드가 없는 실험적 경로다.
기본 ROS2CameraInfoHelper로 받은 K/R/P가 원소 하나씩 밀린 것을 실제 메시지에서
확인했다. 설치된 Lyrical C primitive Sequence에는 is_rosidl_buffer와
owns_rosidl_buffer가 추가되어 이전 backend와 메모리 배치가 다르다.

제공한 장면은 native helper의 CameraInfo 발행을 제거하고
`lyrical_camera_info.py`에서 Lyrical rclpy로 발행한다. 실제 USD 카메라의
초점거리·aperture·상대 위치로 보정값을 계산하고, 영상 header를 그대로 사용한다.
잘못 받은 행렬을 임의로 옮기거나 영상 시각을 다시 부여하지 않는다.
검증한 카메라는 평행·무왜곡 pinhole stereo다. 모든 ROS 메시지의 bridge ABI를
수정한 것이 아니므로 다른 메시지 유형에는 별도 호환 검사가 필요하다.

physics dt=1/60, 카메라 frameSkipCount=1로 발행해 중복 stamp를 해소했다.
최종 검사에서 좌/우 영상 136/120개, 동기화 120쌍, odometry 130개를 수신했다.
초기 구간에 66.7ms 및 166.7ms 프레임 간격 경고가 있었다. 따라서 30Hz
실시간 성능이나 무손실 전달을 보장하는 검사로 해석하지 않는다.

기본 CPU 16스레드의 첫 준비에서는 프로세스 메모리가 약 14GB까지 증가해
중단하고 4스레드로 낮췄다. `empy==3.3.4`, `lark==1.1.9`를 Sim venv에
추가해 시스템 rclpy import도 확인했다. host ROS 패키지는 변경하지 않았다.

## Twin Closed Maze B 월드 교체

2026-10-13 업로드한 STL을 mm/Y-up에서 m/Z-up으로 변환했다.
전체 외곽은 4.464 × 6.364 × 0.390m다. 원본 SHA-256과 변환 정보는
`src/waffle_slam_course/models/twin_closed_maze/source.json`에 기록했다.
1mm 꼭짓점 정리로 2,919,996개 삼각형을 580,100개로 줄였다.
충돌은 판넬 40개 및 기둥 83개의 박스를 사용한다. 기둥 충돌은 60mm 바닥
외곽을 높이 전체에 적용한 보수적인 근사이며 정밀 접촉 시험용 형상이 아니다.

Linux 실제 Gazebo 검사에서 scan 257개, odom 1,283개, map 26개를 수신했다.
직진·회전·정지와 PGM/YAML 저장을 통과했다. 시작 지점의 북쪽 벽 거리는
0.3899m였다. 외벽 방향 전진 명령 후 Gazebo 실제 위치는 y=3.08204m로
외벽 y=3.152m 앞에서 멈췄다. 바퀴 odometry는 충돌 시에도 증가하므로
충돌 판정은 `gz model -m waffle -p`로 얻은 실제 위치를 사용했다.

저장된 지도는 시작 구역의 102 × 35셀 관측 결과이며 미로 전체를 탐색한 지도가 아니다.
새 월드에서 GUI 키보드 주행 및 전체 경로 완주는 별도 확인 대상이다.
Isaac Sim stereo 검사 장면은 기존 장면을 유지한다.
