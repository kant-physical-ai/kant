# 2026-10-13 SLAM 실습 (ROS2 Lyrical + Gazebo + slam_toolbox + RViz)

## 0. 사전 설치 (1회, 비밀번호 필요)

```bash
bash 00_install.sh
```

## 1. 전체 실행 (권장)

```bash
bash run_all.sh turtlebot3_world
# 또는: bash run_all.sh turtlebot3_house
```

탭 4개가 순서대로 뜸:
1. Gazebo (gz sim) + TurtleBot3 Burger 스폰
2. SLAM (slam_toolbox online_async, use_sim_time=true)
3. RViz2 (slam_toolbox 기본 config)
4. Teleop (키보드 조종, 이 탭에 포커스 두고 `i/j/k/l/,` 조작)

## 2. 낱개 실행 (디버깅용)

```bash
bash 01_gazebo.sh turtlebot3_world   # gazebo
bash 02_slam.sh                      # 매핑 노드
bash 03_rviz.sh                      # rviz 보기
bash 04_teleop.sh                    # 조종
bash 05_save_map.sh ./tb3_map        # 지도 저장 -> tb3_map.pgm/.yaml
bash 99_kill_all.sh                  # 전부 종료 (화면 없이 남은 Gazebo 서버 포함)
```

Gazebo 창이나 터미널을 닫아도 화면 없는 서버(`gz-sim-main -s`)가 남을 수 있다.
다시 시작하기 전에 `99_kill_all.sh`로 정리하고 01부터 실행한다.

## 3. 전체 흐름 (데이터가 이렇게 돎)

```
01_gazebo.sh (gz sim + TurtleBot3)
  │  /scan (LiDAR), /odom, tf
  ▼
02_slam.sh 또는 02_cartographer.sh (지도제작자 — 둘 중 하나만)
  │  slam_toolbox → /map 직접 발행
  │  cartographer_node → 서브맵 → occupancy_grid_node → /map 발행
  ▼
03_rviz.sh (보는 도구 — 발행 안 함, 구독만)
  │  /map 구독해서 화면에 표시 (Fixed Frame: map)
  ▼
04_teleop.sh (조종 — 로봇을 움직여서 지도를 넓힘)
  │  /cmd_vel 발행 → Gazebo 로봇 이동 → 새 /scan → 지도 확장
  ▼
05_save_map.sh (저장 — 매핑 끝나면 1회)
     /map 저장 → .pgm + .yaml
```

- 지도제작자가 꺼져 있으면 `/map`이 없어서 RViz에 아무것도 안 나옴.
- RViz는 보기만 하니까 꺼져 있어도 매핑은 계속 됨.
- 실행 순서: 01 → 02 → 03 → 04 → (다 돌았으면) 05.

## 4. RViz에서 볼 것

- Fixed Frame: `map` (없으면 `odom`)
- Add → By topic → `/map` (Map) → Topic Reliability를 맞춰야 할 수 있음
- Add → TF, LaserScan(`/scan`), RobotModel

## 4. Cartographer 코스 (추가됨)

apt에 `cartographer_ros` 바이너리가 없어서 소스 빌드 방식:

```bash
bash 00_install.sh               # cartographer lib 포함
bash 00_build_cartographer.sh    # 소스 빌드, 10~20분
bash run_cartographer.sh turtlebot3_world
# 낱개: bash 02_cartographer.sh
# 저장: bash 05_save_map.sh ./carto_map
```

구성: `config/turtlebot3_lds_2d.lua` + `launch/cartographer_tb3.launch.py`
(Gazebo용 tracking_frame=base_footprint)

## 5. 트러블슈팅

- `TURTLEBOT3_MODEL`은 `burger` 고정 (01 스크립트에 포함).
- `ROS_DOMAIN_ID=9` 고정 (현재 환경값).
- Gazebo가 안 뜨면: `gz sim --version` / `ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-v2"` 로 gz 자체 테스트.

## 6. TF: 왜 오차가 쌓인 `odom→base_footprint`를 그대로 두나

자세한 그림과 계산 예시는 [tf-odom-map-설명.md](tf-odom-map-설명.md) 참고.

오차를 남겨 두는 게 아니라, `odom→base_footprint`에는 **바퀴가 잰 원본 측정값**을 고치지 않고 두고,
SLAM 보정은 그 위 칸(`map→odom`)에 따로 얹는 구조다.

| 변환 | 성질 | 쓰는 곳 |
|---|---|---|
| `odom → base_footprint` | 매끄럽다, 점프 없음, 오래 가면 틀어짐 | 속도 제어, 장애물 회피 (가까운 움직임) |
| `map → odom` | 정확하다, 대신 보정 때 점프 | 전역 위치 보정 |
| `map → base_footprint` | 둘의 합성: 매끄럽고 정확함 | 지도 위 로봇 위치 |

보정을 odom에 직접 넣지 않는 이유:

1. **점프하면 제어가 망가진다.** 0.2m/s로 가다가 0.5m 보정이 odom에 들어가면,
   30Hz 한 주기(0.033초)에 0.5m 이동, 즉 **15m/s**로 계산된다. 급정지, 휘청임, costmap 번짐.
2. **짧은 구간에서는 odom이 더 정확하다.** 오차는 오래 쌓여야 커진다.
   0.1초, 1초 사이 움직임은 바퀴 값이 SLAM보다 매끄럽다.
3. **SLAM은 느리고 멈출 수 있다.** odom은 30~100Hz(이 실습 30Hz),
   SLAM은 scan 주기 5~10Hz에 loop closure는 수 초. 특징 없는 복도에서 실패하거나 노드가 죽어도 odom 주행은 계속돼야 한다.
4. **SLAM이 odom을 입력으로 쓴다.** Cartographer는 `/odom`으로 다음 scan 위치를 예측한다(`use_odometry = true`).
   보정값을 odom에 써 넣으면 자기 출력을 다시 입력으로 받는 순환이 생긴다.
5. **TF는 부모가 하나뿐이다.** `odom→base_footprint`는 이미 Gazebo가 낸다.
   SLAM까지 같은 변환을 내면 충돌해서 로봇이 두 자리 사이를 튄다. 그래서 각자 한 칸씩 맡는다.

비유: odom = 자동차 트립미터(매끄럽지만 조금씩 틀림, 숫자를 손으로 고치지 않음),
SLAM = GPS(가끔 툭 고쳐 주지만 터널에서 끊김), `map→base` = 내비 화면의 내 위치.

### 무빙워크로 보면 (반대로 걷기)

| 무빙워크 | TF |
|---|---|
| 내 발걸음으로 센 거리 (무빙워크 바닥 기준) | `odom → base_footprint` |
| 바닥이 건물에 대해 움직인 양 | `map → odom` |
| 건물 안 내 실제 위치 | `map → base_footprint` |

예시: 발걸음으로는 1.1m 걸었는데(헛걸음 0.1m 포함) 바닥이 **반대 방향으로 0.1m** 밀려서
건물 기준으로는 1.0m. 바닥이 반대로 움직이는 게 곧 오차를 상쇄하는 보정이다.

비유와 다른 점:

1. **바닥은 대부분 멈춰 있다가 툭 움직인다.** `map→odom`은 SLAM이 오차를 알아챌 때만 바뀐다.
   그래서 RViz에서 odom 축이 흐르지 않고 툭툭 이동한다.
2. **바닥이 회전도 한다.** `map→odom`에는 회전이 들어갈 수 있다(예: −10°).
   그러면 "발걸음 + 바닥 이동"을 단순히 더할 수 없고 회전까지 합성해야 한다.
3. **실제로 움직이는 건 없다.** 로봇은 그대로고, odom 좌표계를 map 위 어디에 얹을지라는 숫자만 바뀐다.
