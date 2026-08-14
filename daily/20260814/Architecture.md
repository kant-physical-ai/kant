```text
[바퀴 노드]   ───( /wheel_odom )───┐
[라이다 노드] ───( /laser_odom )───┼──> [EKF 융합 노드] ───( /odom 토픽 & TF )──> [SLAM / Nav2]
[카메라 노드] ───( /visual_odom )──┘

-------------

[바퀴 엔코더] ──┐
[IMU 센서]   ──┼─> [1단계: EKF 노드] ───(보정된 /odom)───┐
[라이다 Odom] ──┘                                        │
                                                         ▼
[라이다 /scan] ──────────────────────────────────> [2단계: SLAM 노드] ───(지도 /map & map->odom TF)

```
```text

[EKF 노드] ──────( /odom 토픽 발행 )──────> [SLAM 노드]
    │                                             │
    ▼ (TF 방송)                                   ▼ (TF 방송)
 ( odom -> base_link )                       ( map -> odom )
 
```



# EKF
- 이렇게 발행해두면, 뒤에서 robot_localization (EKF 노드) 같은 합성 노드가 아래처럼 깔끔하게 수신(Subscribe)받아서 융합할 수 있어.


# 3. 코드 변경 없이 토픽 이름 바꾸는 팁 (리매핑)
- 만약 오픈소스 패키지나 남이 짜둔 코드가 내부적으로 /odom으로 고정되어 있어서 파이썬 코드를 수정하기 힘들다면?
- 우리가 제일 처음에 다뤘던 ROS 2 리매핑(Remapping)을 써서 실행할 때 토픽 이름을 바꿔주면 돼!
```shell
# 바퀴 오도메트리 노드를 켤 때 /odom을 /wheel_odom으로 리매핑
ros2 run my_wheel_pkg wheel_node --ros-args -r /odom:=/wheel_odom

# 비주얼 오도메트리 노드를 켤 때 /odom을 /visual_odom으로 리매핑
ros2 run my_visual_pkg visual_node --ros-args -r /odom:=/visual_odom
```




|프레임|의미|비유|특징|
|---|---|---|---|
|map|세계/지도 좌표계|"지구 상의 건물/벽 위치"|절대로 움직이지 않는 절대적 기준. 오차가 전혀 없음.|
|odom|주행/출발점| 좌표계"로봇이 출발한 지점"|단기적으로 부드럽지만, 바퀴 미끄러짐 등으로 오차가 누적됨.|
|base_footprint|로봇 차체 좌표계|"현재 로봇 그 자체"|로봇의 이동에 따라 실시간으로 계속 움직임.|


```text
[ map ] (지도/절대 좌표계)
   │
   │  <--- SLAM / AMCL 이 계산해서 보정하는 TF (map -> odom)
   ▼
[ odom ] (바퀴/IMU/라이다 오도메트리가 융합된 보정 좌표계)
   │
   │  <--- EKF / Wheel Odometry Node 가 발행하는 TF (odom -> base_footprint)
   ▼
[ base_footprint ] (로봇 차체 중심)
   │
   │  <--- robot_state_publisher 가 발행하는 고정 TF
   ▼
[ base_scan ] (라이다 센서 위치)
```

# 요약
- base_footprint: "나(로봇) 지금 어디 보고 있어?"
- odom: "나 출발한 곳에서 바퀴 얼마나 굴러왔어?"
- map: "됐고, 실제 지도 상의 이 벽 기준으로 너 지금 딱 여기 있어!"