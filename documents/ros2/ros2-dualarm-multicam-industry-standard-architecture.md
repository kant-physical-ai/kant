# 듀얼암 + 멀티카메라 모바일 매니퓰레이터 — 현업 표준 ROS 2 아키텍처

> [ros2-dualarm-multicam-vlm-rl-architecture.md](./ros2-dualarm-multicam-vlm-rl-architecture.md)의 연구형 구조(Nav2/MoveIt 뒤 RL 잔차보정 + 시리얼 `mcu_bridge`)를 **현업 양산형**으로 바꾼 버전입니다. 하드웨어(4륜 베이스 + 듀얼암 + RealSense 3대)는 동일 가정, 실행경로만 바꿉니다.
>
> 도형/이모지 규칙은 자매 문서와 동일: 💊 알약=노드, ⬡ 육각=하드웨어, 🚩 깃발=커스텀, 🛢️ 원기둥=데이터/지도, 📨 토픽·🎬 액션·🛎️ 서비스·🔌 필드버스(EtherCAT/CAN).
>
> 핵심 차이 한 줄: **RL을 실시간 경로에서 제거, VLM을 실시간 루프에서 격리, `mcu_bridge`를 `ros2_control`로 교체, 안전 체인을 추가.**

---

## 0. 전체 구조 한눈에 보기

```mermaid
flowchart TB
    Human((("👤 작업자/관제<br/><i>HMI·MES·태블릿</i>")))

    subgraph PERC["👁️ 인지 (격리됨, 실시간 제어와 분리)"]
        Fusion>"🚩 <i>(커스텀)::</i><b>multicam_3d_segmentation_node</b><br/><i>batch YOLO + 3D역투영 + Voting</i>"]
        Track>"🚩 <i>(커스텀)::</i><b>object_tracking_node</b><br/><i>ID 유지 (Apple_777)</i>"]
    end

    subgraph LOC["🗺️ 위치추정 & 지도"]
        SLAM("<i>slam_toolbox::</i><b>slam_toolbox</b><br/><i>2D LiDAR 기반 (양산 표준)</i>")
        AMCL("<i>nav2_amcl::</i><b>amcl</b>")
        EKF("<i>robot_localization::</i><b>ekf_node</b><br/><i>odom+IMU 퓨전</i>")
        RSP("<i>robot_state_publisher::</i><b>robot_state_publisher</b>")
    end

    subgraph TASK["🧠 임무 관리 (결정적, VLM은 자문만)"]
        Coord>"🚩 <i>(커스텀)::</i><b>mission_manager_node</b><br/><i>BT/상태기계, 순서·감시만<br/>(구 coordinator의 결정적 버전)</i>"]
        VLM>"🚩 <i>(커스텀)::</i><b>vlm_task_advisor_node</b><br/><i>비실시간 자문, 직접 액션호출 금지</i>"]
    end

    subgraph NAVL["🧭 Nav2 표준 체인"]
        BT("<i>nav2_bt_navigator::</i><b>bt_navigator</b>")
        Planner("<i>nav2_planner::</i><b>planner_server</b>")
        Controller("<i>nav2_controller::</i><b>controller_server</b><br/><i>DWB/MPPI/RPP</i>")
        Smoother("<i>nav2_velocity_smoother::</i><b>velocity_smoother</b>")
        CM("<i>nav2_collision_monitor::</i><b>collision_monitor</b><br/><i>안전 정지 최후방어선</i>")
        Behavior("<i>nav2_behaviors::</i><b>behavior_server</b>")
    end

    subgraph MANIPL["🦾 MoveIt 표준 체인"]
        MoveGroup("<i>moveit_ros_move_group::</i><b>move_group</b>")
        Servo("<i>moveit_servo::</i><b>servo_node</b><br/><i>미세보정·수동조그용</i>")
    end

    subgraph CTRL["⚙️ 실시간 제어 (ros2_control)"]
        R2C("<i>controller_manager::</i><b>controller_manager</b><br/><i>diff_drive + joint_trajectory + gripper + servo</i>")
        HW("<i>hardware_interface::</i><b>mobile_manipulator_hw</b><br/><i>EtherCAT/CAN</i>")
    end

    subgraph HWL["⬡ 하드웨어"]
        LiDAR{{"2D Safety LiDAR ×1~2<br/>(SLAM+안전겸용)"}}
        Cams{{"RealSense ×3<br/>(인지 전용)"}}
        Base{{"4륜 베이스 + 서보드라이버"}}
        Arms{{"듀얼암 ×2 + 그리퍼 + F/T센서"}}
        Safety{{"비상정지·안전PLC·범퍼"}}
    end

    Human -->|"🛎️ REST/OPC-UA<br/>작업지시 (가반하중·속도제한 포함)"| Coord
    Coord -->|"🎬 NavigateToPose"| BT
    Coord -->|"🎬 MoveGroup / FollowJointTrajectory"| MoveGroup
    Coord -->|"🛎️ 자문요청 (비실시간)"| VLM
    VLM -->|"📨 후보 sub-goals (승인 전까지 실행안됨)"| Coord
    Track -->|"📨 /tracked_objects<br/>(목표좌표 조회용)"| Coord

    BT -->|"🎬 ComputePathToPose"| Planner
    BT -->|"🎬 FollowPath"| Controller
    Controller -->|"📨 /cmd_vel"| Smoother
    Smoother -->|"📨 /cmd_vel_smoothed"| CM
    CM -->|"📨 /cmd_vel_final"| R2C
    R2C -->|"🔌 EtherCAT/CAN<br/>wheel vel"| HW
    HW -->|"🔌"| Base

    MoveGroup -->|"🎬 FollowJointTrajectory"| R2C
    Servo -->|"📨 joint_cmd (제한 내)"| R2C
    R2C -->|"🔌 EtherCAT/CAN<br/>joint pos/vel/eff"| HW
    HW -->|"🔌"| Arms

    LiDAR -->|"📨 /scan"| SLAM
    LiDAR -->|"📨 /scan"| CM
    LiDAR -->|"📨 /scan"| AMCL
    Cams -->|"📨 image_raw+depth+info"| Fusion
    Fusion -->|"📨 /segmented_objects_3d"| Track
    Fusion -->|"📨 PointCloud (costmap 장애물층용만)"| Controller
    HW -->|"📨 /joint_states, /wheel_odometry"| EKF
    HW -->|"📨 /joint_states"| RSP
    EKF -->|"📨 /odometry/filtered"| BT
    SLAM -->|"📨 /map"| Planner
    Safety -->|"🔌 HW e-stop → R2C halt"| R2C

    classDef custom fill:#ffcdd2,stroke:#c62828,color:#1b1b1b;
    classDef provided fill:#c8e6c9,stroke:#2e7d32,color:#1b1b1b;
    classDef phys fill:#fff9c4,stroke:#f9a825,color:#1b1b1b;
    classDef data fill:#e1bee7,stroke:#6a1b9a,color:#1b1b1b;
    class Fusion,Track,Coord,VLM custom;
    class SLAM,AMCL,EKF,BT,Planner,Controller,Smoother,CM,Behavior,MoveGroup,Servo,R2C,HW provided;
    class LiDAR,Cams,Base,Arms,Safety phys;
    class RSP data;
```

**읽는 법:** 실시간 주행/조작 경로에는 학습 모델이 없음. VLM·YOLO는 경로 밖에서 자문·조회만 한다.

---

## 1. 하드웨어 & ros2_control & TF — 연구형과 가장 크게 달라지는 곳

| 항목 | 연구형(VLM-RL 문서) | 현업(본 문서) | 이유 |
|---|---|---|---|
| 모터 인터페이스 | `mcu_bridge_node` 시리얼 직접 read/write | `controller_manager` + `hardware_interface` → EtherCAT/CAN 서보드라이버 | 실시간성·동기·진단·안전정지. 시리얼은 지터·대역폭 한계 |
| 베이스 제어기 | RL이 `/cmd_vel` 가로채서 보정 | `diff_drive_controller` + `velocity_smoother` + `collision_monitor` | 결정적 속도프로파일 + 최후방어선 |
| 팔 제어기 | RL이 궤적을 토크로 덮어씀 | `joint_trajectory_controller` + `gripper_controller` (+ 필요시 `admittance_controller`) | MoveIt 제한(속도·가속·토크) 우회 금지 |
| 안전 | 없음 (Behavior 복구만) | Safety LiDAR + `collision_monitor` + HW e-stop + 안전PLC | ISO 10218 / ISO 3691-4 대응 |
| TF | `robot_state_publisher`만 | 동일 + `ekf_node(map→odom→base_footprint→base_link)` | `odom`을 EKF 퓨전값으로 통일 |

```mermaid
flowchart LR
    map((map)) --> odom((odom)) --> bf((base_footprint)) --> bl((base_link))
    bl --> la1([left_arm_link1]) --> laee([left_ee]) --> lcam(["left_cam_link (동적)"])
    bl --> ra1([right_arm_link1]) --> raee([right_ee]) --> rcam(["right_cam_link (동적)"])
    bl --> tcam(["topdown_cam_link (고정)"])
    bl --> lidar(["lidar_link (고정)"])
```

> Eye-in-Hand(암 카메라 동적 TF) / Eye-to-Hand(상부 고정 TF) 원리는 동일. 달라지는 건 `odom` 소스가 `mcu_bridge` 엔코더 직접값이 아니라 `ekf_node` 퓨전값이라는 점.

---

## 2. 인지 파이프라인 — 구조는 유지, 연결만 격리

| 노드 | 구독 | 발행 | 메시지 타입 |
|---|---|---|---|
| `realsense2_camera_node` ×3 | — | `/{ns}/color/image_raw`, `/{ns}/aligned_depth_to_color/image_raw`, `/{ns}/color/camera_info` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` |
| `multicam_3d_segmentation_node` 🚩 | 위 9개 + `/tf` | `/segmented_objects_3d` | `vision_msgs/Detection3DArray` (frame_id=`base_link`) |
| `object_tracking_node` 🚩 | `/segmented_objects_3d` | `/tracked_objects` | 커스텀 (`id`, `pose`, `velocity`, `class`, `volume`) |

현업식 격리 3원칙:

1. **인지 → SLAM 직접 주입 금지.** 연구형처럼 분할 pointcloud를 `rtabmap`에 넣지 않는다. 양산 SLAM(`slam_toolbox`)은 2D LiDAR `/scan` 기반. 카메라 pointcloud는 `costmap` 장애물층 보조 입력으로만 사용 (또는 미사용).
2. **인지 → 제어 직접 연결 금지.** YOLO/Tracking 출력이 `/cmd_vel`·`joint_cmd`를 직접 만들지 않는다. `mission_manager`가 좌표 조회용으로만 쓴다.
3. **GPU 분리 권장.** YOLO 배치 추론과 주행 컨트롤러를 같은 Jetson에 두면 지터 전파. 분리 보드 또는 QoS·우선순위 격리.

---

## 3. 주행 — Nav2 표준 체인 (RL 없음)

```mermaid
flowchart LR
    BT("<b>bt_navigator</b>") -->|"🎬 FollowPath"| CT("<b>controller_server</b><br/>DWB/MPPI")
    CT -->|"📨 /cmd_vel"| VS("<b>velocity_smoother</b><br/>가감속 제한")
    VS -->|"📨 /cmd_vel_smoothed"| CMON("<b>collision_monitor</b><br/>스캔 기반 정지/감속")
    CMON -->|"📨 /cmd_vel_final"| R2C("<b>diff_drive_controller</b>")
```

- `planner_server`: `SmacPlanner` / `NavFn`, `/map` 기반 전역경로.
- `controller_server`: DWB 또는 MPPI 그대로. 파라미터는 `nav2_params.yaml`로 관리·버전관리.
- `velocity_smoother`: 가반하중·적재 유무에 따라 프로파일 전환 (빈손/파지중 분리 — `mission_manager`가 파라미터 set).
- `collision_monitor`: Safety LiDAR `/scan` 직결. BT·Controller를 우회하는 최후 정지선이라 반드시 HW e-stop과 연동.

---

## 4. 조작 — MoveIt 표준 체인 (토크 오버라이드 없음)

```mermaid
flowchart LR
    MG("<b>move_group</b><br/>IK/궤적계획") -->|"🎬 FollowJointTrajectory<br/>(control_msgs)"| R2C("<b>joint_trajectory_controller ×2</b>")
    SV("<b>servo_node</b><br/>미세보정") -->|"📨 제한준수 joint_cmd"| R2C
    FT(["F/T센서"]) -->|"📨 /wrench"| ADM("<b>admittance_controller</b> (선택)")
    ADM --> R2C
```

- 듀얼암은 SRDF에 `left_arm` / `right_arm` / `both_arms` 그룹 분리. 양팔 협조 작업만 `both_arms` 사용.
- 파지 검증은 연구형과 동일하게 `mission_manager`가 `object_tracking` 재조회로 수행 (MoveIt result는 모션완료 의미뿐). 단, 판단 결과는 미리 정의된 복구 BT(재파지 각도 테이블)로 처리 — VLM 호출 아님 (6장 참고).
- 힘제어 필요시 RL 토크 보정 대신 `admittance_controller` + F/T센서 사용. 인증 가능한 선형 컴플라이언스.

---

## 5. 임무 관리 — VLM은 자문, 실행은 결정적 BT

연구형: `vlm_task_planner → coordinator` 가 실시간 루프 안에 있음.
현업: `mission_manager(BT)` 가 실행 전권, `vlm_task_advisor`는 비실시간 자문.

```mermaid
sequenceDiagram
    participant HMI as 👤 HMI/MES
    participant MGR as 🚩 mission_manager (BT)
    participant TRK as 🚩 object_tracking
    participant BT as bt_navigator
    participant MG as move_group
    participant VLM as 🚩 vlm_task_advisor (비실시간)

    HMI->>MGR: 🛎️ 작업지시 (품목·수량·속도제한)
    MGR->>TRK: 📨 목표 ID 좌표 조회 (Apple_777)
    MGR->>BT: 🎬 NavigateToPose
    BT-->>MGR: 🎬 성공/실패
    MGR->>MG: 🎬 pick(Apple_777)
    MG-->>MGR: 🎬 모션완료
    MGR->>TRK: 📨 파지검증 재조회
    alt 파지 실패 (정의된 횟수 내)
        MGR->>MGR: 미리정의된 복구 (재파지 테이블·우회경로)
    else 복구 소진 or 미정의 상황
        MGR->>VLM: 🛎️ 자문요청 + 스냅샷 (로봇은 정지대기)
        VLM-->>MGR: 📨 후보안 (작업자가 승인해야 실행)
        MGR->>MGR: 승인된 것만 BT로 실행
    end
```

원칙:

- VLM 출력은 **승인 전 실행 금지 (human-in-the-loop 또는 rule-gate)**.
- 정상계·1차복구는 VLM 없이 100% 동작해야 함. VLM 장애 = 작업일시정지이지 폭주가 아님.
- `/sub_goals` 같은 자유형 함수호출 대신, 검증된 스킬 ID(`SKILL_PICK_APPLE`, `SKILL_PLACE_BASKET`) 열거형으로 제한.

---

## 6. 폐루프 예외처리 — 3단 방어선

```mermaid
flowchart TD
    subgraph L1DEF["1단: 자동복구 (VLM 무관)"]
        PC{"Progress Checker<br/>정체 감지"} --> REC["behavior_server<br/>Spin/BackUp/ClearCostmap"]
        GV["mission_manager 파지검증<br/>tracking 재조회"] --> RETRY["정의된 재시도<br/>(횟수·각도 테이블)"]
    end
    subgraph L2DEF["2단: 안전정지"]
        CMON["collision_monitor"] --> STOP(["감속/정지 + e-stop 연동"])
    end
    subgraph L3DEF["3단: 자문 (정지상태에서만)"]
        FAIL["1단 소진"] --> VLMQ["vlm_task_advisor 자문요청<br/>+ HMI 알람"]
        VLMQ --> APPROVAL{"작업자/규칙 승인?"}
        APPROVAL -->|"승인"| RESUME(["BT 재개"])
        APPROVAL -->|"거부"| PARK(["안전위치 대기"])
    end
```

---

## 7. RL은 어디에? — 실행경로 밖 3곳만

| 위치 | 용도 | 실행경로 영향 |
|---|---|---|
| 오프라인 파라미터 튜닝 | DWB/MPPI 가중치, smoother 프로파일 탐색 | 없음 (배포는 yaml로) |
| Shadow mode 로깅 | `/cmd_vel`·`JointTrajectory` 옆에서 추론만, 발행 안 함 | 없음 (기록·평가만) |
| 시뮬레이터(Issac Sim/Gazebo) 사전검증 | 자문안·신규 스킬 검증 | 없음 |

> 현장 갭이 크다면 RL 잔차노드 추가가 아니라 **캘리브레이션(엔코더·질량·마찰 식별) + controller 게인 스케줄링**을 먼저 한다. 그게 현업의 1순위 처방.

---

## 8. 노드 요약 & 제공 여부

| 노드 | 제공 여부 |
|---|---|
| `realsense2_camera_node` ×3 | ✅ 제공 (Intel 공식) |
| `slam_toolbox`, `amcl`, `ekf_node`, `robot_state_publisher` | ✅ 제공 |
| `bt_navigator`, `planner_server`, `controller_server`, `velocity_smoother`, `collision_monitor`, `behavior_server` | ✅ 제공 + `nav2_params.yaml` 필요 |
| `move_group`, `servo_node`, `controller_manager`, `diff_drive/joint_trajectory/gripper/admittance_controller` | ✅ 제공 + SRDF·`ros2_controllers.yaml` 필요 |
| `multicam_3d_segmentation_node`, `object_tracking_node` | 🚩 직접 구현 (인지 격리 원칙 준수) |
| `mission_manager_node` | 🚩 직접 구현 (BT 기반, Nav2 `bt_navigator` + `BehaviorTree.CPP` 권장) |
| `vlm_task_advisor_node` | 🚩 직접 구현 (비실시간, 승인게이트 필수) |
| RL 노드 | ❌ 실행경로에 두지 않음 (7장 위치만 허용) |
| `mcu_bridge_node` | ❌ 삭제, `hardware_interface`로 대체 |

### 주요 인터페이스

| 구간 | 타입 | 이름 |
|---|---|---|
| 작업지시 | 🛎️ Service/OPC-UA | `SubmitTask` (스킬ID 열거형) |
| 이동 | 🎬 Action | `NavigateToPose`, `FollowPath`, `ComputePathToPose` |
| 조작 | 🎬 Action | `FollowJointTrajectory` (`control_msgs`) |
| 속도 최종단 | 📨 Topic | `/cmd_vel` → `/cmd_vel_smoothed` → `/cmd_vel_final` (`geometry_msgs/Twist`) |
| 인지조회 | 📨 Topic | `/tracked_objects` (커스텀), `/segmented_objects_3d` (`vision_msgs/Detection3DArray`) |
| 안전 | 🔌 HW + 📨 | e-stop 하드와이어 + `/scan` → `collision_monitor` |

---

## 9. 자매 문서와의 관계

- [ros2-dualarm-multicam-vlm-rl-architecture.md](./ros2-dualarm-multicam-vlm-rl-architecture.md): 본 문서의 연구형 원본. RL 잔차보정 + `mcu_bridge` + VLM 실시간 루프 버전.
- [ros2-dualarm-multicam-hierarchical-rl-architecture.md](./ros2-dualarm-multicam-hierarchical-rl-architecture.md): 최상위를 HRL 메타정책으로 교체한 또 다른 연구형. 현업 전환 시 동일하게 실행경로에서 제거 대상.
- [ros2-rule-based-architecture.md](./ros2-rule-based-architecture.md): Nav2 표준 체인의 개념 원형. 본 문서는 그것을 듀얼암 모바일 매니퓰레이터에 확장한 것.
- [ros2-turtlebot3-manipulator-example.md](./ros2-turtlebot3-manipulator-example.md): 단일암 예제. Eye-in-Hand/Eye-to-Hand TF 원리 동일.

## 10. 현업 이식 체크리스트 (이 순서대로)

1. `mcu_bridge` → `ros2_control hardware_interface` + EtherCAT/CAN 교체, e-stop 하드와이어.
2. Safety LiDAR 추가 + `collision_monitor`를 최후방어선으로 배선.
3. SLAM을 `rtabmap`(3D 밀집) → `slam_toolbox`(2D LiDAR)로 전환, 카메라는 costmap 보조로 격리.
4. RL 노드를 실행경로에서 제거 → shadow 모드로 강등.
5. VLM을 실시간 루프에서 분리 → 승인게이트 + 스킬ID 열거형.
6. `coordinator`를 `mission_manager(BT)`로 재작성: 정상계·1차복구는 VLM 없이 완결.
7. 가반하중별 `velocity_smoother`·`controller` 파라미터 세트 분리 + F/T 기반 `admittance` 추가.
