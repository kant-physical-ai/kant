# 듀얼암 + 멀티카메라 + VLM/SLAM/Nav2+RL 통합 로봇 조작 시스템

> 🧭 이 문서는 별도로 제공된 **"다중 카메라 및 Embodied AI 기반 로봇 조작 시스템 설계 보고서"**를 다이어그램으로 재구성한 것입니다. [TurtleBot3 단일암 5종 문서](./ros2-turtlebot3-manipulator-example.md)와는 **다른 로봇**(4륜 베이스 + 듀얼 매니퓰레이터 + RealSense 3대 + SLAM)이지만, 도형/색상/화살표 이모지 규칙은 동일하게 적용했습니다: 💊 알약형=노드, ⬡ 육각형=하드웨어, 🚩 깃발=커스텀, 🛢️ 원기둥=데이터/지도, 📨 토픽·🎬 액션·🛎️ 서비스·🔌 시리얼.
>
> ⚠️ 원본 보고서는 개념 설계 문서라 일부 노드는 구체적 ROS 2 패키지명이 명시되지 않았습니다. 실존 패키지로 특정 가능한 것(RealSense, RTAB-Map, Nav2, MoveIt2)은 실제 패키지명을 달았고, 나머지(YOLO 병합, 투표, 추적, VLM, RL, Coordinator)는 🔧 커스텀으로 표시했습니다.

---

## 0. 전체 구조 한눈에 보기

```mermaid
flowchart TB
    Human((("👤 사람<br/><i>예: '오른쪽 끝 사과를 옆 바구니에'</i>")))

    subgraph PERC["👁️ 인지 레이어 (멀티카메라 3D 인지·추적)"]
        Fusion>"🔧 <i>(커스텀)::</i><b>multicam_3d_segmentation_node</b><br/><i>3개 카메라 RGB-D 직접 구독 →<br/>내부 배치(batch=3) YOLO 추론 →<br/>TF 통합 → Voting 병합<br/>(모델 1벌만 로드, GPU 효율↑)</i>"]
        Track>"🔧 <i>(커스텀)::</i><b>object_tracking_node</b><br/><i>칼만필터, 가림에도<br/>Unique ID 유지(예: Apple_777)</i>"]
    end

    subgraph SLAML["🗺️ SLAM & 지도 레이어"]
        SLAM("<i>rtabmap_ros::</i><b>rtabmap</b><br/><i>동적 TF 반영 3D SLAM</i>")
    end

    subgraph DATA["🧮 데이터 보정 레이어"]
        RSP("<i>robot_state_publisher::</i><b>robot_state_publisher</b><br/><i>URDF+joint_states→전체 TF 계산<br/>(동적 팔 카메라 TF 포함)</i>")
    end

    subgraph VLML["🧠 고차원 계획 레이어"]
        VLM>"🔧 <i>(커스텀)::</i><b>vlm_task_planner_node</b><br/><i>Top-down 영상+객체ID/좌표<br/>→ sub-goal 함수호출</i>"]
        Coord>"🔧 <i>(커스텀)::</i><b>coordinator_node</b><br/><i>sub-goal 실행 순서 관리 +<br/>실패 시 VLM 재계획 요청</i>"]
    end

    subgraph NAVL["🧭 Nav2/MoveIt 전역 계획 레이어"]
        BT("<i>nav2_bt_navigator::</i><b>bt_navigator</b>")
        Planner("<i>nav2_planner::</i><b>planner_server</b>")
        Controller("<i>nav2_controller::</i><b>controller_server</b>")
        Behavior("<i>nav2_behaviors::</i><b>behavior_server</b><br/><i>1차 자체복구(Recovery)</i>")
        MoveGroup("<i>moveit_ros_move_group::</i><b>move_group</b><br/><i>듀얼암 IK/궤적계획</i>")
    end

    subgraph RLL["🟧 Sim-to-Real RL 보정 레이어"]
        RLNav>"🔧 <i>(커스텀)::</i><b>rl_nav_correction_node</b><br/><i>Nav2 명령+센서→<br/>현장 갭 보정한 cmd_vel</i>"]
        RLArm>"🔧 <i>(커스텀)::</i><b>rl_arm_correction_node</b><br/><i>MoveIt 궤적+센서→<br/>현장 갭 보정한 관절 토크</i>"]
    end

    subgraph HWL["⚙️ 하드웨어 레이어"]
        MCU>"🔧 <i>(커스텀)::</i><b>mcu_bridge_node</b><br/><i>시리얼 write: 최종 명령 전달<br/>시리얼 read: 엔코더→odom/joint_states<br/>(양방향 브리지)</i>"]
        Cams{{"RealSense ×3<br/>(Left/Right Arm Cam: 동적 TF,<br/>Top-down Cam: 고정 TF)"}}
        Base{{"4륜 모바일 베이스"}}
        Arms{{"듀얼 매니퓰레이터 ×2"}}
    end

    Human -->|"🗣️ 자연어 명령"| VLM
    Cams -->|"📨 /{left,right,top}_cam/color/image_raw<br/>+ aligned_depth_to_color + camera_info"| Fusion
    RSP -->|"📨 /tf (동적 팔 카메라 TF 포함)"| Fusion
    Fusion -->|"📨 /segmented_objects_3d<br/>(base_link 기준)"| Track
    Track -->|"📨 /tracked_objects<br/>(커스텀, ID+pose+volume)"| VLM
    Track -->|"📨 /tracked_objects"| Coord
    Fusion -->|"📨 /segmented_objects_3d"| SLAM
    RSP -->|"📨 /tf"| SLAM
    MCU -->|"📨 /joint_states (양팔 엔코더)"| SLAM
    SLAM -->|"📨 /map (Occupancy Grid)"| Planner

    VLM -->|"📨 /sub_goals<br/>(함수호출: navigate_to(id), pick(id)...)"| Coord
    Coord -->|"🎬 NavigateToPose<br/>(Track에서 조회한 3D좌표)"| BT
    Coord -->|"🎬 MoveGroup (pick/place)"| MoveGroup
    BT -->|"🎬 ComputePathToPose"| Planner
    BT -->|"🎬 FollowPath"| Controller
    BT -->|"🎬 Spin/BackUp/ClearCostmap"| Behavior

    Controller -->|"📨 /cmd_vel_nav2"| RLNav
    MoveGroup -->|"📨 JointTrajectory"| RLArm
    Cams -->|"📨 /top_cam/color/image_raw 등<br/>(sensor_msgs/Image)"| RLNav
    Cams -->|"📨 /left_cam,/right_cam/color/image_raw<br/>(sensor_msgs/Image)"| RLArm
    MCU -->|"📨 /odom (엔코더 기반)"| RLNav
    MCU -->|"📨 /joint_states (양팔 엔코더)"| RLArm
    MCU -->|"📨 /joint_states"| RSP
    RLNav -->|"📨 /cmd_vel_final"| MCU
    RLArm -->|"📨 joint_cmd_final"| MCU
    MCU -->|"🔌 시리얼 write"| Base
    Base -->|"🔌 시리얼 read: 휠 엔코더"| MCU
    MCU -->|"🔌 시리얼 write"| Arms
    Arms -->|"🔌 시리얼 read: 관절 엔코더"| MCU

    Behavior -.->|"🎬 이동 복구 실패 시 FAIL"| Coord
    MoveGroup -.->|"🎬 pick/place 결과<br/>(모션 완료 여부만)"| Coord
    Coord -->|"📨 파지 검증 재조회<br/>(물체가 그리퍼 따라 움직였나)"| Track
    Coord -->|"📨 실패 리포트(이동 or 파지) +<br/>최신 Top-down 이미지"| VLM

    classDef custom fill:#ffcdd2,stroke:#c62828,color:#1b1b1b;
    classDef provided fill:#c8e6c9,stroke:#2e7d32,color:#1b1b1b;
    classDef rl fill:#ffe0b2,stroke:#e65100,color:#1b1b1b;
    classDef phys fill:#fff9c4,stroke:#f9a825,color:#1b1b1b;
    classDef data fill:#e1bee7,stroke:#6a1b9a,color:#1b1b1b;
    class Fusion,Track,VLM,Coord,MCU custom;
    class SLAM,BT,Planner,Controller,Behavior,MoveGroup provided;
    class RLNav,RLArm rl;
    class Cams,Base,Arms phys;
    class RSP data;
```

---

## 1. 하드웨어 & TF 구성

| 부품 | 세부 | TF 특징 |
|---|---|---|
| 모바일 베이스 | 4륜 개조 TurtleBot Base | `odom → base_footprint → base_link` |
| 매니퓰레이터 ×2 | Robotis Manipulator (좌/우) | `base_link → left_arm_link1..N`, `base_link → right_arm_link1..N` |
| Left/Right Arm Camera | RealSense, 그리퍼 위쪽 장착 | **동적 TF** — `left_arm_tool0 → left_cam_link` (관절 움직이면 카메라도 같이 움직임, Eye-in-Hand) |
| Top-down Camera | RealSense, 로봇 상부 고정 | **고정 TF** — `base_link → topdown_cam_link` (Eye-to-Hand) |

```mermaid
flowchart LR
    odom((odom)) --> bl((base_link))
    bl --> la1([left_arm_link1]) --> la2([...]) --> laee([left_ee]) --> lcam(["left_cam_link<br/>(동적, Eye-in-Hand)"])
    bl --> ra1([right_arm_link1]) --> ra2([...]) --> raee([right_ee]) --> rcam(["right_cam_link<br/>(동적, Eye-in-Hand)"])
    bl --> tcam(["topdown_cam_link<br/>(고정, Eye-to-Hand)"])
```

> 좌/우 암 카메라는 팔이 움직일 때마다 `robot_state_publisher`가 관절각 기반으로 TF를 계속 갱신합니다 — [TurtleBot3 문서에서 다룬 Eye-in-Hand/Eye-to-Hand 원리](./ros2-turtlebot3-manipulator-example.md#layer-5--매니퓰레이터-계획-노드-moveit2)와 동일하되, 이번엔 **한 로봇 안에 두 방식이 동시에** 존재하는 경우입니다.

---

## 2. 인지 파이프라인 (Perception & Tracking) — 실제 토픽/메시지 타입

> `realsense2_camera` 노드는 `camera_name`/`camera_namespace` 파라미터로 카메라마다 네임스페이스를 분리해서 띄웁니다(`left_cam`, `right_cam`, `top_cam`). 아래는 그 실제 기본 토픽명입니다.

| 노드 | 구독(Sub) | 발행(Pub) | 메시지 타입 |
|---|---|---|---|
| `realsense2_camera::realsense2_camera_node` ×3 | — | `/{ns}/color/image_raw`<br/>`/{ns}/color/camera_info`<br/>`/{ns}/aligned_depth_to_color/image_raw` | `sensor_msgs/Image`<br/>`sensor_msgs/CameraInfo`<br/>`sensor_msgs/Image` |
| `(커스텀)::multicam_3d_segmentation_node` | 3개 카메라의 위 3종 토픽 (총 9개 구독) + `/tf` | `/segmented_objects_3d` | 노드 내부에서: ① 3장의 RGB를 **배치(batch=3)로 묶어 YOLO 추론 한 번 호출** → ② 각 카메라 depth+camera_info로 마스크 영역을 3D 역투영 → ③ tf2로 `base_link` 기준 통합 → ④ 겹침 Voting 병합 → `vision_msgs/Detection3DArray` (frame_id=base_link)로 발행 |
| `(커스텀)::object_tracking_node` | `/segmented_objects_3d` | `/tracked_objects` | 표준 메시지에 "여러 프레임에 걸친 지속 ID"라는 개념이 없어서 **커스텀 메시지**(`TrackedObjectArray`: `id`(string), `pose`, `velocity`, `class`, `volume`) 필요 |

```mermaid
flowchart TB
    LCam{{"realsense2_camera::<br/>left_cam"}} -->|"📨 /left_cam/color/image_raw<br/>+ /aligned_depth_to_color/image_raw<br/>+ /color/camera_info"| Fusion
    RCam{{"realsense2_camera::<br/>right_cam"}} -->|"📨 /right_cam/... (동일 3종)"| Fusion
    TCam{{"realsense2_camera::<br/>top_cam"}} -->|"📨 /top_cam/... (동일 3종)"| Fusion["🔧 <i>(커스텀)::</i><b>multicam_3d_segmentation_node</b><br/><i>① 배치 YOLO(batch=3) → ② 3D 역투영<br/>→ ③ TF 통합 → ④ Voting 병합</i>"]
    TFin["/tf (동적, 관절각 반영)"] -->|"📨"| Fusion

    Fusion -->|"📨 /segmented_objects_3d<br/>(vision_msgs/Detection3DArray,<br/>frame_id=base_link)"| Track>"🔧 <i>(커스텀)::</i><b>object_tracking_node</b>"]
    Track -->|"📨 /tracked_objects<br/>(커스텀 msg: id+pose+velocity+class+volume)"| Out(["VLM · SLAM · Coordinator로 전달"])

    classDef custom fill:#ffcdd2,stroke:#c62828,color:#1b1b1b;
    classDef phys fill:#fff9c4,stroke:#f9a825,color:#1b1b1b;
    class Fusion,Track custom;
    class LCam,RCam,TCam phys;
```

> **왜 한 노드로 합쳤나?** 카메라 3대를 노드 3개로 나누면 GPU에 YOLO 모델이 3벌 올라가고 추론도 3번 따로 돕니다. `multicam_3d_segmentation_node` 하나가 3장을 배치로 묶어 처리하면 **모델은 1벌만 로드, 추론은 배치 1회**로 끝나서 Jetson급 단일 GPU에서 훨씬 효율적입니다. 대신 이 노드 하나가 죽으면 카메라 3대 인지가 동시에 멈추는 트레이드오프는 있습니다.
>
> **depth까지 같은 노드에서 같이 처리하는 이유**: 2D 마스크만 만들고 depth 역투영을 다른 노드에 미루면, 원본 depth 이미지 전체를 한 번 더 네트워크로 복사해야 합니다. 같은 노드 안에서 바로 `camera_info` 내부파라미터로 마스크 영역만 3D로 역투영하면, 그 다음부터는 압축된 3D 박스 몇 개(`Detection3DArray`)만 주고받으면 되어 대역폭이 훨씬 적게 듭니다.

---

## 3. SLAM & Nav2 지도 연동

```mermaid
flowchart LR
    PC["3D pointcloud<br/>(multicam_3d_segmentation_node)"] -->|"📨"| SLAM("<i>rtabmap_ros::</i><b>rtabmap</b>")
    JS["/joint_states"] -->|"📨"| SLAM
    TFin["/tf"] -->|"📨"| SLAM
    SLAM -->|"📨 /map (Occupancy Grid)"| GC("<i>nav2_costmap_2d::</i><b>global_costmap</b>")
    SLAM -->|"📨 /tf: map→odom"| BT("<i>nav2_bt_navigator::</i><b>bt_navigator</b>")

    classDef provided fill:#c8e6c9,stroke:#2e7d32,color:#1b1b1b;
    class SLAM,GC,BT provided;
```

> 이 시스템은 팔에 달린 카메라가 **계속 움직이면서** 지도를 그리기 때문에, `rtabmap`이 `/joint_states`+`/tf`로 "지금 이 포인트클라우드가 로봇 기준 어디서 찍혔는지"를 매번 정확히 알아야 합니다. 고정 카메라만 쓰는 일반 SLAM보다 계산이 더 복잡한 이유입니다.

---

## 4. VLM ↔ Coordinator 역할 분담 + 돌발상황 판단 흐름

**둘의 관계를 한 문장으로**: `vlm_task_planner_node`는 **"무엇을 할지" 판단을 100% 전담**하고, `coordinator_node`는 **판단을 전혀 하지 않고** VLM이 정해준 순서대로 하위 액션을 호출·감시하기만 합니다. 그래서 "돌발상황이 생기면 누가 판단하나?"의 답은 **항상 VLM 하나뿐**입니다 — Coordinator는 "성공했는지 실패했는지"만 감지해서 실패 시 무조건 VLM에게 그대로 떠넘깁니다.

| | 하는 일 | 안 하는 일 |
|---|---|---|
| `vlm_task_planner_node` | 최초 계획 수립, **모든 재계획(Re-planning) 판단** | 실시간 액션 호출/감시 (안 함) |
| `coordinator_node` | sub-goal 순서대로 액션 호출, 성공/실패 감지, 실패 시 VLM에 보고 | **왜 실패했는지 스스로 판단하거나 대안을 짜는 것 (절대 안 함)** |

```mermaid
sequenceDiagram
    participant Human as 👤 사람
    participant VLM as (커스텀)::vlm_task_planner_node
    participant Track as (커스텀)::object_tracking_node
    participant Coord as (커스텀)::coordinator_node
    participant BT as nav2_bt_navigator::bt_navigator
    participant MoveGroup as moveit_ros_move_group::move_group

    Human->>VLM: 🗣️ "오른쪽 끝 사과를 옆 바구니에"
    Track-->>VLM: 📨 /tracked_objects (Apple_777, Basket_12)
    VLM-->>Coord: 📨 /sub_goals: [navigate_to(Apple_777), pick(Apple_777),<br/>navigate_to(Basket_12), place(Basket_12)]

    loop sub-goal마다 (Coord는 순서만 따라감, 스스로 판단 없음)
        alt 이동 sub-goal
            Coord->>Track: 최신 3D좌표 조회
            Coord->>BT: 🎬 NavigateToPose(좌표)
            Note over BT: 내부에서 Progress Checker가<br/>제자리걸음 감지 시 1차 자체복구(Spin/BackUp) 시도<br/>(6장 참고)
            BT-->>Coord: 🎬 결과: 성공 or 실패(자체복구도 실패)
        else 집기/놓기 sub-goal
            Coord->>MoveGroup: 🎬 MoveGroup(pick/place)
            MoveGroup-->>Coord: 🎬 결과: 모션 완료
            Coord->>Track: 📨 파지 검증: Apple_777이 그리퍼 위치를<br/>따라 움직였는지 재조회
            Note over Coord: 모션은 성공해도 물체가 그대로면<br/>= 파지 실패로 간주 (Coord가 "감지"만 함, "왜"는 모름)
        end

        alt 성공
            Coord->>Coord: 다음 sub-goal로 진행
        else 실패 (액션 실패 또는 파지검증 실패)
            Coord->>VLM: 📨 실패 리포트(실패한 sub-goal+사유) + 최신 Top-down 이미지
            VLM->>VLM: 🧠 재계획 (유일한 판단 지점)
            VLM-->>Coord: 📨 새 /sub_goals (예: 우회 경로, 재파지 각도 변경)
        end
    end
```

> 원본 보고서의 핵심 아이디어: VLM은 "Apple_777을 어디로 가져갈지"만 말하고, **실제 3D 좌표는 매번 `object_tracking_node`에서 최신 값으로 다시 조회**합니다. 물체가 조금 움직였어도(예: 다른 로봇이나 사람이 건드림) 마지막으로 추적된 위치로 정확히 찾아갑니다 — [하이브리드 문서](./ros2-turtlebot3-manipulator-hybrid.md)의 "정적 semantic_waypoints.yaml 조회"보다 한 단계 더 동적인 버전입니다.
>
> **파지 실패는 Nav2 Progress Checker로 못 잡습니다.** MoveIt의 액션 결과는 "계획한 모션을 완료했다"는 뜻이지 "실제로 물건을 집었다"는 뜻이 아닙니다. 그래서 `coordinator_node`가 pick/place 직후 **`object_tracking_node`에 다시 물어봐서 물체가 그리퍼를 따라 움직였는지 확인**하는 별도 검증 단계가 필요합니다 — 이게 이 시스템에서 "파지 실패"를 감지하는 유일한 방법입니다.

---

## 5. Nav2(전역) + RL(보정) 계층적 제어

이 시스템의 RL은 [TurtleBot3 하이브리드 문서](./ros2-turtlebot3-manipulator-hybrid.md)처럼 **컨트롤러 플러그인을 교체**하는 방식이 아니라, Nav2/MoveIt이 낸 명령을 **한 번 더 사후 보정(residual correction)**하는 방식입니다.

```mermaid
flowchart LR
    Controller("<i>nav2_controller::</i><b>controller_server</b><br/><i>기존 DWB 그대로</i>") -->|"📨 /cmd_vel_nav2"| RLNav>"🔧 <i>(커스텀)::</i><b>rl_nav_correction_node</b>"]
    Cam["카메라 raw"] -->|"🔌"| RLNav
    Odom["/odom"] -->|"📨"| RLNav
    RLNav -->|"📨 /cmd_vel_final<br/>(현장 갭 보정됨)"| MCU("🔧 <i>(커스텀)::</i><b>mcu_bridge_node</b>")
    MCU -->|"🔌 시리얼"| Base{{"모바일 베이스"}}

    classDef custom fill:#ffcdd2,stroke:#c62828,color:#1b1b1b;
    classDef provided fill:#c8e6c9,stroke:#2e7d32,color:#1b1b1b;
    classDef rl fill:#ffe0b2,stroke:#e65100,color:#1b1b1b;
    classDef phys fill:#fff9c4,stroke:#f9a825,color:#1b1b1b;
    class Controller provided;
    class RLNav,MCU custom;
    class Base phys;
```

| 구분 | [하이브리드 문서](./ros2-turtlebot3-manipulator-hybrid.md) 방식 | 이 시스템 방식 |
|---|---|---|
| RL 위치 | `controller_server`의 **플러그인**으로 교체 (Nav2 안에 흡수) | `controller_server` **뒤에** 별도 노드로 연결 (사후 보정) |
| Nav2 컨트롤러 | 없음 (DWB 자리를 RL이 완전히 대체) | DWB **그대로 유지**, RL은 그 출력을 미세 보정만 |
| 장점 | 구조가 깔끔 (Nav2 액션 한 번으로 끝) | Nav2 컨트롤러 로직을 안 건드려서 디버깅 쉬움 |
| 이 방식의 이름 | 대체형(Replacement) | **잔차 보정형(Residual Correction)** — Sim-to-Real 논문에서 흔한 패턴 |

---

## 6. 폐루프 예외 처리 (Closed-loop Recovery)

> 4장에서 본 큰 흐름 중 "실패 감지" 부분만 확대한 것입니다. **원인이 다른 두 갈래**(이동 실패 vs 파지 실패)가 있는데, 원본 보고서의 "Progress Checker" 언급은 **이동 쪽에만** 해당합니다 — 파지 실패는 애초에 Nav2 소관이 아니라서 별도 경로가 필요합니다.

```mermaid
flowchart TD
    subgraph NAVFAIL["🧭 이동 실패 경로"]
        Stuck1["🚨 제자리걸음 / 회피불가"] --> PC{"nav2_behaviors::<br/>Progress Checker<br/>(Timeout 감지)"}
        PC -->|"타임아웃"| Recovery["nav2_behaviors::behavior_server<br/>1차 자체복구<br/>(Spin/BackUp/ClearCostmap)"]
        Recovery -->|"복구 실패"| Fail1["🎬 NavigateToPose<br/>액션 result: FAIL"]
    end

    subgraph MANIPFAIL["🦾 파지 실패 경로 (Progress Checker 无관)"]
        Stuck2["🚨 pick/place 모션은 완료됐지만<br/>물체가 그대로 있음"] --> Verify["🔧 coordinator_node가<br/>object_tracking_node 재조회로 자체 검증<br/>(MoveIt엔 이 검증 기능이 없음)"]
        Verify -->|"물체 위치 안 변함"| Fail2["파지 실패로 판정"]
    end

    Recovery -->|"복구 성공"| Resume(["임무 재개, 다음 sub-goal로"])
    Fail1 --> Coord["🔧 coordinator_node<br/>(판단 없이) 실패 사실 + 최신 Top-down 이미지만 수집"]
    Fail2 --> Coord
    Coord --> Replan["🔧 vlm_task_planner_node<br/>재계획(Re-planning) — 유일한 판단 지점"]
    Replan -->|"새 sub-goal<br/>예: 좌측 우회 경로 / 재파지 각도 변경"| Resume

    classDef custom fill:#ffcdd2,stroke:#c62828,color:#1b1b1b;
    classDef provided fill:#c8e6c9,stroke:#2e7d32,color:#1b1b1b;
    class Coord,Replan,Verify custom;
    class PC,Recovery provided;
```

**단계별 실제 구현 지점**:
1. **이동 실패 — Progress Checker**: `nav2_controller`/`nav2_behaviors`가 이미 제공하는 실존 기능 (`nav2_params.yaml`에서 임계값 설정) — ⚙️ 제공
2. **이동 실패 — 1차 자체복구**: `behavior_server`의 `Spin`/`BackUp`/`ClearCostmap` 액션 — ⚙️ 제공 (룰베이스 문서와 동일)
3. **파지 실패 — 자체 검증**: `coordinator_node`가 pick/place 액션 성공 직후 `object_tracking_node`에 재조회해서 "물체가 실제로 옮겨졌는지" 확인 — 🔧 **직접 구현 필요** (MoveIt/Nav2 어디에도 없는, 이 시스템만의 로직)
4. **공통 — 실패 전파**: 두 경로 모두 결국 `coordinator_node`로 모여서 동일하게 처리됨 — ⚙️(이동) / 🔧(파지) 혼합
5. **공통 — VLM 재계획**: `coordinator_node`가 실패 리포트+이미지를 VLM에 보내는 것과 VLM의 재계획 로직 — 🔧 **직접 구현 필요**

---

## 7. 노드 요약 & 제공 여부

| 노드 | 제공 여부 |
|---|---|
| `realsense2_camera_node` ×3 | ✅ 완전 제공 (Intel 공식 패키지) |
| `rtabmap` | ✅ 완전 제공 (`rtabmap_ros`) |
| `bt_navigator`, `planner_server`, `controller_server`, `behavior_server`, `global/local_costmap` | ⚙️ 제공 + 파라미터 필요 (Nav2, 룰베이스 문서와 동일) |
| `move_group` | ⚙️ 제공 + 듀얼암 SRDF 설정 필요 (MoveIt2) |
| `multicam_3d_segmentation_node`, `object_tracking_node` | 🔧 **직접 구현 필요** (인지 파이프라인 전체) |
| `vlm_task_planner_node`, `coordinator_node` | 🔧 **직접 구현 필요** |
| `rl_nav_correction_node`, `rl_arm_correction_node` | 🔧 **직접 구현 필요** (+ Sim-to-Real 학습 파이프라인) |
| `mcu_bridge_node` | 🔧 **직접 구현 필요** (또는 `ros2_control` 하드웨어 인터페이스로 대체 가능) |

---

## 8. 참고: 다른 문서와의 관계

- **RL을 "상위 개념"(Hierarchical RL)으로 쓰는 대안 버전**: [ros2-dualarm-multicam-hierarchical-rl-architecture.md](./ros2-dualarm-multicam-hierarchical-rl-architecture.md) — 이 문서의 `vlm_task_planner_node` 자리를 학습된 메타 정책(`hrl_meta_policy_node`)으로 교체한 자매 문서. 하드웨어/인지/SLAM/Nav2+RL 보정 레이어는 동일하고, 최상위 계획 방식과 돌발상황 재계획 로직만 다릅니다.
- 구조적으로 [ros2-turtlebot3-manipulator-hybrid.md](./ros2-turtlebot3-manipulator-hybrid.md)의 "Nav2 전역 + RL 지역" 아이디어를 **잔차 보정형**으로 변형하고, 여기에 **멀티카메라 인지(YOLO+Voting+Tracking)**와 **VLM 기반 물체ID 타겟팅**, **SLAM 폐루프 예외처리**를 추가로 결합한 상위 호환 시스템입니다.
- 단일암 TurtleBot3 5종 문서: [룰베이스](./ros2-turtlebot3-manipulator-example.md) · [E2E](./ros2-turtlebot3-manipulator-e2e.md) · [RL(Mapless)](./ros2-turtlebot3-manipulator-simtoreal-rl.md) · [하이브리드](./ros2-turtlebot3-manipulator-hybrid.md) · [VLA](./ros2-turtlebot3-manipulator-vla.md)
- 개념적 원형: [ros2-architecture.md](./ros2-architecture.md)
