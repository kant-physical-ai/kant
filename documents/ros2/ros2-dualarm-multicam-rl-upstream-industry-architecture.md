# 듀얼암 + 멀티카메라 — RL 업스트림(역전) 현업 아키텍처

> 연구형(`move_group → rl_arm_correction → MCU`, `controller → rl_nav_correction → MCU`)을 뒤집은 현업 버전입니다.
> 핵심 한 줄: **RL은 Nav2/MoveIt 뒤에서 덮어쓰지 않고, 앞에서 점수·게인 자문만 하고 최종 명령은 Nav2/MoveIt → ros2_control 직행.**
>
> 관련 문서: [VLM-RL 연구형](./ros2-dualarm-multicam-vlm-rl-architecture.md) · [HRL 연구형](./ros2-dualarm-multicam-hierarchical-rl-architecture.md) · [순수 현업](./ros2-dualarm-multicam-industry-standard-architecture.md) · [VLM·RL 포함 현업](./ros2-dualarm-multicam-vlm-rl-industry-architecture.md)
>
> 규칙 동일: 💊 알약=노드, ⬡ 육각=하드웨어, 🚩 깃발=커스텀, 🛢️ 원기둥=데이터/지도, 📨 토픽·🎬 액션·🛎️ 서비스·🔌 필드버스(EtherCAT/CAN).

---

## 0. 연구형 vs 역전(현업) 비교

| 구간 | 연구형 | 역전(본 문서) |
|---|---|---|
| 팔 | `move_group → rl_arm_correction → MCU` (RL이 최종 토크 덮어씀) | `rl_grasp_scorer → move_group → ros2_control → HW` (RL은 점수만, 최종은 MoveIt) |
| 이동 | `controller → rl_nav_correction → MCU` (RL이 `/cmd_vel` 가로챔) | `rl_nav_tuner(게인만) → controller → smoother → mux → collision_monitor → ros2_control` |
| 안전선 | 없음 (RL이 최후 실행자) | `collision_monitor` + `cmd_vel_mux` + e-stop이 최후, RL 우회 불가 |

---

## 1. 전체 다이어그램 (역전)

```mermaid
flowchart TB
    Human((("👤 작업자/관제<br/>HMI·MES")))

    subgraph PERC["👁️ 인지"]
        Fusion>"🚩 <i>(커스텀)::</i><b>multicam_3d_segmentation_node</b>"]
        Track>"🚩 <i>(커스텀)::</i><b>object_tracking_node</b>"]
    end

    subgraph FRONT_RL["🟣 앞단 RL (자문만, 실행권 없음)"]
        GScore>"🚩 <i>(커스텀)::</i><b>rl_grasp_scorer_node</b><br/><i>파지 후보 점수만 출력<br/>토크·궤적 출력금지</i>"]
        RLNav>"🚩 <i>(커스텀)::</i><b>rl_nav_tuner_node</b><br/><i>DWB/MPPI 게인·cost 스케일만<br/>Twist 출력금지, 0.5~1Hz</i>"]
        HRL>"🚩 <i>(커스텀)::</i><b>hrl_meta_policy_node (선택)</b><br/><i>다음 스킬 1개 제안만<br/>직접 액션호출 금지</i>"]
    end

    subgraph TASK["🧠 임무 (유일 실행권자)"]
        MGR>"🚩 <i>(커스텀)::</i><b>mission_manager_node</b><br/><i>BT, 승인게이트 통과분만 실행</i>"]
        Gate{"승인게이트<br/>스킬ID 열거형·범위검사"}
    end

    subgraph NAVL["🧭 Nav2 표준"]
        BT("<i>nav2_bt_navigator::</i><b>bt_navigator</b>")
        Planner("<i>nav2_planner::</i><b>planner_server</b>")
        Controller("<i>nav2_controller::</i><b>controller_server</b><br/><i>DWB/MPPI</i>")
        Smoother("<i>nav2_velocity_smoother::</i><b>velocity_smoother</b>")
        CM("<i>nav2_collision_monitor::</i><b>collision_monitor</b>")
        MUX("<i>topic_tools::</i><b>cmd_vel_mux</b><br/><i>안전 우선</i>")
        Behavior("<i>nav2_behaviors::</i><b>behavior_server</b>")
    end

    subgraph MANIPL["🦾 MoveIt 표준"]
        MoveGroup("<i>moveit_ros_move_group::</i><b>move_group</b>")
        Servo("<i>moveit_servo::</i><b>servo_node</b>")
    end

    subgraph CTRL["⚙️ ros2_control"]
        R2C("<i>controller_manager::</i><b>controller_manager</b><br/><i>diff_drive + joint_trajectory + gripper</i>")
        HW("<i>hardware_interface::</i><b>mobile_manipulator_hw</b><br/><i>EtherCAT/CAN</i>")
    end

    subgraph HWL["⬡ 하드웨어"]
        LiDAR{{"2D Safety LiDAR"}}
        Cams{{"RealSense ×3"}}
        Base{{"4륜 베이스"}}
        Arms{{"듀얼암 + F/T + 그리퍼"}}
        Safety{{"e-stop·안전PLC"}}
    end

    Human -->|"🛎️ 작업지시"| MGR
    Cams -->|"📨 image_raw+depth+info"| Fusion
    Fusion -->|"📨 /segmented_objects_3d"| Track
    Track -->|"📨 /tracked_objects"| MGR
    Track -->|"📨"| GScore
    Track -->|"📨 state"| HRL
    GScore -->|"📨 /grasp_candidates_scored"| MGR
    HRL -->|"📨 후보 스킬 1개"| Gate
    Gate -->|"승인분만"| MGR

    MGR -->|"🎬 NavigateToPose"| BT
    MGR -->|"🎬 pick/place"| MoveGroup
    BT -->|"🎬 ComputePathToPose"| Planner
    BT -->|"🎬 FollowPath"| Controller
    BT -->|"🎬 Spin/BackUp"| Behavior

    RLNav -->|"📨 게인/스케일만"| Controller
    Controller -->|"📨 /cmd_vel"| Smoother
    Smoother -->|"📨 /cmd_vel_smoothed"| MUX
    CM -->|"📨 정지/감속 오버라이드"| MUX
    MUX -->|"📨 /cmd_vel_final"| R2C
    R2C -->|"🔌 wheel cmd"| HW
    HW -->|"🔌"| Base

    MoveGroup -->|"🎬 FollowJointTrajectory (직행)"| R2C
    Servo -->|"📨 제한내 joint_cmd"| R2C
    R2C -->|"🔌 joint cmd"| HW
    HW -->|"🔌"| Arms

    LiDAR -->|"📨 /scan"| CM
    LiDAR -->|"📨 /scan"| Planner
    HW -->|"📨 /joint_states, /wheel_odometry"| MGR
    Safety -->|"🔌 halt"| R2C

    classDef custom fill:#ffcdd2,stroke:#c62828,color:#1b1b1b;
    classDef provided fill:#c8e6c9,stroke:#2e7d32,color:#1b1b1b;
    classDef phys fill:#fff9c4,stroke:#f9a825,color:#1b1b1b;
    class Fusion,Track,GScore,RLNav,HRL,MGR custom;
    class BT,Planner,Controller,Smoother,CM,MUX,Behavior,MoveGroup,Servo,R2C,HW provided;
    class LiDAR,Cams,Base,Arms,Safety phys;
```

---

## 2. 예시 흐름 (사과 집기)

1. `Track` → `GScore`: `Apple_777` 후보 3개 전달.
2. `GScore` → `MGR`: `/grasp_candidates_scored` 점수 내림차순 (예: 0.91 / 0.72 / 0.45).
3. `MGR`가 0.91 선택 → `MoveGroup`에 `pick` 요청.
4. `MoveGroup` → `R2C`로 `FollowJointTrajectory` 직행. RL 경유 없음.
5. 이동은 `RLNav`가 `Controller` 게인만 미리 조정 → `Controller → Smoother → MUX → R2C` 직행.

---

## 3. 노드 요약

| 노드 | 위치 | 출력 |
|---|---|---|
| `rl_grasp_scorer_node` 🚩 | MoveIt 앞 | 점수만 |
| `rl_nav_tuner_node` 🚩 | Controller 앞 | 게인만 |
| `hrl_meta_policy_node` 🚩 (선택) | Coordinator 앞 | 다음 스킬 제안만 |
| `mission_manager_node` 🚩 + 승인게이트 | 실행권자 | 🎬 Action만 실행 |
| `rl_nav/arm_correction`, `mcu_bridge` | ❌ 삭제 | 연구형 잔재 |

## 4. 이식 순서

1. 뒷단 RL(`rl_nav/arm_correction`) 삭제 → `ros2_control` 직결.
2. 앞단 RL(스코어러·튜너)로 교체, 클램프+안전필터 추가.
3. `collision_monitor` + `cmd_vel_mux` + e-stop을 최후방어선으로 배선.
4. Shadow → 카나리 1대 → 전대 순서로 배포.
