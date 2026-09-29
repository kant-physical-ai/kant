# 듀얼암 + 멀티카메라 + VLM+HRL 하이브리드 통합 로봇 조작 시스템

> 🧭 이 문서는 [ros2-dualarm-multicam-vlm-rl-architecture.md](./ros2-dualarm-multicam-vlm-rl-architecture.md)의 자매 문서입니다. **하드웨어·인지 파이프라인·SLAM·Nav2+RL 잔차보정 레이어는 100% 동일**하고, 최상위가 **VLM(`vlm_task_planner_node`: 자연어→작업명세·단계별 실행계획) + HRL(`hrl_meta_policy_node`: 단계별 다음 스킬 선택)** 2층 하이브리드인 버전입니다. 도형/색상/이모지 규칙도 동일합니다.
>
> 한 줄: **VLM은 "무슨 일을 몇 단계로 할지" 초안+작업명세를 뽑고, HRL은 매 단계 "지금 다음 스킬 하나"를 고릅니다.** 사용자는 자연어로 툭 던지면 됩니다 (예: "오른쪽 끝 사과를 옆 바구니에").
>
> 이 문서만 읽어도 이해되도록 전체 구조를 다시 그렸지만, 1~3장·5장(하드웨어/인지/SLAM/Nav2+RL 보정)은 원본 문서 내용을 그대로 가져온 것이니 세부 토픽/메시지 타입은 [원본 문서](./ros2-dualarm-multicam-vlm-rl-architecture.md)를 참고하세요.

---

## 0. 전체 구조 한눈에 보기

```mermaid
flowchart TB
    Human((("👤 사람<br/><i>자연어로 툭 던짐<br/>예: '오른쪽 끝 사과를 옆 바구니에'</i>")))

    subgraph VLML["🧠 VLM 작업명세·실행계획 레이어 (자연어→단계별 계획 초안)"]
        VLM>"🔧 <i>(커스텀)::</i><b>vlm_task_planner_node</b><br/><i>자연어+물체목록+Top-down →<br/>task_spec(목표ID·완료조건·제약) +<br/>plan_template(단계별 초안 N단계)</i>"]
    end

    subgraph HRLL["🟣 Hierarchical RL 단계별 선택 레이어"]
        HRL>"🔧 <i>(커스텀)::</i><b>hrl_meta_policy_node</b><br/><i>task_spec+현재 상태(물체목록+로봇위치)를<br/>보고 학습된 정책이<br/>다음 sub-goal(스킬) 1개 선택</i>"]
        Coord>"🔧 <i>(커스텀)::</i><b>coordinator_node</b><br/><i>단계별 실행 + 성공/실패<br/>감지 후 HRL에 재질의<br/>N회 실패시 VLM 재계획 요청</i>"]
    end

    subgraph PERC["👁️ 인지 레이어 (원본과 동일)"]
        Fusion>"🔧 <i>(커스텀)::</i><b>multicam_3d_segmentation_node</b>"]
        Track>"🔧 <i>(커스텀)::</i><b>object_tracking_node</b>"]
    end

    subgraph SLAML["🗺️ SLAM & 지도 레이어 (원본과 동일)"]
        SLAM("<i>rtabmap_ros::</i><b>rtabmap</b>")
    end

    subgraph DATA["🧮 데이터 보정 레이어 (원본과 동일)"]
        RSP("<i>robot_state_publisher::</i><b>robot_state_publisher</b>")
    end

    subgraph NAVL["🧭 Nav2/MoveIt 전역 계획 레이어 (원본과 동일)"]
        BT("<i>nav2_bt_navigator::</i><b>bt_navigator</b>")
        Planner("<i>nav2_planner::</i><b>planner_server</b>")
        Controller("<i>nav2_controller::</i><b>controller_server</b>")
        Behavior("<i>nav2_behaviors::</i><b>behavior_server</b>")
        MoveGroup("<i>moveit_ros_move_group::</i><b>move_group</b>")
    end

    subgraph RLL["🟧 Sim-to-Real RL 보정 레이어 (원본과 동일)"]
        RLNav>"🔧 <i>(커스텀)::</i><b>rl_nav_correction_node</b>"]
        RLArm>"🔧 <i>(커스텀)::</i><b>rl_arm_correction_node</b>"]
    end

    subgraph HWL["⚙️ 하드웨어 레이어 (원본과 동일)"]
        MCU>"🔧 <i>(커스텀)::</i><b>mcu_bridge_node</b>"]
        Cams{{"RealSense ×3"}}
        Base{{"4륜 모바일 베이스"}}
        Arms{{"듀얼 매니퓰레이터 ×2"}}
    end

    Human -->|"🗣️ 자연어 명령"| VLM
    Cams -->|"📨 카메라 3종 토픽"| Fusion
    RSP -->|"📨 /tf"| Fusion
    Fusion -->|"📨 /segmented_objects_3d"| Track
    Track -->|"📨 /tracked_objects<br/>(VLM 파싱용 + HRL 관측값)"| VLM
    Track -->|"📨 /tracked_objects<br/>(HRL의 핵심 관측값=state)"| HRL
    VLM -->|"📨 /task_spec<br/>(목표ID·완료조건·제약·스텝예산)"| HRL
    VLM -->|"📨 /plan_template<br/>(단계별 초안: 1.navigate 2.pick 3.navigate 4.place)"| Coord
    Fusion -->|"📨 /segmented_objects_3d"| SLAM
    RSP -->|"📨 /tf"| SLAM
    MCU -->|"📨 /joint_states"| SLAM
    SLAM -->|"📨 /map"| Planner

    HRL -->|"📨 /sub_goals<br/>(원본과 동일 인터페이스:<br/>navigate_to(id), pick(id)...)"| Coord
    Coord -->|"🎬 NavigateToPose<br/>(Track에서 좌표 조회)"| BT
    Coord -->|"🎬 MoveGroup (pick/place)"| MoveGroup
    BT -->|"🎬 ComputePathToPose"| Planner
    BT -->|"🎬 FollowPath"| Controller
    BT -->|"🎬 Spin/BackUp"| Behavior

    Controller -->|"📨 /cmd_vel_nav2"| RLNav
    MoveGroup -->|"📨 JointTrajectory"| RLArm
    Cams -->|"📨 raw 이미지"| RLNav
    Cams -->|"📨 raw 이미지"| RLArm
    MCU -->|"📨 /odom"| RLNav
    MCU -->|"📨 /joint_states"| RLArm
    RLNav -->|"📨 /cmd_vel_final"| MCU
    RLArm -->|"📨 joint_cmd_final"| MCU
    MCU -->|"🔌 시리얼"| Base
    MCU -->|"🔌 시리얼"| Arms
    Base -.->|"🔌 엔코더"| MCU
    Arms -.->|"🔌 엔코더"| MCU

    Behavior -.->|"🎬 이동 복구 실패 FAIL"| Coord
    MoveGroup -.->|"🎬 pick/place 결과"| Coord
    Coord -->|"📨 파지 검증 재조회"| Track
    Coord -->|"📨 (실패 시) 현재 state 갱신<br/>(reasoning 아님, 그냥 재질의)"| HRL
    Coord -->|"📨 N회 연속실패 시 실패리포트+Top-down<br/>(VLM 재계획 요청)"| VLM

    classDef custom fill:#ffcdd2,stroke:#c62828,color:#1b1b1b;
    classDef provided fill:#c8e6c9,stroke:#2e7d32,color:#1b1b1b;
    classDef rl fill:#ffe0b2,stroke:#e65100,color:#1b1b1b;
    classDef phys fill:#fff9c4,stroke:#f9a825,color:#1b1b1b;
    classDef data fill:#e1bee7,stroke:#6a1b9a,color:#1b1b1b;
    classDef hrl fill:#d1c4e9,stroke:#4527a0,color:#1b1b1b;
    class Fusion,Track,Coord,MCU,VLM custom;
    class SLAM,BT,Planner,Controller,Behavior,MoveGroup provided;
    class RLNav,RLArm rl;
    class Cams,Base,Arms phys;
    class RSP data;
    class HRL hrl;
```

**이 문서(VLM+HRL 하이브리드)**: VLM이 최초 1회 자연어를 `task_spec`+`plan_template(단계별 초안)`으로 풀고, 이후 매 단계 HRL이 다음 스킬 1개를 고릅니다. HRL이 막히면 같은 state로 재질의하고, N회 연속실패시에만 VLM이 재계획합니다. 자연어→단계별 처리 전체 흐름은 4.5장 참고.

---

## 1~3장, 5장: 하드웨어 / 인지 / SLAM / Nav2+RL 보정

**원본과 100% 동일**합니다. 실제 토픽명·메시지 타입·다이어그램은 [원본 문서](./ros2-dualarm-multicam-vlm-rl-architecture.md)의 해당 장을 그대로 참고하세요:
- [1장 하드웨어 & TF](./ros2-dualarm-multicam-vlm-rl-architecture.md#1-하드웨어--tf-구성)
- [2장 인지 파이프라인](./ros2-dualarm-multicam-vlm-rl-architecture.md#2-인지-파이프라인-perception--tracking--실제-토픽메시지-타입)
- [3장 SLAM & Nav2 지도 연동](./ros2-dualarm-multicam-vlm-rl-architecture.md#3-slam--nav2-지도-연동)
- [5장 Nav2(전역)+RL(보정) 계층적 제어](./ros2-dualarm-multicam-vlm-rl-architecture.md#5-nav2전역--rl보정-계층적-제어)

---

## 4. `hrl_meta_policy_node` 상세

| 항목 | 내용 |
|---|---|
| 입력 (관측값/State) | `/tracked_objects`(물체 ID+3D좌표+부피 목록), 현재 로봇 위치(`/tf`의 map→base_link), 현재 진행 중인 sub-goal 인덱스, (선택) 사람의 언어 임베딩 |
| 출력 (Action) | `/sub_goals`와 **동일한 인터페이스** — `navigate_to(id)`, `pick(id)`, `place(id)` 등 스킬 함수호출. 다만 **한 번에 전체 시퀀스를 안 내놓고, 매 스텝(또는 매 sub-goal 완료 시)마다 "다음 스킬 하나"만 출력**하는 경우가 많음 (아래 참고) |
| 학습 방식 | 시뮬레이션에서 "전체 작업 완료까지 걸린 시간/이동거리"에 페널티를, "성공"에 보상을 주는 강화학습. `nav_rl_policy`/`manip_rl_policy`보다 **한 단계 위 추상도**(스킬 선택 자체가 행동 공간) |
| 학습 프레임워크 | Options Framework / Hierarchical RL (예: HIRO, Feudal Networks 계열 알고리즘) |

> **VLM처럼 "한 번에 전체 계획"을 안 내놓는 이유**: VLM은 언어 추론으로 한 번에 `[이동, 집기, 이동, 놓기]` 전체 시퀀스를 뽑아낼 수 있지만, RL 정책은 보통 **매 순간 "지금 상태에서 최선의 다음 행동 하나"만 판단**하도록 학습됩니다. 그래서 `coordinator_node`가 sub-goal 하나를 끝낼 때마다 HRL에게 "다음엔 뭐 할까?"를 **매번 다시 물어보는 구조**가 됩니다 — 아래 시퀀스 다이어그램 참고.

```mermaid
sequenceDiagram
    participant Track as (커스텀)::object_tracking_node
    participant HRL as (커스텀)::hrl_meta_policy_node
    participant Coord as (커스텀)::coordinator_node
    participant BT as nav2_bt_navigator::bt_navigator
    participant MoveGroup as moveit_ros_move_group::move_group

    loop 작업 완료까지 반복 (매 sub-goal마다 HRL 재질의)
        Coord->>Track: 현재 /tracked_objects 조회
        Coord->>HRL: 🧠 현재 state 전달, "다음 스킬?" 질의
        HRL-->>Coord: 다음 스킬 1개 (예: navigate_to(Apple_777))
        alt 이동 스킬
            Coord->>BT: 🎬 NavigateToPose
            BT-->>Coord: 결과
        else 조작 스킬
            Coord->>MoveGroup: 🎬 MoveGroup(pick/place)
            MoveGroup-->>Coord: 결과
            Coord->>Track: 파지 검증 재조회
        end
    end
```

> VLM 버전(원본 4장)과 비교하면: VLM 단독은 **최초에 한 번** 전체 계획을 세우고 Coordinator가 그 리스트를 순서대로 소비했지만, 본 하이브리드는 **VLM이 초안+작업명세만 내고 매 스텝 HRL이 다음 1개를 확정**합니다 — HRL 정책 자체가 "지금 상태 → 다음 행동"만 아는 반응형(reactive) 정책이기 때문입니다.

---

## 4.5. VLM 자연어 → 단계별 실행계획 (본 문서 신규)

사용자가 "오른쪽 끝 사과를 옆 바구니에" 툭 던지면 아래 순서로 풀립니다.

| 단계 | 주체 | 입/출력 | 예시 |
|---|---|---|---|
| 0. 목표 파싱 | `vlm_task_planner_node` | 자연어 + `/tracked_objects` + Top-down 1장 → `/task_spec` | `target: Apple_777(오른쪽 끝), goal: Basket_12, done: Apple_777 in Basket_12, budget: 8스텝` |
| 1. 실행계획 초안 | `vlm_task_planner_node` | → `/plan_template` (참고용, 강제 아님) | `1.navigate_to(Apple_777) 2.pick 3.navigate_to(Basket_12) 4.place` |
| 2. 단계별 확정 | `hrl_meta_policy_node` | `task_spec` + 현재 state → 다음 스킬 1개 | `step=0 → navigate_to(Apple_777)` |
| 3. 실행 | `coordinator_node` | `plan_template`의 현재 단계와 HRL 제안을 대조 → `BT`/`MoveGroup` 호출 | 좌표는 `Track`에서 최신값 조회 |
| 4. 검증·전진 | `coordinator_node` | 성공 시 `step+1` → HRL 재질의 | 실패 시 같은 state 재질의, N회 실패 시 VLM 재계획 요청 |

```mermaid
sequenceDiagram
    participant Human as 👤 사람
    participant VLM as (커스텀)::vlm_task_planner_node
    participant Track as (커스텀)::object_tracking_node
    participant HRL as (커스텀)::hrl_meta_policy_node
    participant Coord as (커스텀)::coordinator_node
    participant BT as nav2_bt_navigator::bt_navigator
    participant MoveGroup as moveit_ros_move_group::move_group

    Human->>VLM: 🗣️ "오른쪽 끝 사과를 옆 바구니에"
    Track-->>VLM: 📨 /tracked_objects (Apple_777, Basket_12)
    VLM-->>HRL: 📨 /task_spec (목표·완료조건·제약)
    VLM-->>Coord: 📨 /plan_template (1.navigate 2.pick 3.navigate 4.place)

    loop 단계별 (step=0..3, HRL이 매번 다음 1개 확정)
        Coord->>Track: 최신 좌표 조회
        Coord->>HRL: task_spec + 현재 state, "다음 스킬?"
        HRL-->>Coord: 다음 스킬 1개 (초안과 다를 수 있음)
        alt 이동 단계
            Coord->>BT: 🎬 NavigateToPose
            BT-->>Coord: 성공/실패
        else 집기/놓기 단계
            Coord->>MoveGroup: 🎬 MoveGroup(pick/place)
            MoveGroup-->>Coord: 모션완료
            Coord->>Track: 파지 검증 재조회
        end
        alt 성공
            Coord->>Coord: step+1
        else 실패 (N회 미만)
            Coord->>HRL: 갱신 state로 재질의
        else N회 연속실패
            Coord->>VLM: 📨 실패리포트+Top-down (재계획 요청)
            VLM-->>HRL: 📨 새 /task_spec
            VLM-->>Coord: 📨 새 /plan_template
        end
    end
```

> 포인트: VLM 초안(`plan_template`)은 **힌트**이지 강제 시퀀스가 아닙니다. HRL이 현재 state에서 더 효율적인順序를 알면 초안과 다르게 골라도 됩니다 (예: 바구니가 먼저 가까우면 placeable한 다른 물체부터). VLM 재호출은 N회 실패·모호지시 때만이라 실시간 루프 밖입니다.

---

## 6. 돌발상황 처리 — VLM의 "재계획"과 근본적으로 다름

**원본 문서(VLM 버전)**: 실패 발생 → Coordinator가 실패 리포트+이미지를 모아서 VLM에게 보냄 → VLM이 **"왜 실패했는지 추론해서 대안을 설계"**(예: "오른쪽이 막혔으니 왼쪽으로 우회") → 새 sub-goal 전달.

**이 문서(VLM+HRL 하이브리드)**: 1차는 HRL 재질의입니다. `coordinator_node`가 실패를 감지하면 **갱신된 현재 상태를 다시 HRL에 넣고 "다음 스킬?"을 한 번 더 물어봅니다**. HRL 정책이 학습 중에 "이 상황에서 이 행동은 실패했었다"는 걸 이미 체득했다면 알아서 다른 스킬을 고르지만, **그런 상황을 학습 때 충분히 경험하지 못했다면 같은 실패를 반복할 수 있어서 N회 연속실패 시에는 VLM에 실패리포트+Top-down을 보내 재계획**합니다 (4.5장 시퀀스 참고). VLM 단독 문서처럼 매 실패마다 VLM을 부르지 않는 게 차이입니다.

```mermaid
flowchart TD
    Fail["🚨 이동/파지 실패 감지<br/>(6장 원본과 동일한 감지 메커니즘)"] --> Coord["🔧 coordinator_node<br/>실패를 '이유'가 아니라<br/>'현재 상태 변화'로만 취급"]
    Coord -->|"갱신된 state<br/>(예: 이 경로 막힘 플래그)"| HRL["🔧 hrl_meta_policy_node<br/>같은 정책을 다시 실행<br/>('추론'이 아니라 '재질의')"]
    HRL -->|"학습 때 이 상황을<br/>충분히 겪었다면"| Good["✅ 다른 스킬 선택<br/>(예: 우회 경로)"]
    HRL -->|"학습 때 이 상황을<br/>못 겪었다면"| Bad["❌ 같은 실패 반복 가능<br/>(Sim에 없던 상황 = 일반화 실패)"]

    classDef custom fill:#ffcdd2,stroke:#c62828,color:#1b1b1b;
    classDef hrl fill:#d1c4e9,stroke:#4527a0,color:#1b1b1b;
    classDef warn fill:#ffe0b2,stroke:#e65100,color:#1b1b1b;
    class Coord custom;
    class HRL hrl;
    class Bad warn;
```

> **이게 VLM 대비 HRL의 가장 큰 약점입니다.** VLM은 학습 데이터에 없던 상황도 "추론"으로 그럴듯한 대안을 짜낼 수 있지만, RL 정책은 **시뮬레이션 학습 때 경험한 상황 분포를 벗어나면 무너집니다.** 그래서 실무에서는 이 실패 복구 로직에 "N번 연속 같은 실패 시 사람에게 알림" 같은 **하드코딩된 최후 안전장치**를 꼭 같이 넣습니다 (HRL 정책만 믿고 무한 루프 돌게 두면 안 됨).

---

## 7. 학습 파이프라인 (이 문서만의 신규 항목)

VLM 버전에는 없던, HRL 메타 정책 고유의 학습 준비 작업입니다.

| 항목 | 내용 |
|---|---|
| 시뮬레이션 환경 | 듀얼암+3카메라 로봇 전체를 Isaac Sim 등에 구성, 물체 스폰 랜덤화 |
| 상태 공간(State) 설계 | 물체 ID/좌표 목록을 **가변 개수**로 어떻게 고정 크기 텐서로 인코딩할지가 핵심 난제 (Set Transformer, Attention 기반 인코더 등 필요) |
| 보상함수 설계 | `R = R_task_success - R_time_penalty - R_distance_penalty - R_collision` — "얼마나 효율적으로 끝냈는가"가 핵심 |
| 실패 상황 커리큘럼 | 장애물 막힘/파지 실패 시나리오를 **의도적으로 다양하게 주입**해서 학습해야 6장의 "학습 못 한 상황" 문제가 줄어듦 (Domain/Task Randomization) |
| Sim-to-Real 검증 | 실기에서 "학습 때 못 본 배치"를 일부러 테스트해서 일반화 한계를 미리 파악 |

---

## 8. VLM 버전과 비교 + 현실적 하이브리드

| | VLM 기반 (원본 문서) | VLM+HRL 하이브리드 (이 문서) |
|---|---|---|
| 판단 방식 | 언어 추론 | VLM 추론(초안+명세) + 학습된 보상 최적화(단계 확정) |
| 자연어 명령 | ✅ 기본 지원 | ✅ 지원 (VLM이 파싱, 4.5장) |
| 계획 시점 | 최초 1회 전체 시퀀스 생성 | VLM 초안 1회 + 매 sub-goal마다 HRL 재질의 (반응형) |
| 실패 복구 | 추론 기반 재계획 (학습 안 한 상황에도 대응 가능) | HRL 재질의 우선, N회 실패 시 VLM 재계획 (2단 방어) |
| 학습 필요 | 파인튜닝 정도 | VLM 파인튜닝 + **HRL 전용 시뮬레이션 학습 전체** |
| 적합한 상황 | 지시가 매번 바뀌는 경우 | 지시는 자연어로 + 같은 작업 반복 효율 최적화 (예: 물류 피킹) |

**현실적 절충안**: VLM은 "언어→관련 물체 후보"까지만 추리고, "그 물체들을 어떤 순서로 처리해야 가장 효율적인지"라는 최적화 판단만 HRL에 맡기는 조합도 가능합니다 — 자세한 다이어그램은 [원본 문서 8장](./ros2-dualarm-multicam-vlm-rl-architecture.md#8-심화-rl을-상위-개념으로-쓰는-대안--hierarchical-rl)에 있습니다.

---

## 9. 노드 요약 & 제공 여부

| 노드 | 제공 여부 |
|---|---|
| (1~3, 5장 노드 전부) | 원본 문서와 동일 |
| `vlm_task_planner_node` | 🔧 **직접 구현 필요** (자연어→`task_spec`+`plan_template`, 4.5장 참고) |
| `hrl_meta_policy_node` | 🔧 **직접 구현 필요** (+ 전용 Hierarchical RL 학습 파이프라인, 7장 참고) |
| `coordinator_node` | 🔧 **직접 구현 필요** (VLM 초안+ HRL 확정 병합, N회 실패 시 VLM 재계획 요청) |

---

## 10. 참고: 다른 문서와의 관계

- **원본(VLM 기반)**: [ros2-dualarm-multicam-vlm-rl-architecture.md](./ros2-dualarm-multicam-vlm-rl-architecture.md) — 하드웨어/인지/SLAM/Nav2+RL 보정 레이어의 원본
- **단일암 TurtleBot3 5종 문서**: [룰베이스](./ros2-turtlebot3-manipulator-example.md) · [E2E](./ros2-turtlebot3-manipulator-e2e.md) · [RL(Mapless)](./ros2-turtlebot3-manipulator-simtoreal-rl.md) · [하이브리드](./ros2-turtlebot3-manipulator-hybrid.md) · [VLA](./ros2-turtlebot3-manipulator-vla.md)
- **개념적 원형**: [ros2-architecture.md](./ros2-architecture.md)
