# P3020 Pick & Place Mission Flow — Baseline Snapshot

## 1. 기준 정보

- **Repository:** `https://github.com/rokey-c2/cobot3-ws-c2.git`
- **Branch:** `euiseok-amr`
- **기준 Commit:** `8f084b640c7d50dfd98d51d7df0d968a373874bd`
- **Commit Message:** `Merge pull request #12 from rokey-c2/injae-union-map` / `Injae union map`
- **기준 시점:** 2026-08-24
- **FigJam:** `https://www.figma.com/board/1833sIkwqEzvNOsQS1MEag/P3020-Pick---Place-Mission-Flow?node-id=0-1&t=kCvlPCpkiq4CAuLh-1`

> 이 문서는 `euiseok-amr` 브랜치의 최신 HEAD가 아니라,
> **Commit `8f084b640c7d50dfd98d51d7df0d968a373874bd` 시점의 코드와 FigJam 플로우차트를 기준으로 한다.**
>
> 이후 커밋의 변경사항은 별도로 요청하지 않는 한 이 기준 문서에 포함하지 않는다.

---

## 2. 전체 미션 플로우

```text
START
  ↓
IW Hub Spawn
  ↓
Cargo Pod 접근
  ↓
Lift Up
  ↓
Cargo Pickup 완료?
  ├─ NO → 재시도 / 실패 처리 → MISSION FAIL
  └─ YES
       ↓
     Nav2 자율주행
       ↓
     P3020 작업 위치 도착?
       ├─ NO → Nav2 복구 / 실패 → MISSION FAIL
       └─ YES
            ↓
          YOLO 박스 인식
            ↓
          박스 검출 성공?
            ├─ NO → 재탐지 → 박스 검출 성공? 재확인
            └─ YES
                 ↓
               Pixel + Depth → 3D 좌표
                 ↓
               P3020 작업 범위 안?
                 ├─ NO → P3020 FAIL → MISSION FAIL
                 └─ YES
                      ↓
                    P3020 Pick & Place
                      ↓
                    Pick & Place 성공?
                      ├─ NO → P3020 FAIL → MISSION FAIL
                      └─ YES
                           ↓
                         Action SUCCESS
                           ↓
                         IW Hub Nav2 복귀
                           ↓
                         Cargo 위치 정밀 도킹
                           ↓
                         Lift Down
                           ↓
                         IW Hub Spawn 복귀
                           ↓
                         COMPLETE
```

---

## 3. 단계별 흐름

### 3.1 IW Hub 출발

```text
START
→ IW Hub Spawn
→ Cargo Pod 접근
→ Lift Up
→ Cargo Pickup 완료?
```

- **YES**
  - Nav2 자율주행 시작
- **NO**
  - 재시도 / 실패 처리
  - 최종적으로 `MISSION FAIL`

이후:

```text
Nav2 자율주행
→ P3020 작업 위치 도착?
```

- **YES**
  - Vision / YOLO 단계로 진행
- **NO**
  - Nav2 복구 또는 실패 처리
  - `MISSION FAIL`

---

### 3.2 Vision / YOLO

```text
YOLO 박스 인식
→ 박스 검출 성공?
```

- **NO**
  - 재탐지
  - 다시 박스 검출 여부 확인
- **YES**
  - Pixel + Depth 정보를 사용하여 박스의 3D 좌표 계산

```text
Pixel + Depth
→ 3D 좌표
```

---

### 3.3 P3020 Pick & Place

```text
3D 좌표
→ P3020 작업 범위 안?
```

- **NO**
  - `P3020 FAIL`
  - `MISSION FAIL`
- **YES**
  - P3020 Pick & Place 수행

그 다음:

```text
P3020 Pick & Place
→ Pick & Place 성공?
```

- **NO**
  - `P3020 FAIL`
  - `MISSION FAIL`
- **YES**
  - `Action SUCCESS`

---

### 3.4 IW Hub 복귀

P3020 작업이 정상 완료되면 다음 순서로 복귀한다.

```text
Action SUCCESS
→ IW Hub Nav2 복귀
→ Cargo 위치 정밀 도킹
→ Lift Down
→ IW Hub Spawn 복귀
→ COMPLETE
```

핵심은 단순히 Cargo 근처까지 돌아오는 것이 아니라,

1. Nav2로 Cargo 근처까지 복귀
2. 원래 Cargo 위치에 정밀 도킹
3. Cargo Pod를 Lift Down
4. IW Hub가 처음 Spawn 위치로 복귀
5. 전체 미션 `COMPLETE`

순서로 진행된다는 점이다.

---

## 4. 시스템 아키텍처

FigJam의 `AMR-P3020 System Architecture` 기준 구성은 다음과 같다.

```text
Start / bash script 실행
        ↓
ROS2 Jazzy + Fast DDS
Domain 110
        ↓
AMR-P3020 제어 노드
   ├──────────────→ Nav2 자율주행
   │                 │
   │                 │ /cmd_vel
   │                 │ /odom
   │                 │ /scan
   │                 │ /tf
   │                 ↓
   │              Isaac Sim 5.1
   │              main_mission.py
   │
   └──────────────→ P3020 Pick & Place Action 서버
                     ↕
                   command / status
                     ↕
                 Isaac Sim 5.1
                 main_mission.py

Isaac Sim 5.1
   ├─ Controls AMR ─────────→ IW Hub AMR + LiDAR
   ├─ Controls Arm ─────────→ P3020 + Gripper + Camera
   ├─ Physics ──────────────→ Cargo Pod + Parcel
   └─ /rgb, /box_pixel ↔ YOLO 박스 검출
                         NumPy + ONNX Runtime
```

---

## 5. 주요 통신 관계

### AMR 제어 노드 → Nav2

```text
NavigateToPose
```

Nav2를 이용하여 IW Hub의 목적지 이동과 복귀를 수행한다.

### AMR-P3020 제어 노드 → P3020 Action 서버

```text
/p3020/pick_place
```

P3020 Pick & Place 동작을 Action 방식으로 요청한다.

### Nav2 ↔ Isaac Sim

주요 데이터:

```text
/cmd_vel
/odom
/scan
/tf
```

- `/cmd_vel`: AMR 이동 명령
- `/odom`: AMR 위치 및 이동량
- `/scan`: LiDAR 데이터
- `/tf`: 좌표계 변환

### Isaac Sim ↔ YOLO

```text
/rgb
/box_pixel
```

카메라 이미지와 박스 검출 결과를 이용하여 실제 Pick & Place 좌표 계산에 사용한다.

---

## 6. 시뮬레이션 환경 주요 대상

Isaac Sim에서 직접 제어 또는 물리 시뮬레이션하는 주요 대상:

### IW Hub

```text
IW Hub AMR + LiDAR
```

담당:

- Cargo Pod 접근
- Lift Up / Down
- Nav2 이동
- 장애물 회피
- 정밀 도킹
- Spawn 복귀

### P3020

```text
P3020 + Gripper + Camera
```

담당:

- 박스 위치 확인
- Pick
- Place
- 작업 완료 상태 전달

### Cargo

```text
Cargo Pod + Parcel
```

담당:

- IW Hub Lift 대상
- 운반 대상
- P3020 Pick 대상 Parcel 보관

---

## 7. 기준 코드 구조와 연결되는 핵심 파일

Commit `8f084b...` 시점에서 미션 흐름과 직접 관련된 핵심 파일은 다음과 같다.

### Isaac Sim

```text
isaac_sim/
├── main.py
├── main_mission.py
├── cargo/
│   ├── cargo_pod_physics.py
│   ├── container_payload.py
│   └── container_policy.py
├── project_config/
│   ├── robot_config.py
│   └── simulation_config.py
└── robots/
    ├── iw_hub/
    │   ├── iw_hub_agent.py
    │   ├── iw_hub_mission_agent.py
    │   └── iw_hub_v2.usda
    └── p3020/
        ├── contact_gripper.py
        ├── p3020_mission_agent.py
        └── vision/
            ├── box_detector_node.py
            ├── locate_box_relay.py
            └── object_detector.py
```

### ROS2

```text
ros2_ws/src/
├── amr_controller/
│   ├── amr_controller/
│   │   ├── amr_p3020_mission.py
│   │   ├── container_mission.py
│   │   ├── map_utils.py
│   │   ├── mission_policy.py
│   │   ├── static_map_publisher.py
│   │   ├── velocity_mux.py
│   │   └── velocity_mux_policy.py
│   ├── config/
│   │   ├── amr.yaml
│   │   └── nav2_params.yaml
│   └── launch/
│       └── amr_nav2.launch.py
│
├── arm_controller/
│   └── arm_controller/
│       └── pick_place_action_server.py
│
├── logistics_interfaces/
│   ├── action/
│   │   └── PickPlace.action
│   ├── msg/
│   │   ├── BoxStatus.msg
│   │   └── RobotStatus.msg
│   └── srv/
│       ├── ConfirmGrasp.srv
│       ├── LocateBox.srv
│       └── SetRoute.srv
│
└── vision_node/
    ├── config/
    │   └── vision.yaml
    └── vision_node/
        ├── locate_box_node.py
        └── object_detector.py
```

### 실행 스크립트

```text
scripts/
├── run_amr_p3020_mission.sh
├── run_isaac.sh
├── run_isaac_mission.sh
├── run_ros2.sh
├── setup_ros.sh
└── setup_vision_env.sh
```

---

## 8. 기준 환경

```text
Ubuntu 24.04
Isaac Sim 5.1.0
ROS2 Jazzy
ROS_DOMAIN_ID=110
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

ROS2 Middleware:

```text
ROS2 Jazzy
Fast DDS
Domain 110
```

---

## 9. 앞으로의 작업 기준

이 프로젝트의 AMR / P3020 통합 작업에서는 아래 두 자료를 하나의 기준 상태로 본다.

### Source of Truth 1 — Code

```text
Repository:
rokey-c2/cobot3-ws-c2

Branch:
euiseok-amr

Commit:
8f084b640c7d50dfd98d51d7df0d968a373874bd
```

### Source of Truth 2 — Mission Flow

```text
P3020 Pick & Place Mission Flow
FigJam Board
```

따라서 이후 기능을 분석하거나 수정할 때는 다음을 확인한다.

- FigJam에는 정의되어 있는데 코드에는 없는 단계가 있는지
- 코드 동작이 FigJam의 순서와 다른지
- 실패 처리 경로가 구현되어 있는지
- Nav2 복귀 후 정밀 도킹 단계가 구현되어 있는지
- Lift Down 후 IW Hub Spawn 복귀까지 이어지는지
- P3020 작업 완료가 Action SUCCESS로 정상 반환되는지
- YOLO → Pixel/Depth → 3D 좌표 → Pick & Place 흐름이 연결되어 있는지

---

## 10. 최종 기준 시나리오

```text
IW Hub Spawn
→ Cargo Pod 접근
→ Lift Up
→ Cargo Pickup 확인
→ Nav2 자율주행
→ P3020 작업 위치 도착
→ YOLO 박스 검출
→ Pixel + Depth 기반 3D 좌표 계산
→ P3020 작업 범위 확인
→ P3020 Pick & Place
→ Action SUCCESS
→ IW Hub Nav2 복귀
→ Cargo 원래 위치 정밀 도킹
→ Lift Down
→ IW Hub Spawn 위치 복귀
→ COMPLETE
```

이 흐름을 **Commit `8f084b640c7d50dfd98d51d7df0d968a373874bd` 시점의 기준 미션**으로 사용한다.
