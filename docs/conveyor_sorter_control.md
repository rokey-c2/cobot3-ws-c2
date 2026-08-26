# Conveyor / Wheel Sorter 제어 기준

작성 기준: 2026-08-26  
작업 브랜치: `hwi_new_sorter`

이 문서는 현재 Parcel Sorting Map에서 사용하는 Conveyor / Wheel Sorter 제어의 최신 기준이다.
`hwi_conveyor_test`의 과거 코드는 참고자료로만 사용하고, 현재 구현은 이 문서를 기준으로 작성한다.

## 1. 구현 원칙

- 현재 USD의 Conveyor / Sorter ActionGraph를 삭제하거나 새로 만들지 않는다.
- 새 ROS2 Node나 새 OmniGraph Node를 추가하지 않는다.
- Python에서는 기존 Graph의 값만 읽고 변경한다.
- 과거의 `binary_switch` / `reroute` 기반 제어는 현재 맵에서 사용하지 않는다.
- Wheel Sorter 방향은 기존 `Sorter/ActionGraph/conveyor_belt.inputs:direction`을 직접 변경한다.
- `SorterSpeed`는 `-1.0`으로 유지한다.
- 박스가 소터를 통과한 뒤에는 sorter direction을 기본값 `(1.0, 0.0, 0.0)`으로 자동 복귀시킨다.
- `ConveyorTrack_05`는 이번 Conveyor / Sorter 제어 대상에서 제외한다.
- 기존 AMR / P3020 / Nav2 성공 코드는 시연 2에서 호출하지 않는다.

## 2. hwi_conveyor_test에서 가져온 참고자료

현재 브랜치에 같은 이름의 파일이 이미 있는 경우 충돌을 피하기 위해 별도 이름으로 보존했다.

```text
isaac_sim/equipment/conveyor/conveyor_controller_hwi_conveyor_test_reference.py
isaac_sim/equipment/wheel_sorter/wheel_sorter_controller_hwi_conveyor_test_reference.py
docs/conveyor_sorter_control_hwi_conveyor_test_reference.md
```

현재 브랜치에 없었던 박스 스폰 코드는 원래 이름으로 가져온 뒤, 현재 프로젝트의 CardBox 물리 생성 함수를 재사용하도록 최신화했다.

```text
isaac_sim/cargo/box_spawner.py
```

참고본의 다음 내용은 현재 구현에서 사용하지 않는다.

```text
/World/ConveyorTrack/Sorter/ActionGraph/binary_switch.inputs:value
단일 ConveyorTrack 구조
simulation step마다 자동 toggle
예전 맵 전용 REVERSED_GRAPH_PATHS
```

## 3. 현재 ConveyorTrack 구성

현재 번호형 ConveyorTrack은 번호가 연속적이지 않다.
존재하지 않는 번호를 코드에서 임의로 가정하지 않는다.

현재 확인한 트랙:

```text
ConveyorTrack
ConveyorTrack_01
ConveyorTrack_02
ConveyorTrack_03
ConveyorTrack_04
ConveyorTrack_05
ConveyorTrack_06
ConveyorTrack_07
ConveyorTrack_10
ConveyorTrack_11
ConveyorTrack_16
ConveyorTrack_17
```

`ConveyorTrack_05`는 제어 대상에서 제외한다.

## 4. Conveyor 속도 기준

### ConveyorTrack_01 / 02 / 03

이 세 트랙은 Wheel Sorter가 있고 Conveyor Belt Graph가 2개씩 존재한다.

```text
ConveyorBeltGraph
    Velocity = 1.0

ConveyorBeltGraph_01
    Velocity = -1.0
```

| Track | Graph | Velocity |
|---|---|---:|
| 01 | ConveyorBeltGraph | `1.0` |
| 01 | ConveyorBeltGraph_01 | `-1.0` |
| 02 | ConveyorBeltGraph | `1.0` |
| 02 | ConveyorBeltGraph_01 | `-1.0` |
| 03 | ConveyorBeltGraph | `1.0` |
| 03 | ConveyorBeltGraph_01 | `-1.0` |

### 나머지 제어 대상 트랙

번호 없는 `/World/ConveyorTrack`과 `04`, `06`, `07`, `10`, `11`, `16`, `17`은 Conveyor Belt Graph가 하나만 존재하며 모두 다음 값을 사용한다.

```text
ConveyorBeltGraph
    Velocity = 1.0
```

즉 다음 경로들도 모두 `1.0`이다.

```text
/World/ConveyorTrack/ConveyorBeltGraph
/World/ConveyorTrack_04/ConveyorBeltGraph
/World/ConveyorTrack_06/ConveyorBeltGraph
/World/ConveyorTrack_07/ConveyorBeltGraph
/World/ConveyorTrack_10/ConveyorBeltGraph
/World/ConveyorTrack_11/ConveyorBeltGraph
/World/ConveyorTrack_16/ConveyorBeltGraph
/World/ConveyorTrack_17/ConveyorBeltGraph
```

### 제외

```text
ConveyorTrack_05
```

Python이 `ConveyorTrack_05`의 Graph 값을 변경하지 않는다.

## 5. Wheel Sorter 대상

Wheel Sorter는 정확히 다음 세 트랙을 제어 대상으로 사용한다.

```text
ConveyorTrack_01
ConveyorTrack_02
ConveyorTrack_03
```

ActionGraph 기준 경로:

```text
/World/ConveyorTrack_01/Sorter/ActionGraph
/World/ConveyorTrack_02/Sorter/ActionGraph
/World/ConveyorTrack_03/Sorter/ActionGraph
```

## 6. SorterSpeed

세 소터 모두 다음 값을 사용한다.

```text
SorterSpeed = -1.0
```

예시:

```text
/World/ConveyorTrack_01/Sorter/ActionGraph/SorterSpeed.inputs:value
```

`SorterSpeed`는 실행 중 분류 조건에 따라 바꾸지 않는다.
초기화 시 `-1.0`을 적용하고, 정상 실행 중에는 계속 유지한다.

## 7. 실제 Wheel Sorter 방향 제어

현재 맵에서 실제 물리 방향을 바꾸는 대상은 다음 노드다.

```text
/World/ConveyorTrack_XX/Sorter/ActionGraph/conveyor_belt
```

변경하는 입력:

```text
inputs:direction
```

검증한 두 상태:

```text
sorter_state = 0 / False
    direction = (1.0, 0.0, 0.0)

sorter_state = 1 / True
    direction = (1.0, -2.0, 0.0)
```

Python에서는 논리 상태를 다음처럼 사용한다.

```text
0 / False -> STRAIGHT -> (1.0, 0.0, 0.0)
1 / True  -> DIVERT   -> (1.0, -2.0, 0.0)
```

기존 ActionGraph 안의 `Direction` Constant Bool, `binary_switch`, `reroute`를 다시 연결하거나 새 노드를 만들지 않는다.

## 8. 시연 2 Box 사양

시연 2에서 사용하는 Box는 Isaac Sim 기본 Simple Warehouse CardBox 에셋이다.

```text
Asset:
Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxB_01_359.usd

Scale:
(0.75, 0.75, 0.5)

Spawn world position:
(-0.5, 0.0, 1.2)

Mass:
15.0 kg
```

현재 프로젝트의 `add_parcel_asset_scaled()`를 재사용하여 다음을 처리한다.

```text
NVIDIA CardBox visual reference
non-uniform scale
RigidBody
Mass
Collision
custom int attribute: box_id
```

박스 생성 래퍼:

```text
isaac_sim/cargo/box_spawner.py
```

## 9. box_id 분류 규칙

시연 2에서는 `box_id`를 `1`, `2`, `3`, `4` 중 랜덤으로 생성한다.

확정 매핑:

```text
box_id = 1
    -> ConveyorTrack_01에서 DIVERT

box_id = 2
    -> ConveyorTrack_01은 STRAIGHT
    -> ConveyorTrack_02에서 DIVERT

box_id = 3
    -> ConveyorTrack_01, 02는 STRAIGHT
    -> ConveyorTrack_03에서 DIVERT

box_id = 4
    -> ConveyorTrack_01, 02, 03 모두 STRAIGHT
    -> 세 소터를 모두 통과
```

각 소터에서 Box가 접근하면 해당 Box의 목적 Track인지 확인한다.
목적 Track이면 `state=1`, 아니면 `state=0`을 사용한다.

```text
state=0 -> direction=(1,0,0)
state=1 -> direction=(1,-2,0)
```

Box가 소터를 통과한 뒤 해당 소터는 항상 다음 기본 상태로 복귀한다.

```text
state=0
Direction=(1,0,0)
```

## 10. 소터 접근/통과 판정

현재 구현은 각 Box와 각 Sorter의 world XY 거리를 사용한다.

초기 조정값:

```text
approach_threshold = 0.45 m
reset_threshold    = 0.70 m
```

이 값은 새 맵에서 첫 실제 실행으로 미세 조정해야 할 수 있다.
코드는 실제 접근 거리를 로그로 출력하므로 반응이 너무 빠르거나 늦으면 상수만 수정한다.

관련 파일:

```text
isaac_sim/equipment/wheel_sorter/wheel_sorter_controller.py
```

## 11. 시연 2 — Wheel Sorter 단독 시연

전용 진입 파일:

```text
isaac_sim/sorter_demo.py
```

실행 스크립트:

```text
scripts/run_sorter_demo.sh
```

실행:

```bash
bash scripts/run_sorter_demo.sh
```

동작 흐름:

```text
Parcel_Sorting_Map 로드
    -> 맵에 있는 AMR / P3020 모델은 그대로 유지
    -> AMR controller 시작 안 함
    -> P3020 controller 시작 안 함
    -> Conveyor 초기 속도 적용
    -> Sorter 01 / 02 / 03 SorterSpeed=-1 적용
    -> Sorter direction 기본값 (1,0,0)
    -> 10초마다 CardBox 생성
    -> random box_id 1~4
    -> Conveyor 이동
    -> Box와 Sorter XY 거리 확인
    -> 해당 box_id 목적 Track이면 DIVERT
    -> 아니면 STRAIGHT
    -> Box 통과 후 STRAIGHT 자동 복귀
```

기본 박스 생성 간격:

```text
10.0 seconds
```

## 12. 시연 1 — 이후 결합

시연 1은 이미 성공한 단일 Box 기준으로 AMR + P3020 + Conveyor + Wheel Sorter 전체 흐름을 보여준다.

시연 2에서 만든 다음 코드는 그대로 재사용한다.

```text
ConveyorController
WheelSorterController
box_id 분류 규칙
```

시연 1로 갈 때 제거하는 것은 주기적인 랜덤 Box 생성 부분이다.

```text
AMR
 -> 단일 Box 운반
 -> P3020 작업
 -> Conveyor
 -> Box의 box_id
 -> Wheel Sorter
```

## 13. 초기값 유지와 reset 검증

Python에서 Graph 값을 한 번 설정한 뒤 다른 코드나 Graph가 덮어쓰지 않으면 실행 중 해당 값은 유지되는 것을 전제로 한다.
매 simulation step마다 Conveyor Velocity나 SorterSpeed를 반복해서 쓰지 않는다.

시연 2에서는 다음 순서로 초기값을 적용한다.

```text
맵 로드
 -> world.reset()
 -> world.play()
 -> 기존 OmniGraph가 live 될 때까지 몇 step 진행
 -> Conveyor setup/start
 -> Sorter setup/start
 -> verify 로그 출력
```

검증 항목:

```text
1. 시작 직후 값
2. reset/play 이후 값
3. 장시간 실행 후에도 값 유지 여부
4. direction 전환 후 기본값 자동 복귀 여부
```

## 14. 구현 시 건드리지 않는 것

- `ConveyorTrack_05`
- 기존 Sorter ActionGraph 구조
- 기존 Conveyor physics
- 새 OmniGraph Node 추가
- 새 ROS2 Node 추가
- AMR Nav2 / LiDAR 설정
- P3020 기존 성공 로직
- 기존 `main_mission.py`의 AMR/P3020 미션 흐름

## 15. 핵심 기준 요약

```text
ConveyorTrack
    ConveyorBeltGraph = 1.0

Track 01 / 02 / 03
    ConveyorBeltGraph     =  1.0
    ConveyorBeltGraph_01  = -1.0
    SorterSpeed           = -1.0
    Sorter direction      = (1,0,0) <-> (1,-2,0)

Track 04 / 06 / 07 / 10 / 11 / 16 / 17
    ConveyorBeltGraph = 1.0

Track 05
    제어 대상 제외

Wheel Sorter 상태
    0 / False -> (1,0,0)
    1 / True  -> (1,-2,0)

Box
    CardBoxB_01
    scale=(0.75,0.75,0.5)
    spawn=(-0.5,0.0,1.2)
    random box_id=1~4

box_id routing
    1 -> Track 01
    2 -> Track 02
    3 -> Track 03
    4 -> PASS ALL

과거 binary_switch
    현재 구현에서 사용하지 않음
```
