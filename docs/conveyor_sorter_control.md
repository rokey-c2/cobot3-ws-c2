# Conveyor / Wheel Sorter 제어 기준

작성 기준: 2026-08-26  
작업 브랜치: `hwi_new_sorter`

이 문서는 현재 Parcel Sorting Map에서 사용하는 Conveyor / Wheel Sorter 제어의 최신 기준이다.
`hwi_conveyor_test`의 과거 코드는 참고자료로만 사용한다.

## 1. 구현 원칙

- 기존 USD의 Conveyor / Sorter ActionGraph를 그대로 사용한다.
- 새 ROS2 Node나 새 OmniGraph Node를 만들지 않는다.
- Python에서는 기존 Graph의 값만 변경한다.
- 과거 `binary_switch` / `reroute` 방식은 사용하지 않는다.
- Wheel Sorter 방향은 기존 `Sorter/ActionGraph/conveyor_belt.inputs:direction`을 직접 변경한다.
- `SorterSpeed = -1.0`을 유지한다.
- `ConveyorTrack_05`는 제어하지 않는다.
- 시연 2에서는 AMR / P3020 제어를 실행하지 않는다.

## 2. 현재 Conveyor 속도

### 번호 없는 ConveyorTrack

```text
/World/ConveyorTrack/ConveyorBeltGraph = 1.0
```

### ConveyorTrack_01 / 02 / 03

```text
ConveyorBeltGraph    =  1.0
ConveyorBeltGraph_01 = -1.0
```

### 단일 Graph 트랙

```text
ConveyorTrack_04 = 1.0
ConveyorTrack_06 = 1.0
ConveyorTrack_07 = 1.0
ConveyorTrack_10 = 1.0
ConveyorTrack_11 = 1.0
ConveyorTrack_16 = 1.0
ConveyorTrack_17 = 1.0
```

### 제외

```text
ConveyorTrack_05
```

## 3. Wheel Sorter 대상

```text
ConveyorTrack_01
ConveyorTrack_02
ConveyorTrack_03
```

각 소터의 기존 ActionGraph를 사용한다.

```text
/World/ConveyorTrack_01/Sorter/ActionGraph
/World/ConveyorTrack_02/Sorter/ActionGraph
/World/ConveyorTrack_03/Sorter/ActionGraph
```

## 4. Wheel Sorter 고정값

세 소터 모두:

```text
SorterSpeed = -1.0
```

방향 상태:

```text
STRAIGHT / False / 0
    direction = (1.0, 0.0, 0.0)

DIVERT / True / 1
    direction = (1.0, -2.0, 0.0)
```

실제 변경 대상:

```text
/World/ConveyorTrack_XX/Sorter/ActionGraph/conveyor_belt.inputs:direction
```

## 5. 시연 2 Box 사양

```text
Asset:
Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxB_01_359.usd

Scale:
(0.75, 0.75, 0.5)

Spawn world position:
(-0.5, 0.0, 1.2)

Mass:
15.0 kg

Spawn interval:
2.0 simulation seconds
```

각 Box에는 custom int attribute `box_id`를 넣는다.

## 6. box_id 분류 규칙

```text
box_id = 1
    -> Track 01에서 DIVERT

box_id = 2
    -> Track 01은 STRAIGHT
    -> Track 02에서 DIVERT

box_id = 3
    -> Track 01, 02는 STRAIGHT
    -> Track 03에서 DIVERT

box_id = 4
    -> Track 01, 02, 03 모두 STRAIGHT
    -> 끝까지 통과
```

## 7. 접근 / Reset 판정

현재 구현은 Box 중심과 Sorter 중심의 world XY 거리를 사용한다.

```text
approach_threshold = 0.35 m
reset_threshold    = 0.70 m
```

동작 순서:

```text
Box가 Sorter에서 0.35 m 이내로 접근
    -> box_id 확인
    -> 해당 Sorter 상태 결정
    -> 목적 Track이면 DIVERT
    -> 목적 Track이 아니면 STRAIGHT

해당 Box가 Sorter를 지나 다시 0.70 m 이상 멀어짐
    -> 해당 Sorter를 STRAIGHT로 자동 reset
    -> direction = (1,0,0)
```

Reset을 다시 사용하는 이유는 이전 Box가 DIVERT 상태를 남긴 경우 다음 Box가 접근 임계값에 도달하기 전에 잘못 분기되는 상황을 막기 위해서다.

예:

```text
box_id=1
    -> Track 01 DIVERT
    -> Box 통과
    -> Track 01 reset -> STRAIGHT

다음 box_id=2
    -> Track 01은 이미 STRAIGHT 상태
    -> Track 01 통과
    -> Track 02 접근 시 DIVERT
```

`busy` 상태는 사용하지 않는다.

## 8. 시연 2 실행

진입 파일:

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

전체 흐름:

```text
Parcel_Sorting_Map 로드
 -> AMR/P3020 모델은 그대로 유지
 -> AMR/P3020 controller는 실행하지 않음
 -> Conveyor 속도 적용
 -> SorterSpeed=-1 적용
 -> 초기 direction=(1,0,0)
 -> 2.0 simulation seconds마다 CardBox 생성
 -> random box_id 1~4
 -> 0.35 m 이내 접근 시 분류
 -> Box 통과 후 0.70 m에서 STRAIGHT reset
```

## 9. 초기값 유지

Conveyor Velocity와 SorterSpeed는 매 simulation step마다 반복해서 쓰지 않는다.

```text
맵 로드
 -> world.reset()
 -> world.play()
 -> OmniGraph 활성화 대기
 -> Conveyor setup/start
 -> Sorter setup/start
 -> verify
```

실행 중 반복 변경하는 값은 분류 조건에 따라 달라지는 Sorter `direction`이다.

## 10. 건드리지 않는 것

- `ConveyorTrack_05`
- 기존 Sorter ActionGraph 구조
- 기존 Conveyor physics
- 새 OmniGraph Node
- 새 ROS2 Node
- AMR Nav2 / LiDAR 설정
- P3020 기존 성공 로직
- 기존 `main_mission.py`의 미션 흐름

## 11. 현재 핵심값

```text
Box spawn interval = 2.0 simulation seconds

Approach = 0.35 m
Reset    = 0.70 m
Busy     = 사용 안 함

SorterSpeed = -1.0

STRAIGHT = (1,0,0)
DIVERT   = (1,-2,0)

box_id 1 -> Track 01
box_id 2 -> Track 02
box_id 3 -> Track 03
box_id 4 -> PASS ALL
```
