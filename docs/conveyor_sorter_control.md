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

## 2. hwi_conveyor_test에서 가져온 참고자료

현재 브랜치에 같은 이름의 파일이 이미 있는 경우 충돌을 피하기 위해 별도 이름으로 보존했다.

```text
isaac_sim/equipment/conveyor/conveyor_controller_hwi_conveyor_test_reference.py
isaac_sim/equipment/wheel_sorter/wheel_sorter_controller_hwi_conveyor_test_reference.py
docs/conveyor_sorter_control_hwi_conveyor_test_reference.md
```

현재 브랜치에 없었던 박스 스폰 코드는 재사용 가능한 시작점으로 원래 이름으로 가져왔다.

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

현재 확인한 번호형 트랙:

```text
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

정확한 규칙:

| Track | Graph | Velocity |
|---|---|---:|
| 01 | ConveyorBeltGraph | `1.0` |
| 01 | ConveyorBeltGraph_01 | `-1.0` |
| 02 | ConveyorBeltGraph | `1.0` |
| 02 | ConveyorBeltGraph_01 | `-1.0` |
| 03 | ConveyorBeltGraph | `1.0` |
| 03 | ConveyorBeltGraph_01 | `-1.0` |

### 나머지 제어 대상 트랙

`04`, `06`, `07`, `10`, `11`, `16`, `17`은 Conveyor Belt Graph가 하나만 존재하며 모두 다음 값을 사용한다.

```text
ConveyorBeltGraph
    Velocity = 1.0
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
초기 setup / reset 이후 재적용 / play 이후 확인 시 같은 `-1.0`을 적용한다.

## 7. 실제 Wheel Sorter 방향 제어

현재 맵에서 실제 물리 방향을 바꾸는 대상은 다음 노드다.

```text
/World/ConveyorTrack_XX/Sorter/ActionGraph/conveyor_belt
```

변경하는 입력:

```text
inputs:direction
```

현재 검증한 두 상태:

```text
sorter_state = 0 / False
    direction = (1.0, 0.0, 0.0)

sorter_state = 1 / True
    direction = (1.0, -2.0, 0.0)
```

즉 Python에서는 벡터를 여러 곳에서 직접 작성하지 않고 한 곳에서 다음처럼 매핑한다.

```text
0 / False -> STRAIGHT -> (1.0, 0.0, 0.0)
1 / True  -> DIVERT   -> (1.0, -2.0, 0.0)
```

과거 ActionGraph 안의 `Direction` Constant Bool이나 `binary_switch`를 다시 연결하거나 새 노드를 만들지 않는다.
Python의 논리 상태만 0/1 또는 False/True로 사용하고, 그 상태를 기존 `conveyor_belt.inputs:direction` 값으로 변환한다.

## 8. box_id 기준 제어

테스트 Box에는 USD custom int attribute를 추가한다.

```text
box_id
```

박스 생성 함수:

```text
isaac_sim/cargo/box_spawner.py
```

시연 2에서는 일정 주기마다 Box를 생성하고 각 Box에 랜덤 `box_id`를 부여한다.
Wheel Sorter Controller는 박스 위치를 확인하다가 해당 소터의 진입 영역에 도달하면 `box_id`를 읽어 그 소터의 0/1 상태를 결정한다.

기본 동작 흐름:

```text
Box spawn
    -> random box_id
    -> Conveyor 이동
    -> Track 01 / 02 / 03 중 현재 접근한 sorter 판정
    -> box_id에 따른 해당 sorter 상태 결정
    -> direction = (1,0,0) 또는 (1,-2,0)
    -> Box가 sorter 통과
    -> 해당 sorter direction을 (1,0,0)으로 자동 복귀
```

중요: `box_id`가 어느 Track에서 `DIVERT`되어야 하는지에 대한 최종 숫자 매핑은 구현 직전에 별도로 확정한다. 임의로 `box_id=1 -> Track01` 같은 규칙을 만들지 않는다.

## 9. 시연 방식

### 시연 2 — 먼저 구현

목적: Wheel Sorter의 `box_id` 기반 분류 동작을 여러 박스로 명확하게 보여준다.

```text
현재 완성 맵 사용
AMR 스폰 상태 유지, 동작시키지 않음
P3020 스폰 상태 유지, 동작시키지 않음
Conveyor만 동작
Wheel Sorter 01 / 02 / 03 동작
일정 주기마다 random box_id Box 생성
box_id에 따라 sorter 방향 전환
통과 후 기본 방향 자동 복귀
```

AMR / P3020 다중 박스 작업을 시연 2에 넣지 않는다.

### 시연 1 — 이후 결합

목적: 이미 성공한 단일 Box 기준으로 AMR + P3020 + Conveyor + Wheel Sorter 전체 흐름을 보여준다.

시연 2에서 만든 Conveyor / Sorter 로직을 그대로 재사용하고, `주기적인 랜덤 Box 생성`만 제거한 뒤 기존 단일 Box 미션과 결합한다.

```text
AMR
 -> 단일 Box 운반
 -> P3020 작업
 -> Conveyor
 -> Box의 box_id
 -> Wheel Sorter
```

## 10. 초기값 유지와 reset 검증

Python에서 Graph 값을 한 번 설정한 뒤 다른 코드나 Graph가 덮어쓰지 않으면 실행 중 해당 값은 유지되는 것을 전제로 한다.
매 simulation step마다 Conveyor Velocity나 SorterSpeed를 반복해서 쓰지 않는다.

다만 OmniGraph 초기화 구간은 별도로 확인한다.

```text
1. setup 직후
2. world.reset() 직후 재적용
3. world.play() 직후 확인 / 재적용
4. 장시간 실행 후 값 유지 확인
```

따라서 구현은 초기화 구간에서만 값을 재적용하고 정상 실행 중에는 `conveyor_belt.direction`처럼 실제로 조건에 따라 바뀌어야 하는 값만 변경한다.

## 11. 구현 시 건드리지 않는 것

- `ConveyorTrack_05`
- 기존 Sorter ActionGraph 구조
- 기존 Conveyor physics
- 새 OmniGraph Node 추가
- 새 ROS2 Node 추가
- AMR Nav2 / LiDAR 설정
- P3020 기존 성공 로직

## 12. 핵심 기준 요약

```text
Track 01 / 02 / 03
    ConveyorBeltGraph     =  1.0
    ConveyorBeltGraph_01  = -1.0
    SorterSpeed           = -1.0
    Sorter direction      = (1,0,0) <-> (1,-2,0)

Track 04 / 06 / 07 / 10 / 11 / 16 / 17
    ConveyorBeltGraph     = 1.0

Track 05
    제어 대상 제외

Wheel Sorter 상태
    0 / False -> (1,0,0)
    1 / True  -> (1,-2,0)

Box
    custom attribute: box_id

과거 binary_switch
    현재 구현에서 사용하지 않음
```
