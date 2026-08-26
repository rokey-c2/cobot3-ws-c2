# Conveyor / Wheel Sorter 작업 정리

작성 기준: 2026-08-26  
작업 브랜치: `hwi_new_sorter`

이 문서는 `hwi_new_sorter` 브랜치에서 지금까지 진행한 Conveyor / Wheel Sorter 작업 내용과 현재 최종 설정을 정리한 문서다.
현재 시연 2는 AMR/P3020을 동작시키지 않고, 같은 Parcel Sorting Map에서 Conveyor + Wheel Sorter + 랜덤 Box만 독립적으로 테스트하는 구조다.

---

## 1. 작업 목적

현재 맵을 그대로 보존하면서 기존 `hwi_conveyor_test`의 Conveyor / Wheel Sorter 코드를 참고해 새 맵 구조에 맞는 제어 코드를 만드는 것이 목적이다.

구현 원칙은 다음과 같다.

- 기존 USD의 Conveyor / Sorter ActionGraph를 그대로 사용한다.
- 새 ROS2 Node를 만들지 않는다.
- 새 OmniGraph Node를 만들지 않는다.
- Python에서 기존 Graph의 입력값만 변경한다.
- 과거 `binary_switch` / `reroute` 방식은 사용하지 않는다.
- `ConveyorTrack_05`는 이번 제어 대상에서 제외한다.
- 시연 2에서는 AMR / P3020 / Nav2 제어 코드를 실행하지 않는다.

---

## 2. 과거 코드 참고본 보존

`hwi_conveyor_test`에서 사용하던 코드를 완전히 버리지 않고 현재 브랜치에 참고용으로 보존했다.
같은 이름의 현재 파일과 충돌하지 않도록 이름을 변경했다.

```text
isaac_sim/equipment/conveyor/conveyor_controller_hwi_conveyor_test_reference.py
isaac_sim/equipment/wheel_sorter/wheel_sorter_controller_hwi_conveyor_test_reference.py
docs/conveyor_sorter_control_hwi_conveyor_test_reference.md
```

현재 실제 실행에는 위 reference 파일을 직접 사용하지 않는다.
현재 구현 파일은 다음과 같다.

```text
isaac_sim/equipment/conveyor/conveyor_controller.py
isaac_sim/equipment/wheel_sorter/wheel_sorter_controller.py
isaac_sim/cargo/box_spawner.py
isaac_sim/sorter_demo.py
scripts/run_sorter_demo.sh
```

---

## 3. 현재 사용하는 맵

시연 2 전용 실행 파일은 다음 USD를 연다.

```text
isaac_sim/usd/Parcel_Sorting_Map/Parcel_Sorting_Map.usd
```

맵에 이미 들어 있는 AMR / P3020 모델은 그대로 유지하지만 시연 2에서는 해당 controller를 시작하지 않는다.

---

## 4. 현재 ConveyorTrack 구성

현재 확인한 ConveyorTrack은 다음과 같다.

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

번호가 연속적이지 않으므로 코드에서 존재하지 않는 Track 번호를 임의로 생성하지 않는다.

`ConveyorTrack_05`는 Python 제어 대상에서 제외한다.

---

## 5. Conveyor 속도 설정

### 번호 없는 ConveyorTrack

```text
/World/ConveyorTrack/ConveyorBeltGraph
Velocity = 1.0
```

### ConveyorTrack_01 / 02 / 03

Wheel Sorter가 있는 세 Track은 Conveyor Graph가 두 개다.

```text
ConveyorBeltGraph
Velocity = 1.0

ConveyorBeltGraph_01
Velocity = -1.0
```

즉:

```text
Track 01 : +1.0 / -1.0
Track 02 : +1.0 / -1.0
Track 03 : +1.0 / -1.0
```

### 나머지 제어 Track

다음 Track은 `ConveyorBeltGraph = 1.0`을 사용한다.

```text
ConveyorTrack_04
ConveyorTrack_06
ConveyorTrack_07
ConveyorTrack_10
ConveyorTrack_11
ConveyorTrack_16
ConveyorTrack_17
```

### 제외

```text
ConveyorTrack_05
```

이 Track의 Graph 값은 Python에서 변경하지 않는다.

---

## 6. Conveyor Velocity 적용 방식

Conveyor Velocity는 simulation step마다 계속 덮어쓰지 않는다.

현재 초기화 순서는 다음과 같다.

```text
USD 맵 로드
 -> world.reset()
 -> world.play()
 -> OmniGraph가 활성화될 때까지 몇 step 진행
 -> Conveyor setup()
 -> Conveyor start()
 -> verify()
```

다른 코드나 Graph가 값을 다시 덮어쓰지 않는 한 설정한 Velocity가 유지되는 구조다.

---

## 7. Wheel Sorter 대상

현재 Python에서 제어하는 Wheel Sorter는 세 개다.

```text
ConveyorTrack_01
ConveyorTrack_02
ConveyorTrack_03
```

ActionGraph 경로:

```text
/World/ConveyorTrack_01/Sorter/ActionGraph
/World/ConveyorTrack_02/Sorter/ActionGraph
/World/ConveyorTrack_03/Sorter/ActionGraph
```

---

## 8. SorterSpeed

세 Wheel Sorter 모두 고정값을 사용한다.

```text
SorterSpeed = -1.0
```

예시 경로:

```text
/World/ConveyorTrack_01/Sorter/ActionGraph/SorterSpeed.inputs:value
```

`SorterSpeed` 역시 정상 실행 중 simulation step마다 반복해서 변경하지 않는다.

---

## 9. Wheel Sorter 방향 제어

과거 `binary_switch` 대신 기존 ActionGraph의 실제 Conveyor 입력을 직접 변경한다.

제어 대상:

```text
/World/ConveyorTrack_XX/Sorter/ActionGraph/conveyor_belt.inputs:direction
```

현재 확정 상태는 다음과 같다.

```text
STRAIGHT / False / 0
    direction = (1.0, 0.0, 0.0)

DIVERT / True / 1
    direction = (1.0, -2.0, 0.0)
```

즉 Python 내부에서는 논리적으로 False / True를 사용하지만 새로운 Binary Switch OmniGraph Node를 만드는 방식은 아니다.

---

## 10. Box 생성 방식

시연 2에서는 Isaac Sim 기본 Simple Warehouse CardBox를 사용한다.

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

기존 프로젝트의 `add_parcel_asset_scaled()`를 재사용한다.

```text
NVIDIA CardBox visual reference
RigidBody
Mass
Collision
non-uniform scale
custom int attribute: box_id
```

Box 생성 래퍼:

```text
isaac_sim/cargo/box_spawner.py
```

---

## 11. box_id 분류 규칙

Box가 생성될 때 `box_id`는 `1 ~ 4` 중 랜덤으로 선택한다.

현재 확정 매핑:

```text
box_id = 1
    -> ConveyorTrack_01에서 DIVERT

box_id = 2
    -> ConveyorTrack_01은 STRAIGHT
    -> ConveyorTrack_02에서 DIVERT

box_id = 3
    -> ConveyorTrack_01은 STRAIGHT
    -> ConveyorTrack_02는 STRAIGHT
    -> ConveyorTrack_03에서 DIVERT

box_id = 4
    -> ConveyorTrack_01 STRAIGHT
    -> ConveyorTrack_02 STRAIGHT
    -> ConveyorTrack_03 STRAIGHT
    -> 모든 Sorter 통과
```

---

## 12. Box 스폰 주기 테스트 과정

처음에는 다음 값을 사용했다.

```text
BOX_SPAWN_INTERVAL_SECONDS = 10.0
```

실제 테스트에서 simulation time 10초가 실제 스톱워치 기준 약 `29.93초` 정도 걸리는 것을 확인했다.
즉 현재 실행 환경에서는 simulation time과 실제 시간이 1:1로 진행되지 않는다.

이후 Box 수를 늘리기 위해 다음 순서로 테스트했다.

```text
10.0
 -> 3.0
 -> 1.0
 -> 2.0
```

`1.0`은 Box 간격이 너무 좁아 앞 Box 분류가 끝나기 전에 다음 Box가 접근하는 문제가 있었다.

현재 선택한 값:

```text
BOX_SPAWN_INTERVAL_SECONDS = 2.0
```

현재 테스트에서는 이 간격이 가장 적당한 Box 간격으로 판단했다.

주의:

```text
2.0은 실제 벽시계 2초가 아니라 simulation time 2초다.
```

---

## 13. Approach / Reset 동작

현재 분류 판정은 Box 중심과 Sorter 중심의 `world XY 거리`를 사용한다.

최종 현재값:

```text
approach_threshold = 0.45 m
reset_threshold    = 0.70 m
```

### Approach

Box와 Sorter 중심의 거리가 `0.45 m 이하`가 되는 순간 Box의 `box_id`를 확인하고 방향을 결정한다.

```text
목적 Track
    -> DIVERT
    -> direction=(1,-2,0)

목적 Track 아님
    -> STRAIGHT
    -> direction=(1,0,0)
```

### Reset

한 번 접근 판정된 Box가 Sorter를 지나 다시 `0.70 m 이상` 멀어지면 해당 Sorter를 기본 상태로 되돌린다.

```text
state = False
Direction = (1,0,0)
```

---

## 14. Approach / Reset 튜닝 과정

초기 구현값은 다음이었다.

```text
Approach = 0.45 m
Reset    = 0.70 m
```

Box를 더 촘촘하게 보내는 테스트 중 다음 문제를 확인했다.

```text
앞 Box가 DIVERT 상태를 남겨놓은 상태에서
다음 Box가 도착하면
다음 Box가 정상적인 STRAIGHT 경로로 진입하기 전에 잘못 빠질 수 있음
```

한 번은 자동 Reset을 제거하고 다음처럼 테스트했다.

```text
Approach = 0.25 m
Reset = 없음
```

하지만 이전 Box의 Sorter 상태가 그대로 남는 문제가 발생했다.
예를 들어 이전 `box_id=1` 때문에 Track_01이 DIVERT인 상태라면 다음 `box_id=2`가 Track_01을 정상 통과하기 어려웠다.

그래서 자동 Reset을 다시 추가했다.

```text
Reset = 0.70 m
```

Approach 값은 다음 순서로 실제 테스트했다.

```text
0.25 m
 -> 판단이 너무 늦음

0.35 m
 -> 여전히 늦음

0.45 m
 -> 현재 테스트에서 가장 적합
```

따라서 현재 최종값은 다시 초기 설정과 동일하다.

```text
Approach = 0.45 m
Reset    = 0.70 m
```

`busy` 상태는 현재 사용하지 않는다.
거리 임계값과 Reset으로 단순하게 처리한다.

---

## 15. 현재 한 Box의 Sorter 동작 예시

### box_id = 2

```text
Box 생성
 -> Conveyor 이동
 -> Track_01 접근 거리 <= 0.45 m
 -> box_id=2 확인
 -> Track_01 STRAIGHT
 -> Track_01 통과
 -> Track_01과 거리 >= 0.70 m
 -> Track_01 STRAIGHT로 Reset

 -> Track_02 접근 거리 <= 0.45 m
 -> box_id=2 확인
 -> Track_02 DIVERT
 -> Track_02로 분류
 -> Track_02와 거리 >= 0.70 m
 -> Track_02 STRAIGHT로 Reset
```

이 Reset 덕분에 다음 Box는 이전 Box가 남긴 DIVERT 상태의 영향을 받지 않고 시작할 수 있다.

---

## 16. 시연 2 전용 실행 파일

시연 2 진입 파일:

```text
isaac_sim/sorter_demo.py
```

실행 스크립트:

```text
scripts/run_sorter_demo.sh
```

실행 방법:

```bash
cd ~/collaboration/cobot3-ws-c2

git fetch origin
git pull --ff-only origin hwi_new_sorter

bash scripts/run_sorter_demo.sh
```

실행 시 확인할 핵심 로그:

```text
[BOX] interval: 2.0 s
[SORTER] approach threshold=0.45 m
[SORTER] reset threshold=0.70 m
```

---

## 17. 시연 2 전체 흐름

```text
Parcel_Sorting_Map 로드

 -> world.reset()
 -> world.play()

 -> Conveyor 초기 속도 적용
 -> Wheel Sorter 01/02/03 초기화
 -> SorterSpeed=-1.0
 -> 초기 direction=(1,0,0)

 -> 2.0 simulation seconds마다 CardBox 생성
 -> random box_id 1~4 부여

 -> Conveyor 이동
 -> 각 Box와 Sorter의 XY 거리 계산

 -> 0.45 m 이내 접근
 -> box_id 확인
 -> STRAIGHT 또는 DIVERT 적용

 -> 해당 Box가 Sorter에서 0.70 m 이상 멀어짐
 -> STRAIGHT로 자동 Reset

 -> 다음 Box 처리
```

AMR / P3020 controller는 이 흐름에서 실행하지 않는다.

---

## 18. 현재 테스트로 확인한 내용

현재까지 실제 실행으로 확인한 내용:

```text
- sorter_demo.sh 실행 가능
- CardBox 정상 스폰
- Conveyor 위에서 Box 이동
- 랜덤 box_id 생성
- Spawn 1.0은 너무 촘촘함
- Spawn 2.0은 현재 원하는 간격에 가까움
- Approach 0.25는 너무 늦음
- Approach 0.35도 늦음
- Approach 0.45가 현재 가장 적합
- Reset이 없으면 이전 DIVERT 상태가 다음 Box에 영향을 줄 수 있음
- Reset 0.70을 다시 사용
```

현재 Demo 2의 기준값은 아래 값을 기준으로 유지한다.

---

## 19. 시연 1과 이후 통합

현재 작업은 먼저 시연 2를 안정화하기 위한 것이다.

이후 시연 1에서는 현재 완성된 다음 코드를 그대로 재사용할 수 있다.

```text
ConveyorController
WheelSorterController
box_id routing
```

시연 1에서는 주기적인 랜덤 Box 스폰 대신 기존에 성공했던 단일 Box AMR + P3020 흐름과 연결한다.

예상 흐름:

```text
AMR
 -> 단일 Box 운반
 -> P3020 작업
 -> Conveyor 투입
 -> Box의 box_id 확인
 -> Wheel Sorter 분류
```

현재 시연 2 작업에서는 기존 `main_mission.py`의 AMR/P3020 미션 흐름을 변경하지 않는다.

---

## 20. 건드리지 않는 항목

현재 Conveyor / Sorter 작업에서 다음은 임의로 변경하지 않는다.

```text
ConveyorTrack_05
기존 Sorter ActionGraph 구조
기존 Conveyor physics
새 OmniGraph Node 추가
새 ROS2 Node 추가
AMR Nav2 설정
AMR LiDAR 설정
P3020 기존 성공 로직
기존 main_mission.py의 AMR/P3020 미션 흐름
```

---

## 21. 현재 최종 기준값

```text
[BOX]
Asset   = SM_CardBoxB_01_359.usd
Scale   = (0.75, 0.75, 0.5)
Spawn   = (-0.5, 0.0, 1.2)
Mass    = 15.0 kg
Interval = 2.0 simulation seconds

[ROUTING]
box_id 1 -> Track 01
box_id 2 -> Track 02
box_id 3 -> Track 03
box_id 4 -> PASS ALL

[SORTER]
SorterSpeed = -1.0
STRAIGHT    = (1,0,0)
DIVERT      = (1,-2,0)
Approach    = 0.45 m
Reset       = 0.70 m
Busy        = 사용 안 함

[CONVEYOR]
ConveyorTrack                  = +1.0
Track 01/02/03 Graph           = +1.0
Track 01/02/03 Graph_01        = -1.0
Track 04/06/07/10/11/16/17    = +1.0
Track 05                       = 제어 제외
```

이 값을 현재 `hwi_new_sorter` 시연 2의 기준 설정으로 사용한다.
