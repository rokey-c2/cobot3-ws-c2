# Conveyor / Wheel Sorter 제어

이 문서는 `feat/iw-hub-single-navigation` 브랜치에서 Isaac Sim 컨베이어 벨트와 Wheel Sorter를 Python으로 제어하는 방법을 정리한다.

## 목적

- 컨베이어 벨트 속도를 Python에서 `1.0`으로 설정한다.
- Wheel Sorter의 기존 ActionGraph를 삭제하거나 다시 만들지 않는다.
- Wheel Sorter는 기존 `binary_switch` 값을 Python에서 변경해서 방향을 전환한다.
- 현재는 테스트 목적으로 일정 simulation step마다 방향을 바꾼다.
- 나중에는 barcode / vision / mission 조건에 따라 A 또는 B 방향으로 보내도록 조건 부분만 교체한다.

## 변경 파일

```text
isaac_sim/main.py
isaac_sim/equipment/conveyor/conveyor_controller.py
isaac_sim/equipment/wheel_sorter/wheel_sorter_controller.py
docs/conveyor_sorter_control.md
README.md
```

## 사용 Stage 경로

Wheel Sorter binary switch:

```text
/World/ConveyorTrack/Sorter/ActionGraph/binary_switch.inputs:value
```

컨베이어는 Stage 안에서 이름이 `ConveyorBeltGraph`로 시작하고 `Velocity` 또는 `velocity` 속성을 가진 Graph를 찾아서 제어한다.

현재 월드에서 예시는 다음과 같다.

```text
/World/ConveyorTrack/ConveyorBeltGraph
/World/ConveyorTrack/ConveyorBeltGraph_01
/World/ConveyorTrack_01/ConveyorBeltGraph
```

## ConveyorController

파일:

```text
isaac_sim/equipment/conveyor/conveyor_controller.py
```

주요 기능:

```text
setup()            Stage에서 ConveyorBeltGraph 검색 후 초기 속도 적용
set_speed(speed)   모든 검색된 컨베이어 속도 변경
start()            설정된 속도로 시작
stop()             속도 0.0으로 정지
```

현재 `main.py`에서는 다음 값으로 생성한다.

```python
conveyor = ConveyorController(speed=1.0)
```

따라서 Isaac Sim 실행 시 컨베이어 Graph의 속도가 `1.0`으로 설정된다.

## WheelSorterController

파일:

```text
isaac_sim/equipment/wheel_sorter/wheel_sorter_controller.py
```

Wheel Sorter 방향은 기존 ActionGraph의 `binary_switch`를 사용한다.

```text
False = A 방향 테스트 상태
True  = B 방향 테스트 상태
```

현재 테스트 단계에서는 실제 좌/우 물리 방향과 A/B 라벨이 반대일 수 있다. 화면에서 한 번 확인한 뒤 필요하면 `route()`의 bool 매핑만 바꾸면 된다.

주요 기능:

```text
setup()              초기 상태 False
set_state(state)     binary_switch를 직접 True/False로 변경
update()             일정 simulation step마다 상태 전환
route("A" or "B")   나중에 실제 분류 조건에서 사용할 인터페이스
```

현재 `main.py`에서는 다음 값으로 생성한다.

```python
sorter = WheelSorterController(toggle_steps=120)
```

따라서 simulation loop가 120회 실행될 때마다:

```text
False -> True -> False -> True ...
```

순서로 전환한다.

`time.sleep()`, 별도 thread, asyncio timer는 사용하지 않는다. 기존 simulation loop에서 정수 counter 하나만 증가시키므로 테스트 제어의 추가 부담을 작게 유지한다.

## 실행

```bash
cd ~/collaboration/cobot3-ws-c2
git pull
./scripts/run_isaac.sh
```

별도로 Isaac Sim Extensions 창에서 Conveyor UI extension을 직접 켤 필요는 없다.

`main.py`에서 runtime에 필요한 extension을 자동으로 활성화한다.

```python
enable_extension("isaacsim.asset.gen.conveyor")
```

## 정상 동작 확인

Isaac 실행 로그에서 다음과 비슷한 내용을 확인한다.

```text
[CONVEYOR] /World/.../ConveyorBeltGraph speed=1.00
[SORTER] binary switch ready: ... toggle_steps=120
[SORTER] binary switch=True
[SORTER] binary switch=False
```

화면에서는 physics가 설정된 Cube를 컨베이어 위에 두고 Play 했을 때 다음을 확인한다.

1. Cube가 컨베이어 진행 방향으로 이동하는지 확인한다.
2. Wheel Sorter가 일정 간격으로 방향을 바꾸는지 확인한다.
3. `True`와 `False`가 실제 어느 방향인지 확인한다.

## 속도 변경

`isaac_sim/main.py`:

```python
conveyor = ConveyorController(speed=1.0)
```

예를 들어 속도를 `0.5`로 테스트하려면:

```python
conveyor = ConveyorController(speed=0.5)
```

## Wheel Sorter 전환 간격 변경

현재:

```python
sorter = WheelSorterController(toggle_steps=120)
```

더 자주 바꾸려면 숫자를 줄이고, 더 늦게 바꾸려면 숫자를 늘린다.

이 값은 wall-clock second가 아니라 simulation loop 횟수다.

## 실제 분류 조건 연결

현재 임시 테스트는:

```python
sorter.update()
```

가 simulation loop마다 호출되면서 자동으로 방향을 전환한다.

나중에 barcode / vision / mission 조건이 정해지면 자동 toggle은 제거하고 다음 인터페이스를 사용한다.

```python
sorter.route("A")
sorter.route("B")
```

예:

```python
if sorting_result == "A":
    sorter.route("A")
elif sorting_result == "B":
    sorter.route("B")
```

이렇게 하면 기존 USD ActionGraph와 Wheel Sorter 물리 구성은 그대로 유지하고 판단 조건만 교체할 수 있다.

## 주의사항

- 기존 `Sorter/ActionGraph`를 삭제하거나 다시 만들지 않는다.
- 기존 `binary_switch` Prim 경로를 임의로 변경하지 않는다.
- Conveyor asset의 기존 physics / graph 구성은 유지하고 Python에서는 속도 값만 제어한다.
- IW Hub LiDAR 및 NVIDIA Navigation 설정은 이 기능 구현과 무관하므로 변경하지 않는다.
