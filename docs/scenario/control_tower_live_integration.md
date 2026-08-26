# Control Tower Live Process Integration

## 완료 범위

`euiseok-fix`의 기존 AMR 제어와 P3020 1회 Pick & Place 동작을 유지하면서
다음 경로를 연결한다.

```text
Isaac AMR / P3020 / Conveyor / Sorter
  -> ROS2 process topics
  -> scripts/process_mqtt_adapter.py
  -> MQTT controltower/process/event
  -> FastAPI process event handler
  -> PostgreSQL mission / mission_stage / package / package_event
  -> React Dashboard / Packages
```

- P3020 `SCANNING`부터 `DONE_SUCCESS`/`DONE_FAIL`까지 Mission에 반영
- P3020 성공 후 실제 Parcel destination 속성으로 Wheel Sorter 경로 설정
- 기본 Parcel은 `parcel_box_01` 1개만 생성하며 목적지는 `A`로 설정
- 컨베이어·소터·P3020 START/STOP을 Dashboard System 제어에 포함
- Package 현재 Zone 및 Event History 자동 갱신
- Region A/B/C에 따라 실제 통과하는 Sorter Stage를 동적으로 생성
- Process Event `event_key`로 중복 반영 방지
- 기존 PostgreSQL volume에도 Backend 시작 시 Migration 자동 적용

## 실행 순서

Nav2를 실행할 수 없는 PC에서도 P3020, Lift, Frontend 관제는 실행할 수 있다.
AMR Navigate만 `./scripts/run_ros2.sh`가 추가로 필요하다.

### Isaac PC

```bash
cd ~/collaboration/cobot3-ws-c2
git switch euiseok-fix
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_isaac_mission.sh
```

### ROS / Control Tower PC - Docker

```bash
cd ~/collaboration/cobot3-ws-c2
sudo docker compose up -d --build
sudo docker compose ps
```

### ROS / Control Tower PC - AMR + Process Adapter

```bash
cd ~/collaboration/cobot3-ws-c2
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_control_tower_adapters.sh
```

이 스크립트는 `ros2_mqtt_adapter.py`와 `process_mqtt_adapter.py`를 함께
실행한다. Nav2 Launch를 실행하지 않아도 Lift와 공정 상태 연동은 동작한다.
Navigate 버튼만 Nav2 Action Server가 없으면 실패한다.

### YOLO

```bash
cd ~/collaboration/cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 isaac_sim/robots/p3020/vision/box_detector_node.py
```

### Frontend

```bash
cd ~/collaboration/cobot3-ws-c2/frontend
npm run dev
```

브라우저에서 `http://localhost:5173`을 연다.

## 테스트

1. Dashboard의 Live Tracking에서 Package ID와 Region을 선택해 Mission 생성
   - 기본 테스트의 `parcel_box_01`은 Region A이므로 Region A를 선택
2. System Start를 눌러 장비 상태가 `RUNNING`으로 바뀌는지 확인
3. P3020 앞에 박스를 배치하고 `/arm_a/pick_place_command` 전송
4. Dashboard Mission Stage가 다음과 같이 이동하는지 확인

```text
MANIPULATOR_PICK -> MANIPULATOR_PLACE -> MAIN_CONVEYOR
-> SORTER_<REGION> -> COMPLETE
```

5. Packages 화면에서 현재 Zone과 Event History 확인

Region B/C Package는 앞쪽 Sorter를 통과하므로 Mission Stage에도
`SORTER_A -> SORTER_B` 또는 `SORTER_A -> SORTER_B -> SORTER_C`가 표시된다.

## 주요 ROS2 Topic

| Topic | 방향 | 용도 |
|---|---|---|
| `/arm_a/pick_place_status` | Isaac -> Adapter | P3020 단계 |
| `/amr_a/pickup_state` | Isaac -> Adapter | AMR Cargo 단계 |
| `/controltower/equipment/command` | Adapter -> Isaac | P3020/Conveyor/Sorter START/STOP |
| `/controltower/equipment/status` | Isaac -> Adapter | 실제 장비 상태 피드백 |
| `/controltower/process/event` | Isaac -> Adapter | Conveyor/Sorter Package 이벤트 |

## 제한 사항

- Sorter 도착은 현재 물리 센서가 아니라 기존 저자원 시뮬레이션 시간 조건
  (`ROUTE_COMPLETE_SECONDS=6.0`)으로 확정한다.
- AMR Navigate는 기존과 동일하게 Nav2가 필요하다.
- 한 P3020 명령은 한 박스만 처리하고 종료한다.
