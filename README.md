# AMR–협동로봇 연계 택배 분류 자동화 및 실시간 관제 시스템

## MVP Flow
Input Zone
→ IW Hub AMR
→ Doosan P3020
→ Main Conveyor
→ Wheel Sorter
→ A/B Conveyor
→ P3020 A/B 적재
→ Box Full
→ 출고 IW Hub
→ 배송지

## 개발 환경
- Isaac Sim 5.1.0
- ROS 2 Jazzy
- ROS_DOMAIN_ID=110
- RMW_IMPLEMENTATION=rmw_fastrtps_cpp
- Nav2
- RTX LiDAR
- PhysX
- Python

## IW Hub 단일 자율주행 테스트

현재 `feat/iw-hub-single-navigation` 브랜치는 프로젝트 Warehouse USD에
NVIDIA Isaac Sim 5.1의 공식 `iw_hub_warehouse_navigation.usd` 안에 들어 있는
IW Hub Nav2 로봇 구성을 reference해서 사용합니다.

프로젝트 코드에서 LiDAR를 새로 만들거나 센서 위치, 방향, range를 변경하지 않습니다.
NVIDIA Navigation 샘플의 front/back 2D LiDAR와 ROS 2 graph를 그대로 사용합니다.

기본 ROS 2 토픽:

```text
/cmd_vel
/chassis/odom
/front_2d_lidar/scan
/back_2d_lidar/scan
```

출발 위치:

```text
x = 8.155903816223145
y = -5.628969192504883
```

목표 위치:

```text
x = -13.813100814819336
y = 0.7454315423965454
```

추가 테스트 장애물과 cargo는 현재 단일 주행 검증에서 생성하지 않습니다.

## Conveyor / Wheel Sorter

메인 컨베이어는 Python에서 속도 `1.0`으로 제어합니다.
Wheel Sorter는 기존 ActionGraph의 `binary_switch` 값을 Python에서 변경해서 방향을 제어합니다.

현재 테스트 단계에서는 Wheel Sorter가 일정 simulation step마다 자동으로 방향을 바꿉니다.
나중에는 barcode / vision / mission 조건에 맞춰 `A` 또는 `B` 방향으로 보내도록 조건만 교체할 수 있습니다.

상세 내용은 [docs/conveyor_sorter_control.md](docs/conveyor_sorter_control.md)를 참고합니다.

## 실행 (전체 통합 미션: AMR + P3020 Pick&Place)

저장소를 클론한 경로에서, 터미널 5개로 순서대로 실행합니다.
(터미널 1이 완전히 뜬 다음 2를 실행하고, 3·4는 순서 상관없이, 마지막에 5)

터미널 1 — Isaac Sim (맵 + AMR + P3020 미션 에이전트):

```bash
./scripts/run_isaac_mission.sh
```

터미널 2 — Nav2 (터미널 1의 센서 토픽이 뜬 뒤 자동으로 기다렸다가 시작됨):

```bash
./scripts/run_ros2.sh
```

터미널 3 — ROS2 빌드 + P3020 액션 서버:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=110 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run arm_controller pick_place_action_server

(venv)
cd ~/collaboration/cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

터미널 4 — 박스 인식(YOLO):

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 isaac_sim/robots/p3020/vision/box_detector_node.py
```

onnxruntime이 시스템 python3에 없다면 먼저 `./scripts/setup_vision_env.sh`로 venv를 만들고
`source .venv/bin/activate` 한 뒤 실행합니다.

터미널 5 — 전체 미션 트리거:

```bash
./scripts/run_amr_p3020_mission.sh --ros-args -p simulate_p3020:=false -p pickup_x:=1.5 -p pickup_y:=-2.0 -p place_x:=-0.5 -p place_y:=0.0
```

> `simulate_p3020` 파라미터를 생략하면 기본값 `true`로 동작해서, 터미널 3·4가 떠 있어도
> 실제 P3020 액션 서버에 요청을 보내지 않고 결과만 흉내 냅니다 (AMR 로직만 먼저 테스트할 때 사용).
> 실제 로봇팔까지 동작을 확인하려면 반드시 `simulate_p3020:=false`를 명시해야 합니다.

> 각 터미널을 새로 열 때마다 `echo $ROS_DOMAIN_ID`로 110이 맞는지 확인합니다.
> `.bashrc` 등에 다른 기본값이 설정돼 있으면 터미널끼리 서로 통신이 안 되는 문제가 생길 수 있습니다.

## 실행 (IW Hub 단일 자율주행 테스트)

> **주의**: 이 경로는 현재 이 저장소만으로는 끝까지 실행되지 않습니다. 아래 두 터미널로
> 시뮬레이션과 Nav2/AMCL은 뜨지만, 실제 주행 목표(goal)를 보내는 스크립트가 이 저장소에
> 없습니다 (`isaac_sim/main.py`는 목표 좌표를 화면에 출력만 하고 Nav2에 goal을 보내지
> 않습니다). goal을 직접 보내려면 `ros2 action send_goal /navigate_to_pose ...`를 수동으로
> 실행하거나, 별도 트리거 스크립트를 새로 작성해야 합니다.
> 실제로 끝까지 검증된 경로는 위 "전체 통합 미션" 절차입니다.

터미널 1:

```bash
./scripts/run_isaac.sh
```

터미널 2:

```bash
./scripts/run_ros2.sh
```

IW Hub 상세 내용은 [docs/iw_hub_navigation.md](docs/iw_hub_navigation.md)를 참고합니다.

## 중요
`isaac_sim/project_config/`를 사용합니다.

`config/`라는 일반 이름은 Isaac Sim/OpenCV 내부 모듈과 충돌할 수 있으므로 사용하지 않습니다.
