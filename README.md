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
- ROS_DOMAIN_ID=111
- RMW_IMPLEMENTATION=rmw_fastrtps_cpp
- Nav2
- RTX LiDAR
- PhysX
- Python
- Docker + Docker Compose plugin (관제타워 PostgreSQL/Mosquitto/FastAPI)
- Node.js 18+ / npm 9+ (관제타워 프론트엔드)

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

Control Tower의 P3020·Conveyor·Sorter·Mission·Package 실시간 연동 및
Nav2 없이 실행하는 방법은
[Control Tower Live Process Integration](docs/scenario/control_tower_live_integration.md)을 참고합니다.

## 최초 1회 설정

```bash
cp .env.example .env
./scripts/setup_all.sh
./scripts/check_environment.sh
```

`setup_all.sh`가 프론트엔드 npm 패키지, ROS2 rosdep 의존성, 비전(YOLO) venv,
관제타워 어댑터 venv(`paho-mqtt`)를 모두 설치합니다. 자세한 내용은
[docs/NEW_PC_SETUP.md](docs/NEW_PC_SETUP.md)를 참고합니다.

## 실행 (전체 통합 미션: AMR + P3020 Pick&Place + 관제타워)

저장소를 클론한 경로에서, 터미널 9개로 아래 순서대로 실행합니다.
(1이 완전히 뜬 다음 2, 2가 "Nav2 is ready." 뜬 다음 3. 4는 아무 때나 먼저 띄워도 되지만
5는 4가 뜬 뒤여야 합니다. 6·7·8은 순서 상관없이. 9는 1·2·3·5·6·7이 모두 뜬 마지막에.)

터미널 1 — Isaac Sim (맵 + AMR + P3020 미션 에이전트):

```bash
export ROS_DOMAIN_ID=111
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_isaac_mission.sh
```

터미널 2 — Nav2 (터미널 1의 센서 토픽이 뜬 뒤 자동으로 기다렸다가 시작됨):

```bash
export ROS_DOMAIN_ID=111
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_ros2.sh
```

터미널 3 — Pose Sync (AMCL을 Isaac의 실제 좌표로 초기화):

```bash
export ROS_DOMAIN_ID=111
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_pose_sync.sh
```

터미널 4 — 관제타워 DB/MQTT/백엔드 (Docker):

```bash
sudo docker compose up -d --build
sudo docker compose ps
```

터미널 5 — 관제타워 AMR + Process 어댑터 (Nav2 없이도 P3020/컨베이어/소터/미션 추적은 동작):

```bash
export ROS_DOMAIN_ID=111
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_control_tower_adapters.sh
```

터미널 6 — ROS2 빌드 + P3020 액션 서버:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=111 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run arm_controller pick_place_action_server
```

터미널 7 — 박스 인식(YOLO):

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=111 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 isaac_sim/robots/p3020/vision/box_detector_node.py
```

터미널 8 — 관제타워 프론트엔드:

```bash
cd frontend
npm ci
npm run dev
```

브라우저에서 `http://localhost:5173`을 엽니다.

터미널 9 — 전체 미션 트리거:

```bash
export ROS_DOMAIN_ID=111
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_amr_p3020_mission.sh --ros-args -p simulate_p3020:=false -p pickup_x:=1.5 -p pickup_y:=-2.0 -p place_x:=-0.5 -p place_y:=0.0
```

> `simulate_p3020` 파라미터를 생략하면 기본값 `true`로 동작해서, 터미널 6·7이 떠 있어도
> 실제 P3020 액션 서버에 요청을 보내지 않고 결과만 흉내 냅니다 (AMR 로직만 먼저 테스트할 때 사용).
> 실제 로봇팔까지 동작을 확인하려면 반드시 `simulate_p3020:=false`를 명시해야 합니다.

> 각 터미널을 새로 열 때마다 `echo $ROS_DOMAIN_ID`로 111이 맞는지 확인합니다.
> `.bashrc` 등에 다른 기본값이 설정돼 있으면 터미널끼리 서로 통신이 안 되는 문제가 생길 수 있습니다.

> Docker 볼륨이 이미 있는 상태에서 `.env`를 새로 만들거나 바꿨다면, Postgres는
> 빈 볼륨일 때만 계정/DB를 초기화하므로 `sudo docker compose down -v` 후
> `sudo docker compose up -d --build`로 볼륨째 다시 만들어야 반영됩니다.

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
