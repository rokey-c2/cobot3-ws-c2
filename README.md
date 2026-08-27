# AMR–협동로봇 연계 택배 분류 자동화 및 실시간 관제 시스템

Isaac Sim 5.1 + ROS 2 Jazzy 기반의 AMR / P3020 / Conveyor / Wheel Sorter / Control Tower 통합 프로젝트입니다.

현재 `main`의 기본 통신 설정은 다음과 같습니다.

```text
ROS_DOMAIN_ID=110
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 전체 흐름

```text
IW Hub AMR
→ Cargo Pod Pickup
→ Nav2 이동
→ P3020 IN Pick & Place
→ Main Conveyor
→ Wheel Sorter A/B/C
→ Region 또는 Exception
→ P3020 OUT
→ Control Tower 실시간 관제
```

관제 영상:

```text
8090  AMR Front Camera
8091  P3020 IN + YOLO Laser HUD
8092  Warehouse Top View
8093  P3020 OUT + YOLO Laser HUD
5173  React Frontend
8000  FastAPI Backend
```

---

# 새 PC에서 가장 빠른 실행 방법

## 0. PC에 미리 설치되어 있어야 하는 것

아래 대형 플랫폼은 Git 저장소에서 자동 설치하지 않습니다.

- Ubuntu 24.04
- NVIDIA Isaac Sim 5.1.0
- ROS 2 Jazzy
- Docker + Docker Compose plugin
- NVIDIA `iw_hub_navigation`이 들어 있는 Isaac Sim ROS Jazzy workspace

기본 경로:

```text
Isaac Sim:           ~/isaacsim
Isaac ROS workspace: ~/IsaacSim-ros_workspaces/jazzy_ws
```

다른 위치에 설치했다면 `ISAAC_SIM_DIR`, `ISAAC_ROS_WS` 환경변수로 지정할 수 있습니다.

## 1. Clone

```bash
git clone https://github.com/rokey-c2/cobot3-ws-c2.git
cd cobot3-ws-c2
git switch main
```

## 2. 최초 1회 자동 설정

Ubuntu helper package까지 같이 설치하려면:

```bash
./scripts/quick_setup.sh --install-system-deps
```

이미 `python3.12-venv`, OpenCV, rosdep, colcon, Node/npm이 준비되어 있으면:

```bash
./scripts/quick_setup.sh
```

이 스크립트가 자동으로 처리하는 것:

```text
.env / frontend/.env 생성
Frontend npm dependency 설치
Vision onnxruntime venv 구성
ROS2 MQTT adapter dependency 설치
rosdep dependency 설치
ros2_ws colcon build
Docker image build
환경 최종 검사
```

`~/.bashrc`는 수정하지 않습니다.

## 3. 전체 시스템 시작

기존 DB 데이터를 유지하면서 시작:

```bash
./scripts/start_all.sh
```

PostgreSQL/MQTT 데이터를 초기화하고 새로 시작:

```bash
./scripts/start_all.sh --fresh-db
```

`start_all.sh`가 자동으로 순서를 관리합니다.

```text
Docker PostgreSQL/MQTT/FastAPI
→ Isaac Sim
→ /clock + LiDAR + odom 확인
→ Nav2 / RViz
→ Control Tower Adapter
→ P3020 Action Server
→ AMR/Top/P3020 Vision
→ Frontend
→ Nav2 READY 확인
→ Pose Sync
```

각 프로세스는 `.runtime/logs/` 아래에 로그를 남깁니다.

## 4. 실제 미션 시작

전체 시스템이 올라온 뒤:

```bash
./scripts/start_mission.sh
```

이 스크립트는 실제 P3020을 사용하도록 기본적으로 다음 옵션을 사용합니다.

```text
simulate_p3020:=false
pickup_x:=1.5
pickup_y:=-2.0
place_x:=-0.5
place_y:=0.0
```

전체 시스템과 실제 미션까지 한 번에 시작하려면:

```bash
./scripts/start_all.sh --fresh-db --mission
```

---

# 상태 확인 / 종료

상태 확인:

```bash
./scripts/status_all.sh
```

로그 확인 예:

```bash
tail -f .runtime/logs/isaac.log
tail -f .runtime/logs/nav2.log
tail -f .runtime/logs/vision.log
tail -f .runtime/logs/mission.log
```

전체 종료, Docker 데이터 유지:

```bash
./scripts/stop_all.sh
```

전체 종료 + PostgreSQL/MQTT volume 삭제:

```bash
./scripts/stop_all.sh --volumes
```

---

# 수동 실행이 필요한 경우

자동 실행 대신 직접 터미널을 나눠 실행할 수도 있습니다. 모든 터미널에서 다음 값을 사용합니다.

```bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

순서:

```text
1. sudo docker compose up -d --build
2. ./scripts/run_isaac_mission.sh
3. ./scripts/run_ros2.sh
4. ./scripts/run_pose_sync.sh
5. ./scripts/run_control_tower_adapters.sh
6. ros2 run arm_controller pick_place_action_server
7. ./scripts/run_vision_streams.sh
8. cd frontend && npm run dev
9. ./scripts/start_mission.sh
```

P3020 Action Server를 직접 실행할 때:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run arm_controller pick_place_action_server
```

---

# 주요 문서

- 새 PC 설치 상세: [`docs/NEW_PC_SETUP.md`](docs/NEW_PC_SETUP.md)
- Live Monitor: [`docs/LIVE_MONITOR.md`](docs/LIVE_MONITOR.md)
- AMR Pose Sync: [`docs/scenario/amr_pose_sync_complete.md`](docs/scenario/amr_pose_sync_complete.md)
- Control Tower 연동: [`docs/scenario/control_tower_live_integration.md`](docs/scenario/control_tower_live_integration.md)
- Conveyor / Sorter: [`docs/conveyor_sorter_control.md`](docs/conveyor_sorter_control.md)

## 중요

`isaac_sim/project_config/`를 사용합니다.

`config/`라는 일반 이름은 Isaac Sim/OpenCV 내부 모듈과 충돌할 수 있으므로 사용하지 않습니다.
