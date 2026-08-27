# AMR–협동로봇 연계 택배 분류 자동화 및 실시간 관제 시스템

Isaac Sim 5.1 + ROS 2 Jazzy 기반의 물류 자동화 통합 프로젝트입니다.

IW Hub AMR, P3020 협동로봇, Conveyor, Wheel Sorter, YOLO Vision, Nav2, MQTT, FastAPI, PostgreSQL, React Control Tower를 하나의 시나리오로 연결합니다.

> 이 README는 **새로운 PC에서 Git clone 후 프로젝트를 실행하는 사용자**를 기준으로 작성되어 있습니다.

---

## Contents

- [System Overview](#system-overview)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Run the Project](#run-the-project)
- [Open the Control Tower](#open-the-control-tower)
- [Check Status and Logs](#check-status-and-logs)
- [Stop the Project](#stop-the-project)
- [Environment Variables](#environment-variables)
- [Network Ports](#network-ports)
- [Manual Run](#manual-run)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)
- [Documentation](#documentation)

---

# System Overview

전체 물류 시나리오는 다음과 같습니다.

```text
IW Hub AMR Spawn
        ↓
Cargo Pod Pickup / Lift Up
        ↓
Nav2 Autonomous Navigation
        ↓
P3020 IN Pick & Place
        ↓
Main Conveyor
        ↓
Wheel Sorter A / B / C
        ↓
Region A / B / C or Exception
        ↓
P3020 OUT
        ↓
Control Tower Monitoring
```

Control Tower에서는 다음 정보를 실시간으로 확인합니다.

```text
AMR 위치 / 상태
P3020 작업 상태
Conveyor / Sorter 공정 상태
Package 흐름
P3020 IN YOLO 영상
P3020 OUT YOLO 영상
Warehouse Top View
AMR Camera
```

기본 ROS 2 통신 설정은 다음과 같습니다.

```text
ROS_DOMAIN_ID=110
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

# Requirements

## 1. Required platform software

아래 프로그램은 프로젝트를 clone하기 전에 PC에 설치되어 있어야 합니다.

| Component | Required | Default / Expected Path |
|---|---:|---|
| Ubuntu | 24.04 | - |
| NVIDIA Isaac Sim | 5.1.x | `~/isaacsim` |
| ROS 2 | Jazzy | `/opt/ros/jazzy` |
| Docker Engine | Required | `docker` command |
| Docker Compose Plugin | Required | `docker compose` command |
| Git | Required | `git` command |
| Node.js | 18+ | `node` command |
| npm | Required | `npm` command |
| Python | 3.12 | `python3` command |
| rosdep | Required | `rosdep` command |
| colcon | Required | `colcon` command |

Isaac Sim은 다음 파일이 존재해야 합니다.

```text
~/isaacsim/python.sh
```

ROS 2 Jazzy는 다음 파일이 존재해야 합니다.

```text
/opt/ros/jazzy/setup.bash
```

---

## 2. Isaac Sim ROS Jazzy workspace

이 프로젝트의 Nav2 실행에는 NVIDIA IW Hub navigation package가 포함된 별도의 ROS 2 Jazzy workspace가 필요합니다.

기본 경로는 다음과 같습니다.

```text
~/IsaacSim-ros_workspaces/jazzy_ws
```

최소한 다음 파일이 존재해야 합니다.

```text
~/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash
```

그리고 해당 workspace에는 프로젝트에서 사용하는 `iw_hub_navigation` package가 준비되어 있어야 합니다.

기본 경로가 아닌 곳에 설치했다면 아래 환경변수로 지정할 수 있습니다.

```bash
export ISAAC_ROS_WS=/your/path/to/jazzy_ws
```

---

## 3. Isaac Sim installation path

Isaac Sim을 `~/isaacsim`이 아닌 다른 위치에 설치했다면 다음 환경변수를 지정합니다.

```bash
export ISAAC_SIM_DIR=/your/path/to/isaacsim
```

예를 들어:

```bash
export ISAAC_SIM_DIR=$HOME/isaacsim
export ISAAC_ROS_WS=$HOME/IsaacSim-ros_workspaces/jazzy_ws
```

`~/.bashrc` 수정은 필요하지 않습니다.

새 터미널을 열었다면 필요한 환경변수를 다시 export하면 됩니다.

---

# Quick Start

새 PC에서 처음 실행하는 경우 아래 순서대로 진행하면 됩니다.

## 1. Clone repository

```bash
git clone https://github.com/rokey-c2/cobot3-ws-c2.git
cd cobot3-ws-c2
git switch main
```

최신 `main`을 사용하는 경우:

```bash
git pull --ff-only origin main
```

---

## 2. Optional: set custom Isaac paths

기본 경로를 사용하는 경우 이 단계는 생략합니다.

기본값:

```text
Isaac Sim           ~/isaacsim
Isaac ROS workspace ~/IsaacSim-ros_workspaces/jazzy_ws
```

다른 경로를 사용하는 경우:

```bash
export ISAAC_SIM_DIR=/your/path/to/isaacsim
export ISAAC_ROS_WS=/your/path/to/jazzy_ws
```

---

## 3. First-time project setup

새 PC에서는 최초 한 번 다음 명령을 실행하는 것을 권장합니다.

```bash
./scripts/quick_setup.sh --install-system-deps
```

이 스크립트는 프로젝트 실행에 필요한 작은 Ubuntu package와 프로젝트 dependency를 자동으로 준비합니다.

자동으로 처리하는 항목:

```text
python3.12-venv
python3-opencv
python3-rosdep
python3-colcon-common-extensions
Node.js / npm 확인
.env 생성
frontend/.env 생성
Frontend npm dependency 설치
Vision Python venv 생성
onnxruntime 설치
paho-mqtt 설치
ROS dependency 설치
ros2_ws colcon build
Docker image build
최종 환경 검사
```

이미 위 패키지가 설치되어 있다면 다음 명령만 사용해도 됩니다.

```bash
./scripts/quick_setup.sh
```

설치가 정상적으로 완료되면 마지막에 다음 안내가 출력됩니다.

```text
SETUP COMPLETE
```

---

## 4. Check environment

언제든 현재 PC가 프로젝트 실행 조건을 만족하는지 확인할 수 있습니다.

```bash
./scripts/check_environment.sh
```

정상적인 경우 마지막에 다음 메시지가 출력됩니다.

```text
Environment check completed without blocking errors.
```

`[FAIL]`이 하나라도 있다면 해당 항목을 먼저 해결한 뒤 실행하는 것을 권장합니다.

---

# Run the Project

## Recommended: clean first run

새 PC에서 최초 실행하거나 PostgreSQL/MQTT 데이터를 초기화하고 시작하려면:

```bash
./scripts/start_all.sh --fresh-db
```

`start_all.sh`가 다음 프로세스를 순서대로 자동 실행합니다.

```text
1. PostgreSQL + MQTT + FastAPI Docker containers
2. Isaac Sim Mission
3. /clock 확인
4. Front LiDAR 확인
5. Back LiDAR 확인
6. Chassis odometry 확인
7. Nav2 + RViz
8. Control Tower ROS2/MQTT adapters
9. P3020 PickPlace Action Server
10. Vision streams
11. React Frontend
12. Nav2 READY 확인
13. Pose Sync
14. /p3020/pick_place Action 확인
15. /navigate_to_pose Action 확인
```

프로젝트가 정상적으로 준비되면 다음 메시지가 출력됩니다.

```text
FULL STACK STARTED
```

---

## Start without deleting database

기존 PostgreSQL/MQTT 데이터를 유지하려면:

```bash
./scripts/start_all.sh
```

---

## Start the real AMR + P3020 mission

전체 시스템이 준비된 다음 별도로 실제 미션을 시작하려면:

```bash
./scripts/start_mission.sh
```

기본 미션 parameter는 다음과 같습니다.

```text
simulate_p3020=false
pickup_x=1.5
pickup_y=-2.0
place_x=-0.5
place_y=0.0
```

즉 기본 `start_mission.sh`는 P3020 simulation fallback을 사용하지 않고 실제 P3020 Action Server를 사용합니다.

---

## Start everything including the mission

환경 구성이 끝난 PC에서 전체 시스템과 실제 미션까지 한 번에 시작하려면:

```bash
./scripts/start_all.sh --fresh-db --mission
```

기존 DB를 유지하면서 미션까지 시작하려면:

```bash
./scripts/start_all.sh --mission
```

---

# Open the Control Tower

전체 시스템이 준비되면 브라우저에서 다음 주소를 엽니다.

```text
http://localhost:5173
```

Backend API:

```text
http://localhost:8000
```

영상 스트림을 직접 확인하려면:

```text
AMR Front Camera
http://localhost:8090/stream.mjpg

P3020 IN + YOLO Laser HUD
http://localhost:8091/stream.mjpg

Warehouse Top View
http://localhost:8092/stream.mjpg

P3020 OUT + YOLO Laser HUD
http://localhost:8093/stream.mjpg
```

---

# Check Status and Logs

## Process status

```bash
./scripts/status_all.sh
```

---

## Runtime logs

모든 자동 실행 프로세스의 로그는 다음 디렉터리에 저장됩니다.

```text
.runtime/logs/
```

주요 로그:

```bash
tail -f .runtime/logs/isaac.log
tail -f .runtime/logs/nav2.log
tail -f .runtime/logs/adapters.log
tail -f .runtime/logs/p3020_action.log
tail -f .runtime/logs/vision.log
tail -f .runtime/logs/frontend.log
tail -f .runtime/logs/pose_sync.log
tail -f .runtime/logs/mission.log
```

전체 로그 파일 확인:

```bash
ls -lah .runtime/logs/
```

---

# Stop the Project

## Stop all processes

Docker database volume은 유지하면서 전체 시스템을 종료합니다.

```bash
./scripts/stop_all.sh
```

---

## Stop and delete PostgreSQL / MQTT data

완전히 초기화하려면:

```bash
./scripts/stop_all.sh --volumes
```

다음 실행에서 새 DB로 시작하려는 경우 사용합니다.

---

# Environment Variables

프로젝트 기본값:

```bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

자동 실행 스크립트는 기본적으로 위 값을 사용합니다.

수동 실행 시에는 각 ROS 2 터미널에서 동일한 값을 사용하는 것이 중요합니다.

선택적으로 지정할 수 있는 경로:

```bash
export ISAAC_SIM_DIR=$HOME/isaacsim
export ISAAC_ROS_WS=$HOME/IsaacSim-ros_workspaces/jazzy_ws
```

FastDDS whitelist를 명시적으로 사용하려는 경우에만:

```bash
export USE_FASTDDS_WHITELIST=1
```

기본 실행에서는 기존 `FASTRTPS_DEFAULT_PROFILES_FILE`에 의한 통신 제한을 피하기 위해 whitelist 설정을 사용하지 않습니다.

---

# Network Ports

프로젝트에서 기본적으로 사용하는 포트입니다.

| Port | Service |
|---:|---|
| `5173` | React Frontend |
| `8000` | FastAPI Backend |
| `8090` | AMR Front Camera MJPEG |
| `8091` | P3020 IN YOLO MJPEG |
| `8092` | Warehouse Top View MJPEG |
| `8093` | P3020 OUT YOLO MJPEG |
| `5432` | PostgreSQL |
| `1883` | MQTT / Mosquitto |

포트가 이미 사용 중이면 해당 프로세스를 종료한 후 다시 실행합니다.

예:

```bash
ss -ltnp | grep -E '5173|8000|8090|8091|8092|8093|5432|1883'
```

---

# Manual Run

자동 실행에 문제가 있거나 개별 프로세스를 디버깅할 때만 사용하는 방법입니다.

모든 ROS 2 터미널에서 먼저:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`ISAAC_ROS_WS`를 기본 위치가 아닌 곳에 두었다면:

```bash
export ISAAC_ROS_WS=/your/path/to/jazzy_ws
```

`ISAAC_SIM_DIR`가 기본 위치가 아닌 경우:

```bash
export ISAAC_SIM_DIR=/your/path/to/isaacsim
```

권장 수동 실행 순서:

### Terminal 1 - Docker / Backend

```bash
cd cobot3-ws-c2
sudo docker compose up -d --build
sudo docker compose ps
```

### Terminal 2 - Isaac Sim

```bash
cd cobot3-ws-c2
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_isaac_mission.sh
```

Isaac Sim이 완전히 올라온 뒤 다음 단계를 실행합니다.

### Terminal 3 - Nav2 / RViz

```bash
cd cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
source ${ISAAC_ROS_WS:-$HOME/IsaacSim-ros_workspaces/jazzy_ws}/install/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_ros2.sh
```

### Terminal 4 - Pose Sync

```bash
cd cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_pose_sync.sh
```

### Terminal 5 - Control Tower adapters

```bash
cd cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_control_tower_adapters.sh
```

### Terminal 6 - P3020 Action Server

```bash
cd cobot3-ws-c2/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run arm_controller pick_place_action_server
```

### Terminal 7 - Vision

```bash
cd cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_vision_streams.sh
```

### Terminal 8 - Frontend

```bash
cd cobot3-ws-c2/frontend
npm run dev -- --host 0.0.0.0
```

### Terminal 9 - Mission

```bash
cd cobot3-ws-c2
./scripts/start_mission.sh
```

---

# Troubleshooting

## 1. `Isaac Sim python.sh not found`

예:

```text
[ERROR] Isaac Sim python.sh not found
```

Isaac Sim 경로를 확인합니다.

```bash
ls ~/isaacsim/python.sh
```

다른 위치에 설치했다면:

```bash
export ISAAC_SIM_DIR=/your/path/to/isaacsim
./scripts/quick_setup.sh
```

실행할 때도 같은 환경변수를 사용합니다.

```bash
export ISAAC_SIM_DIR=/your/path/to/isaacsim
./scripts/start_all.sh
```

---

## 2. `Isaac ROS Jazzy workspace not found`

다음 파일이 존재하는지 확인합니다.

```bash
ls ~/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash
```

다른 위치라면:

```bash
export ISAAC_ROS_WS=/your/path/to/jazzy_ws
```

해당 workspace에는 `iw_hub_navigation`이 준비되어 있어야 합니다.

---

## 3. `ModuleNotFoundError: onnxruntime`

시스템 Python에 직접 `pip install --user` 하지 않습니다.

프로젝트 venv를 다시 구성합니다.

```bash
./scripts/quick_setup.sh --install-system-deps
```

확인:

```bash
.venv/bin/python3 -c 'import onnxruntime; print(onnxruntime.__version__)'
```

---

## 4. `externally-managed-environment`

Ubuntu 24.04의 시스템 Python에 직접 pip 설치를 시도할 때 발생할 수 있습니다.

이 프로젝트는 `.venv`를 사용하므로 다음 스크립트를 사용합니다.

```bash
./scripts/quick_setup.sh --install-system-deps
```

---

## 5. Node.js version error

프로젝트는 Node.js 18 이상을 요구합니다.

확인:

```bash
node -v
npm -v
```

환경 검사:

```bash
./scripts/check_environment.sh
```

---

## 6. Docker permission denied

확인:

```bash
docker info
```

현재 사용자가 Docker daemon에 직접 접근할 수 없는 환경에서는 프로젝트 스크립트가 필요한 경우 `sudo docker compose`를 사용합니다.

수동으로 확인하려면:

```bash
sudo docker compose ps
```

---

## 7. ROS 2 node/topic이 서로 보이지 않음

모든 ROS 2 프로세스가 같은 domain을 사용해야 합니다.

```bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

확인:

```bash
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
```

필요한 기본 publisher 확인:

```bash
ros2 topic info /clock
ros2 topic info /front_2d_lidar/scan
ros2 topic info /back_2d_lidar/scan
ros2 topic info /chassis/odom
```

---

## 8. P3020 Action Server 확인

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 action list | grep p3020
```

정상적인 경우 다음 Action이 보여야 합니다.

```text
/p3020/pick_place
```

---

## 9. P3020이 실제 동작하지 않고 simulated 상태로 넘어감

실제 통합 미션은 다음 스크립트로 실행합니다.

```bash
./scripts/start_mission.sh
```

이 스크립트는 기본적으로 다음 parameter를 사용합니다.

```text
simulate_p3020=false
```

미션 node가 실행 중이라면 확인할 수 있습니다.

```bash
ros2 param get /amr_p3020_mission simulate_p3020
```

기대값:

```text
Boolean value is: False
```

---

## 10. Vision stream이 나오지 않음

ROS image topic 확인:

```bash
ros2 topic info /rgb
ros2 topic info /top_view/rgb
ros2 topic info /arm_b/rgb
```

Vision process log:

```bash
tail -f .runtime/logs/vision.log
```

HTTP health / stream 확인:

```bash
curl http://localhost:8091/health
curl http://localhost:8092/health
curl http://localhost:8093/health
```

---

## 11. Frontend가 열리지 않음

Frontend process 확인:

```bash
./scripts/status_all.sh
```

로그:

```bash
tail -f .runtime/logs/frontend.log
```

수동 실행:

```bash
cd frontend
npm run dev -- --host 0.0.0.0
```

브라우저:

```text
http://localhost:5173
```

---

## 12. 완전히 초기화해서 다시 실행

프로세스와 Docker volume을 정리합니다.

```bash
./scripts/stop_all.sh --volumes
```

환경을 다시 확인합니다.

```bash
./scripts/check_environment.sh
```

그리고 다시 시작합니다.

```bash
./scripts/start_all.sh --fresh-db
```

---

# Project Structure

```text
cobot3-ws-c2/
├── backend/                 # FastAPI + PostgreSQL + MQTT backend
├── docs/                    # Project documentation
├── frontend/                # React Control Tower
├── isaac_sim/               # Isaac Sim mission / robots / cameras / simulation
├── models/                  # YOLO ONNX model
├── mqtt/                    # Mosquitto configuration
├── requirements/            # Python dependency lists
├── ros2_ws/                 # ROS 2 Jazzy workspace
├── rviz/                    # RViz configuration
├── scripts/                 # Setup / run / stop / adapter scripts
├── tests/                   # Tests
├── compose.yaml             # PostgreSQL + MQTT + FastAPI
└── README.md
```

주요 실행 스크립트:

```text
scripts/quick_setup.sh       새 PC 최초 설정
scripts/check_environment.sh 환경 검사
scripts/start_all.sh         전체 시스템 시작
scripts/start_mission.sh     실제 AMR + P3020 미션 시작
scripts/status_all.sh        실행 상태 확인
scripts/stop_all.sh          전체 시스템 종료
```

---

# Documentation

상세 문서:

- [New PC Setup](docs/NEW_PC_SETUP.md)
- [Live Monitor](docs/LIVE_MONITOR.md)
- [AMR Pose Sync](docs/scenario/amr_pose_sync_complete.md)
- [Control Tower Live Integration](docs/scenario/control_tower_live_integration.md)
- [Conveyor / Sorter Control](docs/conveyor_sorter_control.md)

---

# Important Notes

- 프로젝트 기본 ROS domain은 `110`입니다.
- `~/.bashrc` 수정 없이 실행할 수 있도록 구성되어 있습니다.
- `isaac_sim/project_config/`를 사용합니다.
- `config/`라는 일반 Python package 이름은 Isaac Sim/OpenCV 내부 module과 충돌할 수 있으므로 프로젝트 설정 package 이름으로 사용하지 않습니다.
- 새 PC에서는 먼저 `./scripts/quick_setup.sh --install-system-deps`를 실행하는 것을 권장합니다.
- 전체 시스템이 정상적으로 올라오기 전에 미션만 먼저 실행하지 않는 것을 권장합니다.

---

# Minimal Commands

이미 Ubuntu / ROS 2 Jazzy / Isaac Sim 5.1 / Docker / Isaac ROS Jazzy workspace가 준비된 PC라면 실제로 필요한 핵심 명령은 아래와 같습니다.

```bash
# 1. Clone
git clone https://github.com/rokey-c2/cobot3-ws-c2.git
cd cobot3-ws-c2

# 2. First-time setup
./scripts/quick_setup.sh --install-system-deps

# 3. Start full stack
./scripts/start_all.sh --fresh-db

# 4. Start real mission
./scripts/start_mission.sh
```

또는 전체 시스템 + 미션을 한 번에:

```bash
./scripts/start_all.sh --fresh-db --mission
```

Control Tower:

```text
http://localhost:5173
```
