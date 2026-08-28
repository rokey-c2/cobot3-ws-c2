# AMR–협동로봇 연계 택배 분류 자동화 및 실시간 관제 시스템

Isaac Sim 5.1 + ROS 2 Jazzy 기반의 물류 자동화 통합 프로젝트입니다.

IW Hub AMR, Doosan P3020, VGP20 gripper, RGB/Depth camera, Conveyor, Wheel Sorter, YOLO ONNX vision, Nav2, MQTT, FastAPI, PostgreSQL, React Control Tower를 하나의 시나리오로 연결합니다.

> 본 저장소의 `main`은 Isaac Sim 시뮬레이션 기반 제출/시연 버전입니다. 아래 설치 및 실행 순서는 `ROS_DOMAIN_ID=110`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 기준입니다.

---

## 목차

1. [시스템 설계](#1-시스템-설계)
2. [전체 Flow Chart](#2-전체-flow-chart)
3. [운영체제 및 개발 환경](#3-운영체제-및-개발-환경)
4. [사용 장비 및 시뮬레이션 구성](#4-사용-장비-및-시뮬레이션-구성)
5. [주요 시뮬레이션 Asset / 실행 파일](#5-주요-시뮬레이션-asset--실행-파일)
6. [로봇 및 장비 제어 Python 코드](#6-로봇-및-장비-제어-python-코드)
7. [ROS2 패키지](#7-ros2-패키지)
8. [Dependencies / 설치 방법](#8-dependencies--설치-방법)
9. [실행 순서](#9-실행-순서)
10. [제출용 ZIP 생성 전 정리](#10-제출용-zip-생성-전-정리)

---

# 1. 시스템 설계

## System Architecture

```mermaid
flowchart LR
    SIM[Isaac Sim 5.1\nWarehouse / IW Hub / P3020 / Conveyor] --> BRIDGE[Isaac ROS2 Bridge\nROS2 Jazzy]

    BRIDGE --> AMR[IW Hub AMR\nLiDAR / Odom / Lift]
    BRIDGE --> ARM[P3020 IN / OUT\nRGB-Depth Camera / VGP20]
    BRIDGE --> PROC[Conveyor / Wheel Sorter]

    AMR --> NAV[Nav2 / AMCL]
    ARM --> VISION[YOLO ONNX\nBox Detection]

    NAV --> MISSION[AMR-P3020 Mission]
    VISION --> MISSION
    PROC --> MISSION

    MISSION --> ADAPTER[ROS2-MQTT Adapter]
    ADAPTER --> MQTT[Mosquitto MQTT]
    MQTT --> API[FastAPI Backend]
    API --> DB[(PostgreSQL)]
    API --> WEB[React Control Tower]

    ARM --> STREAM[MJPEG Vision Stream\n8091 / 8093]
    SIM --> STREAM2[AMR / Top View Stream\n8090 / 8092]
    STREAM --> WEB
    STREAM2 --> WEB
```

### 데이터 흐름

- **로봇/공정 상태**: Isaac Sim → ROS2 → MQTT Adapter → Mosquitto → FastAPI → PostgreSQL / React
- **AMR 자율주행**: Isaac Sim LiDAR/Odom → ROS2 → Nav2/AMCL → AMR velocity command
- **P3020 작업**: Camera → YOLO ONNX → box pixel → P3020 Pick & Place → Action result
- **실시간 영상**: ROS Image → MJPEG stream → React Control Tower

---

# 2. 전체 Flow Chart

```mermaid
flowchart TD
    A[System Start] --> B[Docker\nPostgreSQL / MQTT / FastAPI]
    B --> C[Isaac Sim Warehouse Load]
    C --> D[IW Hub / P3020 / Conveyor / Sorter Ready]
    D --> E[Nav2 + AMCL Ready]
    E --> F[Pose Sync]
    F --> G[IW Hub Cargo 접근]
    G --> H[Lift Up]
    H --> I[Nav2로 P3020 IN 이동]
    I --> J[P3020 IN Box Detection]
    J --> K[P3020 Pick & Place]
    K --> L[Main Conveyor]
    L --> M[Wheel Sorter 분류]
    M --> N[P3020 OUT 처리]
    N --> O[Control Tower 상태 반영]
    O --> P[IW Hub Cargo 복귀]
    P --> Q[정밀 도킹]
    Q --> R[Lift Down]
    R --> S[IW Hub Spawn 복귀]
    S --> T[Mission Complete]
```

---

# 3. 운영체제 및 개발 환경

| 항목 | 버전 / 설정 |
|---|---|
| OS | Ubuntu 24.04 |
| NVIDIA Isaac Sim | 5.1.0 |
| ROS 2 | Jazzy |
| Python | 3.12 |
| ROS Domain | `ROS_DOMAIN_ID=110` |
| RMW | `rmw_fastrtps_cpp` |
| Navigation | Nav2 / AMCL |
| Simulation | USD / PhysX / OmniGraph |
| Vision | OpenCV / ONNX Runtime / YOLO model |
| Backend | FastAPI |
| Message Broker | Eclipse Mosquitto MQTT |
| Database | PostgreSQL 16 |
| Frontend | React 18 / Vite 5 |
| Container | Docker + Docker Compose plugin |
| Node.js | 18+ |
| npm | 9+ |

Isaac Sim 실행 PC는 NVIDIA RTX GPU가 필요합니다.

프로젝트 기본 ROS 통신 설정:

```bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

> 일부 개별 스크립트에는 과거 기본값 `111`이 남아 있을 수 있으므로, 실행 시 위 두 환경변수를 각 터미널에서 명시적으로 export하는 것을 권장합니다.

---

# 4. 사용 장비 및 시뮬레이션 구성

본 제출본은 **Isaac Sim 기반 시뮬레이션**으로 구성됩니다.

| 장비 / 구성 | 역할 |
|---|---|
| NVIDIA IW Hub AMR | Cargo Pod 운반 및 Nav2 자율주행 |
| Front / Back 2D LiDAR | AMR 장애물 인식 및 Nav2 costmap 입력 |
| Doosan P3020 | Parcel Pick & Place |
| VGP20 Gripper | 박스 흡착 / 해제 |
| RSD455 RGB/Depth Camera | P3020 박스 위치 인식 |
| Main Conveyor | Parcel 이송 |
| Wheel Sorter | Parcel 지역 분류 |
| Cargo Pod / Parcel | 물류 적재 및 분류 대상 |
| Control Tower PC | 관제 UI / Backend / DB / MQTT |

---

# 5. 주요 시뮬레이션 Asset / 실행 파일

## 메인 시뮬레이션 USD

최종 Warehouse 메인 USD:

```text
isaac_sim/usd/Final_Real_Map/Parcel_Sorting_Map.usd
```

`isaac_sim/main_mission.py`가 위 USD를 메인 Warehouse stage로 사용합니다.

Nav2 map:

```text
isaac_sim/usd/Final_Real_Map/navigation/maps/Final.yaml
```

## 메인 Isaac 실행 Python

```text
isaac_sim/main_mission.py
isaac_sim/main_mission_live_view.py
```

실행 스크립트:

```text
scripts/run_isaac_mission.sh
```

`run_isaac_mission.sh`는 Control Tower용 live view wrapper인 `main_mission_live_view.py`를 실행하고, 내부적으로 기존 mission logic을 사용합니다.

## P3020 Asset / URDF

```text
isaac_sim/assets/p3020/p3020.urdf
isaac_sim/assets/p3020/p3020_description.yaml
isaac_sim/assets/p3020/P3020_mount_vgp20/
isaac_sim/assets/p3020/P3020_mount_vgp20_rsd455_1/
isaac_sim/assets/p3020/meshes/
isaac_sim/assets/vgp20/
```

그 외 Warehouse / conveyor / wheel sorter / parcel 관련 asset은 다음 위치에 있습니다.

```text
isaac_sim/assets/
isaac_sim/usd/
```

> 제출 ZIP에는 `usd`, `usda`, `urdf`, mesh 및 이들이 참조하는 asset 파일을 함께 포함해야 합니다.

---

# 6. 로봇 및 장비 제어 Python 코드

과제의 “서보모터/로봇 제어 Python 코드”에 해당하는 핵심 파일입니다.

## P3020 IN Pick & Place / Joint / Gripper 제어

```text
isaac_sim/robots/p3020/p3020_mission_agent.py
```

- P3020 관절 동작
- VGP20 grasp / release
- RGB/Depth 기반 박스 접근
- Pick & Place 상태 관리

## P3020 OUT 제어

```text
isaac_sim/robots/p3020/p3020_out_mission_agent.py
```

## ROS2 P3020 Action Server

```text
ros2_ws/src/arm_controller/arm_controller/pick_place_action_server.py
```

Action:

```text
/p3020/pick_place
```

## IW Hub AMR Mission 제어

```text
isaac_sim/robots/iw_hub/iw_hub_mission_agent.py
scripts/run_amr_p3020_mission.sh
```

## Conveyor / Wheel Sorter 제어

```text
isaac_sim/equipment/conveyor/conveyor_controller.py
isaac_sim/equipment/wheel_sorter/wheel_sorter_controller.py
```

## Vision

```text
isaac_sim/robots/p3020/vision/box_detector_node.py
isaac_sim/robots/p3020/vision/object_detector.py
models/parcel_box_yolo_model/best.onnx
```

---

# 7. ROS2 패키지

프로젝트 자체 ROS2 workspace:

```text
ros2_ws/src/
├── amr_controller
├── arm_controller
├── logistics_bringup
├── logistics_interfaces
├── mission_manager
├── sorter_controller
└── vision_node
```

외부 NVIDIA Isaac Sim ROS Jazzy workspace에서 사용하는 패키지:

```text
iw_hub_navigation
```

기본 외부 workspace 경로:

```text
~/IsaacSim-ros_workspaces/jazzy_ws
```

다른 경로를 사용하면 해당 workspace의 `install/setup.bash`를 source한 뒤 실행합니다.

---

# 8. Dependencies / 설치 방법

## 8.1 사전에 설치되어 있어야 하는 프로그램

- Ubuntu 24.04
- NVIDIA Isaac Sim 5.1.0
- ROS 2 Jazzy (`/opt/ros/jazzy`)
- Docker Engine
- Docker Compose plugin (`docker compose`)
- NVIDIA Isaac Sim ROS Jazzy workspace + `iw_hub_navigation`
- Git
- Python 3.12
- Node.js 18+
- npm 9+

Ubuntu helper package:

```bash
sudo apt update
sudo apt install -y \
  python3.12-venv \
  python3-opencv \
  python3-rosdep \
  python3-colcon-common-extensions \
  nodejs \
  npm
```

## 8.2 Clone

```bash
git clone https://github.com/rokey-c2/cobot3-ws-c2.git
cd cobot3-ws-c2
git switch main
```

## 8.3 환경 파일

```bash
cp .env.example .env
cp frontend/.env.example frontend/.env
```

## 8.4 Python / Frontend / ROS dependency 설치

전체 dependency helper:

```bash
./scripts/setup_all.sh
```

Host Python dependency entry point:

```text
requirements.txt
```

세부 파일:

```text
requirements/vision.txt
requirements/ros2-adapter.txt
backend/requirements.txt
frontend/package.json
frontend/package-lock.json
```

현재 중요한 호환성 조건:

```text
paho-mqtt >= 2.0, < 3
onnxruntime >= 1.18, < 2
```

`pose_sync_manager.py`와 ROS2-MQTT adapter는 `paho-mqtt 2.x`의 `CallbackAPIVersion.VERSION2`를 사용합니다.

## 8.5 ROS2 workspace build

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
cd ..
```

환경 검사:

```bash
./scripts/check_environment.sh
```

---

# 9. 실행 순서

아래 명령은 저장소 root에서 실행하는 것을 기준으로 합니다.

## Terminal 1 — Docker DB / MQTT / Backend

처음 실행하거나 DB를 초기화할 때:

```bash
cd ~/collaboration/cobot3-ws-c2/cobot3-ws-c2
sudo docker compose down -v --remove-orphans
sudo docker compose up -d --build
sudo docker compose ps
```

`-v`는 PostgreSQL / MQTT volume 데이터를 삭제합니다.

DB를 유지할 때는:

```bash
sudo docker compose up -d --build
```

## Terminal 2 — Isaac Sim

```bash
cd ~/collaboration/cobot3-ws-c2/cobot3-ws-c2
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_isaac_mission.sh
```

Isaac Sim이 완전히 올라오고 다음 sensor publisher가 생성된 뒤 Terminal 3을 실행합니다.

```text
/clock
/front_2d_lidar/scan
/back_2d_lidar/scan
/chassis/odom
```

## Terminal 3 — ROS2 Nav2 / RViz

```bash
cd ~/collaboration/cobot3-ws-c2/cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_ros2.sh
```

다음 로그 확인:

```text
[ROS2] Nav2 is ready.
```

## Terminal 4 — Pose Sync

```bash
cd ~/collaboration/cobot3-ws-c2/cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_pose_sync.sh
```

## Terminal 5 — Control Tower ROS2 / MQTT Adapter

```bash
cd ~/collaboration/cobot3-ws-c2/cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_control_tower_adapters.sh
```

## Terminal 6 — P3020 Action Server

```bash
cd ~/collaboration/cobot3-ws-c2/cobot3-ws-c2/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run arm_controller pick_place_action_server
```

## Terminal 7 — Vision / YOLO / MJPEG

```bash
cd ~/collaboration/cobot3-ws-c2/cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_vision_streams.sh
```

기본 stream:

| Port | Stream |
|---:|---|
| 8090 | AMR Front Camera |
| 8091 | P3020 IN + YOLO |
| 8092 | Warehouse Top View |
| 8093 | P3020 OUT + YOLO |

## Terminal 8 — React Control Tower

```bash
cd ~/collaboration/cobot3-ws-c2/cobot3-ws-c2/frontend
npm ci
npm run dev
```

Browser:

```text
http://localhost:5173
```

FastAPI:

```text
http://localhost:8000
```

## Terminal 9 — 실행 전 확인

```bash
cd ~/collaboration/cobot3-ws-c2/cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 action list | grep p3020
ros2 action list | grep navigate_to_pose
ros2 topic info /clock
ros2 topic info /chassis/odom
ros2 topic info /rgb
ros2 topic info /box_pixel
ros2 topic info /top_view/rgb
ros2 topic info /arm_b/rgb
ros2 topic info /arm_b/box_pixel
```

정상 Action 예:

```text
/p3020/pick_place
/navigate_to_pose
```

## Terminal 10 — 전체 AMR + P3020 Mission

```bash
cd ~/collaboration/cobot3-ws-c2/cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

./scripts/run_amr_p3020_mission.sh \
  --ros-args \
  -p simulate_p3020:=false \
  -p pickup_x:=1.5 \
  -p pickup_y:=-2.0 \
  -p place_x:=-0.5 \
  -p place_y:=0.0
```

실제 P3020 Action 경로를 사용하려면 반드시 다음 값이 필요합니다.

```text
simulate_p3020:=false
```

---

# 10. 제출용 ZIP 생성 전 정리

과제 제출용 ZIP에는 **소스 및 Asset만 포함**하고 생성물/캐시/가상환경은 제외합니다.

## 반드시 포함

```text
README.md
requirements.txt
requirements/
backend/requirements.txt
frontend/package.json
frontend/package-lock.json
isaac_sim/
models/
ros2_ws/src/
scripts/
docs/
compose.yaml
.env.example
frontend/.env.example
```

특히 다음 파일/폴더는 삭제하지 않습니다.

```text
isaac_sim/usd/
isaac_sim/assets/
isaac_sim/assets/p3020/p3020.urdf
models/parcel_box_yolo_model/best.onnx
ros2_ws/src/
```

## 제출 전에 삭제

프로젝트 root에서:

```bash
sudo docker compose down --remove-orphans

rm -rf ros2_ws/build
rm -rf ros2_ws/install
rm -rf ros2_ws/log
rm -rf .venv
rm -rf .runtime
rm -rf frontend/node_modules
rm -rf frontend/dist
rm -rf frontend/.vite

find . -type d -name "__pycache__" -prune -exec rm -rf {} +
find . -type f -name "*.pyc" -delete
```

개인 환경 파일은 제출 ZIP에서 제외하고 example 파일만 포함하는 것을 권장합니다.

```bash
rm -f .env
rm -f frontend/.env
```

## ZIP 생성 예시

상위 폴더에서:

```bash
cd ..
zip -r cobot3-ws-c2_submission.zip cobot3-ws-c2 \
  -x "cobot3-ws-c2/.git/*" \
     "*/__pycache__/*" \
     "*.pyc"
```

## ZIP 검증

```bash
mkdir -p ~/submission_test
cd ~/submission_test
unzip ~/collaboration/cobot3-ws-c2/cobot3-ws-c2_submission.zip
cd cobot3-ws-c2
```

필수 파일 확인:

```bash
ls README.md requirements.txt
ls ros2_ws/src
ls isaac_sim/usd/Final_Real_Map
ls isaac_sim/assets/p3020
```

`build`, `install`, `log`가 제출본에 없는지 확인:

```bash
find ros2_ws -maxdepth 1 -type d -print
```

제출본에서는 `ros2_ws`와 `ros2_ws/src`만 남아 있는 것이 정상입니다.

---

## 프로젝트 구조

```text
cobot3-ws-c2/
├── backend/              # FastAPI + PostgreSQL + MQTT backend
├── frontend/             # React Control Tower
├── isaac_sim/
│   ├── assets/           # Robot / gripper / parcel assets
│   ├── equipment/        # Conveyor / sorter controller
│   ├── robots/           # IW Hub / P3020 control
│   ├── usd/              # Warehouse USD + navigation map
│   ├── main_mission.py
│   └── main_mission_live_view.py
├── models/               # YOLO ONNX model
├── mqtt/                 # Mosquitto configuration
├── requirements/         # Host Python dependencies
├── ros2_ws/
│   └── src/              # ROS2 source packages
├── scripts/              # Setup / launch / mission scripts
├── docs/                 # Architecture / scenario documents
├── compose.yaml
├── requirements.txt
└── README.md
```

---

## 주요 문서

- `docs/NEW_PC_SETUP.md`
- `docs/LIVE_MONITOR.md`
- `docs/conveyor_sorter_control.md`
- `docs/scenario/amr_pose_sync_complete.md`
- `docs/scenario/control_tower_live_integration.md`
- `docs/architecture/locate_box_pipeline.md`

---

## 주의 사항

- `isaac_sim/project_config/`를 사용합니다. 일반적인 `config/` 이름은 Isaac Sim/OpenCV 내부 모듈과 충돌할 수 있습니다.
- 제출 ZIP 생성 시 `ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log`를 반드시 제외합니다.
- `.venv`, `node_modules`, `__pycache__`, `.pyc`는 제출하지 않습니다.
- `.env`에는 개인 환경값이 들어갈 수 있으므로 `.env.example`을 제출합니다.
