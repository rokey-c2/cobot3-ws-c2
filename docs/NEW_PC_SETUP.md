# New PC Setup

이 문서는 팀원이 GitHub에서 `main`을 새로 clone한 뒤 프로젝트를 실행하는 기준 절차입니다.

## 1. 외부 플랫폼 요구사항

Git 저장소에서 자동으로 설치하지 않는 항목:

- Ubuntu 24.04
- Isaac Sim 5.1.0
- ROS 2 Jazzy
- Docker + Docker Compose plugin
- NVIDIA `iw_hub_navigation` 패키지가 빌드된 Isaac ROS Jazzy workspace

기본 경로:

```text
~/isaacsim/python.sh
~/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash
```

다른 경로라면 실행 전에 지정합니다.

```bash
export ISAAC_SIM_DIR=/path/to/isaacsim
export ISAAC_ROS_WS=/path/to/jazzy_ws
```

프로젝트 표준 ROS 환경:

```bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`~/.bashrc` 수정은 필요하지 않습니다.

## 2. Clone

```bash
git clone https://github.com/rokey-c2/cobot3-ws-c2.git
cd cobot3-ws-c2
git switch main
```

## 3. 최초 1회 설정

가장 쉬운 방법:

```bash
./scripts/quick_setup.sh --install-system-deps
```

작은 Ubuntu helper package가 이미 설치되어 있으면:

```bash
./scripts/quick_setup.sh
```

자동 처리 항목:

1. `.env` 생성
2. `frontend/.env` 생성
3. npm dependency 설치
4. `.venv` 생성 및 `onnxruntime`, MQTT adapter dependency 설치
5. rosdep dependency 설치
6. `ros2_ws` colcon build
7. Docker image build
8. 환경 검사

### 수동으로 나눠서 설정하려면

```bash
cp .env.example .env
cp frontend/.env.example frontend/.env

./scripts/setup_frontend.sh
./scripts/install_ros_dependencies.sh
./scripts/setup_vision_env.sh
./scripts/setup_adapter_env.sh

cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
cd ..

./scripts/check_environment.sh
```

## 4. 전체 시스템 시작

DB 유지:

```bash
./scripts/start_all.sh
```

DB/MQTT volume 초기화:

```bash
./scripts/start_all.sh --fresh-db
```

자동 시작 순서:

```text
Docker
→ Isaac
→ ROS sensor 확인
→ Nav2
→ Adapter
→ P3020 Action Server
→ Vision
→ Frontend
→ Nav2 READY
→ Pose Sync
```

실행 프로세스는 `.runtime/pids/`, 로그는 `.runtime/logs/`에서 관리합니다.

## 5. 실제 미션

별도로 시작:

```bash
./scripts/start_mission.sh
```

전체 시스템과 동시에 자동 시작:

```bash
./scripts/start_all.sh --mission
```

DB까지 초기화하고 전체 데모를 한 번에 시작:

```bash
./scripts/start_all.sh --fresh-db --mission
```

`start_mission.sh`는 `simulate_p3020:=false`를 기본 사용하므로 실제 P3020 Action Server와 YOLO 경로를 사용합니다.

## 6. 상태 / 로그 / 종료

```bash
./scripts/status_all.sh
```

```bash
tail -f .runtime/logs/isaac.log
tail -f .runtime/logs/vision.log
tail -f .runtime/logs/mission.log
```

데이터 유지 종료:

```bash
./scripts/stop_all.sh
```

Docker volume까지 삭제:

```bash
./scripts/stop_all.sh --volumes
```

## 7. Dependency 관리 위치

- Frontend: `frontend/package.json`, `frontend/package-lock.json`
- Backend: `backend/requirements.txt` + Dockerfile
- ROS2: `ros2_ws/src/*/package.xml`
- Vision: `requirements/vision.txt`
- ROS2 MQTT Adapter: `requirements/ros2-adapter.txt`
- PostgreSQL / MQTT / Backend: `compose.yaml`
- Isaac Sim: 외부 설치
- `iw_hub_navigation`: 외부 Isaac ROS workspace

## 8. 자주 생기는 문제

### `onnxruntime` 없음

```bash
./scripts/setup_vision_env.sh
```

### Python venv 생성 실패

```bash
sudo apt install -y python3.12-venv
rm -rf .venv
./scripts/setup_vision_env.sh
./scripts/setup_adapter_env.sh
```

### `iw_hub_navigation package not found`

```bash
ls ~/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash
```

다른 경로라면:

```bash
export ISAAC_ROS_WS=/actual/path/to/jazzy_ws
```

### Docker DB를 처음부터 다시 만들기

```bash
./scripts/stop_all.sh --volumes
./scripts/start_all.sh --fresh-db
```

### 현재 실행 상태가 이상할 때

```bash
./scripts/status_all.sh
```

그 다음 관련 로그를 확인합니다.
