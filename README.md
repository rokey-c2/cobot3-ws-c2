# AMR–협동로봇 연계 택배 분류 자동화 및 실시간 관제 시스템

## 현재 검증 시나리오

현재 `feat/iw-hub-single-navigation` 브랜치는 Isaac Sim 5.1 + ROS 2 Jazzy + NVIDIA IW Hub Nav2 구성을 사용합니다.

실행 흐름:

```text
IW Hub 시작 위치 (10.5, 1.80122, yaw=0°)
→ 제자리 +90° 회전
→ Cargo Pod 위치로 직선 접근 (10.5, -1.25, yaw=90°)
→ Lift Up
→ Nav2 배송지 이동 (1.30104, -0.06065)
→ P3020 PickPlace Action
→ Nav2 Cargo 복귀 구간
→ Local 정밀 Dock
→ Lift Down
→ Cargo 원위치 확인
→ IW Hub 시작 위치 복귀
```

Cargo Pod 안의 택배 상자는 NVIDIA Simple Warehouse의 cardboard box asset을 사용하며 PhysX 물리 설정을 적용합니다.

## 개발 환경

- Ubuntu 24.04
- Isaac Sim 5.1.0
- ROS 2 Jazzy
- ROS_DOMAIN_ID=110
- RMW_IMPLEMENTATION=rmw_fastrtps_cpp
- Nav2
- RTX LiDAR
- PhysX
- Python

## NVIDIA IW Hub 구성

IW Hub는 NVIDIA Isaac Sim 5.1의 공식 Navigation sample 구성을 reference해서 사용합니다.

프로젝트 코드에서 LiDAR를 새로 만들거나 센서 위치, 방향, range를 변경하지 않습니다.
NVIDIA Navigation sample의 front/back 2D LiDAR와 ROS 2 graph를 그대로 사용합니다.

기본 ROS 2 토픽:

```text
/cmd_vel
/chassis/odom
/front_2d_lidar/scan
/back_2d_lidar/scan
```

## 실행

### Terminal 1 — Isaac Sim

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/run_isaac_mission.sh
```

### Terminal 2 — Nav2

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/run_ros2.sh
```

### Terminal 3 — 전체 Mission

P3020 simulation mode:

```bash
cd ~/collaboration/test/cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
./scripts/run_amr_p3020_mission.sh --ros-args -p simulate_p3020:=true
```

실제 P3020 Action Server 연동 시:

```bash
./scripts/run_amr_p3020_mission.sh --ros-args -p simulate_p3020:=false
```

## 주요 파일

```text
isaac_sim/main_mission.py
isaac_sim/project_config/robot_config.py
isaac_sim/robots/iw_hub/iw_hub_agent.py
isaac_sim/robots/iw_hub/iw_hub_mission_agent.py
isaac_sim/cargo/cargo_pod_physics.py
ros2_ws/src/amr_controller/amr_controller/amr_p3020_mission.py
ros2_ws/src/logistics_interfaces/action/PickPlace.action
scripts/run_isaac_mission.sh
scripts/run_ros2.sh
scripts/run_amr_p3020_mission.sh
```

## 중요 원칙

- NVIDIA IW Hub 기본 LiDAR/ROS 2 graph를 임의로 변경하지 않습니다.
- custom LiDAR, scan self filter, odom TF bridge를 사용하지 않습니다.
- `main` 브랜치는 직접 수정하지 않습니다.
- ROS 2의 `build/`, `install/`, `log/` 및 Python `__pycache__/`는 Git에 포함하지 않습니다.
