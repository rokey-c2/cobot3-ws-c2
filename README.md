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

## IW Hub 단일 자율주행 테스트

현재 `feat/iw-hub-single-navigation` 브랜치는 프로젝트 Warehouse USD에
NVIDIA Isaac Sim 5.1의 공식 `iw_hub_warehouse_navigation.usd` 안에 들어 있는
IW Hub Nav2 로봇 구성을 reference해서 사용합니다.

프로젝트 코드에서 LiDAR를 새로 만들거나 센서 위치, 방향, range를 변경하지 않습니다.
NVIDIA Navigation 샘플의 front/back 2D LiDAR와 ROS 2 graph를 그대로 사용합니다.
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

## 실행

터미널 1:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/run_isaac.sh
```

터미널 2:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/run_ros2.sh
```

터미널 3:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/test_iw_hub_avoidance.sh
```

IW Hub 상세 내용은 [docs/iw_hub_navigation.md](docs/iw_hub_navigation.md)를 참고합니다.

## 중요
`isaac_sim/project_config/`를 사용합니다.

- NVIDIA IW Hub 기본 LiDAR/ROS 2 graph를 임의로 변경하지 않습니다.
- custom LiDAR, scan self filter, odom TF bridge를 사용하지 않습니다.
- `main` 브랜치는 직접 수정하지 않습니다.
- ROS 2의 `build/`, `install/`, `log/` 및 Python `__pycache__/`는 Git에 포함하지 않습니다.
