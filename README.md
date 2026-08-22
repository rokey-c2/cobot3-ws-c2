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

상세 내용은 [docs/iw_hub_navigation.md](docs/iw_hub_navigation.md)를 참고합니다.

## 중요
`isaac_sim/project_config/`를 사용합니다.

`config/`라는 일반 이름은 Isaac Sim/OpenCV 내부 모듈과 충돌할 수 있으므로 사용하지 않습니다.
