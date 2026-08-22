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
- NVIDIA `iw_hub_navigation`
- Isaac Sim 기본 IW Hub Sensor 에셋

## IW Hub 단일 자율주행 테스트

현재 `feat/iw-hub-single-navigation`은 Isaac Sim 기본 IW Hub Sensor 에셋의
ROS graph와 LiDAR를 수정하지 않고 그대로 사용합니다.

기본 센서/주행 토픽:

- `/front_2d_lidar/scan`
- `/back_2d_lidar/scan`
- `/chassis/odom`
- `/cmd_vel`

프로젝트에서 별도의 RTX LiDAR를 생성하거나 LiDAR range 값을 변경하지 않습니다.
scan self-filter와 odom TF bridge도 사용하지 않습니다.

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

추가 테스트 장애물과 cargo는 생성하지 않습니다.

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

`run_ros2.sh`는 기본 IW Hub의 front/back 2D LiDAR와 `/chassis/odom`이 실제로
publish되는지 확인한 뒤 NVIDIA `iw_hub_navigation`을 프로젝트 맵으로 실행합니다.

터미널 3:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/test_iw_hub_avoidance.sh
```

이 스크립트는 `map` 기준 목표 좌표 `(-13.813100814819336, 0.7454315423965454)`로
`NavigateToPose` goal을 전송합니다.

## 중요

`isaac_sim/project_config/`를 사용합니다.

`config/`라는 일반 이름은 Isaac Sim/OpenCV 내부 모듈과 충돌할 수 있으므로 사용하지 않습니다.
