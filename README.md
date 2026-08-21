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

## 장비 구성
- IW Hub AMR
  - 1차 구현: `amr_a` 한 대 Nav2 자율주행 검증
  - 확장: 입고 1대, 출고 A/B 각 1대
- Doosan P3020 × 3
  - 입고 1대
  - A 적재 1대
  - B 적재 1대
- Main Conveyor × 1
- Branch Conveyor × 2
- Wheel Sorter × 1

## 개발 환경
- Isaac Sim 5.1.0
- ROS 2 Jazzy
- ROS_DOMAIN_ID=110
- RMW_IMPLEMENTATION=rmw_fastrtps_cpp
- Nav2
- RTX LiDAR
- PhysX
- Python

## IW Hub 컨테이너 운반 실행

현재 Isaac 실행 월드는 Ground Plane, 박스를 담은 컨테이너와
`amr_a` IW Hub 한 대만 생성합니다.
P3020, 컨베이어, ForkliftB는 이 실행 구성에 포함하지 않습니다.

장애물 회피 실험은 출발 `(1.5, 0.0)`, 장애물 `(3.5, 0.0)`,
목표 `(6.0, 0.0)` 구성입니다.

```bash
./scripts/setup_ros.sh
./scripts/run_isaac.sh
./scripts/run_ros2.sh
./scripts/test_iw_hub_container_mission.sh
```

세 번째 스크립트는 컨테이너 리프트 상승(0.30 m), 장애물 회피 주행,
목표 `(6.0, 0.0)` 도착, 리프트 하강과 컨테이너 배치를 순서대로 수행합니다.
주행만 다시 확인할 때는 `./scripts/test_iw_hub_avoidance.sh`를 사용합니다.

상세 실행 및 목표 전송 방법은 [docs/iw_hub_navigation.md](docs/iw_hub_navigation.md)를 참고합니다.

## 중요
`isaac_sim/project_config/`를 사용합니다.

`config/`라는 일반 이름은 Isaac Sim/OpenCV 내부 모듈과 충돌할 수 있으므로 사용하지 않습니다.
