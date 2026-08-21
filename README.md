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

## IW Hub 자율주행 실행

```bash
./scripts/setup_ros.sh
./scripts/run_isaac.sh
./scripts/run_ros2.sh
```

상세 실행 및 목표 전송 방법은 [docs/iw_hub_navigation.md](docs/iw_hub_navigation.md)를 참고합니다.

## 중요
`isaac_sim/project_config/`를 사용합니다.

`config/`라는 일반 이름은 Isaac Sim/OpenCV 내부 모듈과 충돌할 수 있으므로 사용하지 않습니다.

