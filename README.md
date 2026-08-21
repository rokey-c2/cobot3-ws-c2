# AMR–협동로봇 연계 택배 분류 자동화 및 실시간 관제 시스템

## MVP Flow
Input Zone
→ ForkliftB AMR
→ Doosan P3020
→ Main Conveyor
→ Wheel Sorter
→ A/B Conveyor
→ P3020 A/B 적재
→ Box Full
→ 출고 ForkliftB
→ 배송지

## 장비 구성()
- ForkliftB AMR × 3
  - 입고 1대
  - 출고 A 1대
  - 출고 B 1대
- Doosan P3020 × 3
  - 입고 1대
  - A 적재 1대
  - B 적재 1대
- Main Conveyor × 1
- Branch Conveyor × 2
- Wheel Sorter × 1

## 개발 환경
- Isaac Sim 5.1.0
- ROS2 Jazzy
- ROS_DOMAIN_ID=110
- RMW_IMPLEMENTATION=rmw_fastrtps_cpp
- PhysX
- Python

## ForkliftB 한 대 실행

각 명령은 별도 터미널에서 실행합니다.

```bash
./scripts/run_isaac.sh
./scripts/run_ros2.sh
./scripts/run_foxglove.sh
```

- 자율주행 설정: `docs/amr_ros2_setup.md`
- Foxglove 관제 설정: `docs/foxglove_monitoring.md`

## 중요
`isaac_sim/project_config/`를 사용합니다.

`config/`라는 일반 이름은 Isaac Sim/OpenCV 내부 모듈과 충돌할 수 있으므로 사용하지 않습니다.
