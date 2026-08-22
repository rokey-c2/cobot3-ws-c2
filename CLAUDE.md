# 프로젝트: 대형 물류 자동화 & 관제 제어 시스템

## 개요
- 기간: 2026-08-14 ~ 2026-08-28
- 입고(AMR) → P3020 Pick&Place → Main Conveyor → Wheel Sorter(A/B 분류) → 적재 P3020 → 출고 AMR
- GitHub: https://github.com/rokey-c2/cobot3-ws-c2
- ROS2 Jazzy, Isaac Sim 5.1.0, Python 3.12.3

## 이 PC(WSL) 관련 주의사항
- 이 PC는 팀의 4대 공식 개발 PC(정의석/황인재 개인 PC, ROKEY 공용 PC1/2)와 별개 환경이며, WSL 기반 Linux임
- 여기서 실제 개발 진행 후 결과물은 ROKEY 공용 PC2로 이전 예정
- 아래 팀 규칙은 이 PC에서도 동일하게 적용

## ROS2 공통 환경

    export ROS_DOMAIN_ID=110
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
    source /opt/ros/jazzy/setup.bash

- 별도 테스트 목적이 아니면 ROS_DOMAIN_ID 임의 변경 금지
- 공용 GPU PC의 Driver/CUDA/Isaac Sim/ROS2/Network/DDS/공통 .bashrc는 팀 협의 없이 변경 금지

## Naming Convention
| 대상 | Case | 예시 |
|---|---|---|
| Repository, 일반 폴더/문서 | kebab-case | cobot3-ws-c2 |
| ROS2 Package/Node, Python 파일·변수·함수 | snake_case | amr_controller, publish_status() |
| Python Class, React Component | PascalCase | AMRController |
| JS/TS 변수·함수 | camelCase | getRobotStatus() |
| 상수, 환경변수 | UPPER_SNAKE_CASE | MAX_SPEED, ROS_DOMAIN_ID |

- test1, temp2, asdf 등 의미 없는 이름 금지
- ROS2 관련 이름에는 - 대신 _ 사용
- src/, docs/, config/ 등 기본 폴더명과 README.md, .gitignore 등 표준 파일명은 그대로 유지

## Git / Commit
- main은 통합/안정 버전, 직접 개발 금지 → 개인 dev 브랜치에서 작업 후 PR
- 작업 시작 전: git status → git branch → git switch main → git pull origin main → 본인 dev 브랜치로 이동
- Commit prefix: feat, fix, docs, refactor, test, chore (예: feat: LocateBox 서비스 노드 추가)
- README.md, .gitignore, 공통 Launch/Interface/Config, Network, Docker, DB Schema 등 공통 파일 수정 전 팀에 공유

## 담당 역할 참고
- 정의석: 팀 리드 / AMR / 전체 통합
- 황인재: P3020 Manipulator
- 서진우: OpenCV 비전 인식
- 정동휘: Conveyor / Wheel Sorter (단, 현재 이 PC에서는 OpenCV 비전 인식 + P3020 로봇팔 제어 관련 작업 진행 중)

## 현재 작업 중: LocateBox 서비스 (영상인식 ↔ 로봇팔)
- 목적: 카메라(rgb/depth)로 박스를 검출하고 3D 좌표를 계산해 로봇팔에 제공
- YOLO(1클래스: box) + depth 픽셀 매핑으로 카메라 좌표계 3D point 계산

LocateBox.srv (확정본):

    # Request: 로봇팔 -> 영상인식
    bool release_lock
    # true  : 잠긴 좌표 해제 후 재검출 (Pick 완료 후 다음 사이클 시작 시)
    # false : 잠겨 있으면 같은 좌표 유지, 안 잠겨 있으면 새로 검출해서 잠금

    ---

    # Response: 영상인식 -> 로봇팔
    bool box_detected
    geometry_msgs/Point box_position   # 카메라 좌표계, meter
    bool position_locked               # true면 이번 응답이 고정된(재계산 아닌) 좌표

- 설계 이유: Pick 도중 반복 호출로 좌표가 흔들리는 것을 방지하기 위해 WAITING → DETECTED → LOCKED 상태를 노드 내부에서 관리
- 관련 서비스: ConfirmGrasp.srv (로봇팔 → 영상인식, rgb/depth 보내 흡착 성공 여부 확인)
- 카메라 intrinsics: bag에 /camera_info 없음 → Isaac Sim 카메라 Prim의 실제 Focal Length/Horizontal Aperture 값으로 교체 필요 (현재 placeholder: 640x480, 수평 FOV 60도 가정)

## 다음 단계
1. LocateBox 서비스 노드(rclpy) 작성 — WAITING/DETECTED/LOCKED 상태 관리
2. ros2 bag play로 업로드된 mcap 재생해 오프라인 검증
3. Isaac Sim 실카메라 연동 + 실제 intrinsics 반영
