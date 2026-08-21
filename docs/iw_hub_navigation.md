# IW Hub 단일 AMR 자율주행

이 구성은 저장된 물류 월드를 불러오지 않고 빈 테스트 월드에서
Idealworks IW Hub 한 대(`amr_a`)만 실행한다. Isaac Sim 5.1에서
odometry, RTX LiDAR, simulation clock을 발행하고 ROS 2 Jazzy Nav2가
목표 경로 계획, LiDAR 장애물 반영, 충돌 정지, 속도 명령 중재를 담당한다.

## 데이터 흐름

`NavigateToPose → controller → velocity smoother → collision monitor → velocity mux → /amr_a/drive_cmd_vel → IW Hub`

| 방향 | 토픽 | 용도 |
|---|---|---|
| Isaac → ROS 2 | `/clock` | Nav2 simulation time |
| Isaac → ROS 2 | `/amr_a/odom` | IW Hub ground-truth odometry |
| Isaac → ROS 2 | `/amr_a/scan` | RTX 2D LiDAR |
| ROS 2 → Isaac | `/amr_a/drive_cmd_vel` | 안전 중재 후 최종 속도 |
| ROS 2 → Isaac | `/amr_a/lift_cmd` | lift joint command |

## 1. 의존성 설치와 빌드

```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

cd ~/collaboration/cobot3-ws-c2
./scripts/setup_ros.sh
```

모든 터미널은 `ROS_DOMAIN_ID=110`,
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 사용한다. 실행 스크립트가 두 값을 자동 설정한다.

## 2. 실행

터미널 1:

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/run_isaac.sh
```

터미널 2:

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/run_ros2.sh
```

## 3. 자율주행 시작과 목표 전송

기본 장애물 회피 실험 배치는 다음과 같다.

| 항목 | 좌표/크기 |
|---|---|
| IW Hub 출발 | `(1.5, 0.0)` |
| 경로 장애물 | 중심 `(3.5, 0.0)`, 크기 `0.8 × 1.4 × 1.0 m` |
| 이동 목표 | `(6.0, 0.0)` |

Isaac Sim과 Nav2를 실행한 뒤 세 번째 터미널에서 실험 스크립트를 실행한다.

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/test_iw_hub_avoidance.sh
```

직선 경로 중앙의 빨간 장애물을 RTX LiDAR가 감지하고, Nav2 global/local
costmap에 반영하여 IW Hub가 장애물 옆으로 우회하는지 확인한다.

### 목표를 직접 전송하는 방법

Nav2 실행 직후에는 안전을 위해 최종 속도가 차단돼 있다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/collaboration/cobot3-ws-c2/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub --once /amr_a/navigation_enabled std_msgs/msg/Bool \
  "{data: true}"

ros2 action send_goal /amr_a/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

## 4. 확인

```bash
ros2 topic echo /clock --once
ros2 topic echo /amr_a/scan --once
ros2 topic echo /amr_a/odom --once
ros2 run tf2_ros tf2_echo map amr_a/base_link
ros2 action list | grep /amr_a/navigate_to_pose
```

비상정지와 해제:

```bash
ros2 topic pub --once /amr_a/emergency_stop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /amr_a/emergency_stop std_msgs/msg/Bool "{data: false}"
```

## 현재 범위

- IW Hub `amr_a` 한 대부터 검증한다.
- 세계 좌표와 odometry가 같다는 전제로 `map → amr_a/odom`을 identity로 둔다.
- 정적 지도는 40 m × 40 m 경계 지도이며 실제 장애물은 LiDAR costmap에 반영한다.
- `spawn_xyz`, footprint, LiDAR 높이는 실제 월드 배치 확인 후 조정한다.
- ForkliftB 코드는 이 실행 경로와 로봇 registry에서 사용하지 않는다.
- P3020, 컨베이어, ForkliftB와 다른 AMR은 생성하지 않는다.
