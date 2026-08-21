# ForkliftB 한 대 LiDAR · 지도 · Nav2 장애물 회피

`amr_a` 한 대를 대상으로 Isaac Sim RTX LiDAR, ROS 2 Jazzy, Nav2를
연결한다. 정적 지도 위에서 Hybrid-A* 계열 경로를 만들고, LiDAR로 발견한
장애물을 global/local costmap에 표시해 우회하거나 안전 정지한다.

## 1. 브랜치와 ROS 2 의존성 준비

```bash
cd ~/collaboration/cobot3-ws-c2
git switch feat/forkliftb-single-navigation
git pull

sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-keyboard

./scripts/setup_ros.sh
```

모든 터미널은 같은 DDS 설정을 사용한다.

```bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 2. Isaac Sim 실행

첫 번째 터미널:

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/run_isaac.sh
```

Isaac Sim 입출력은 다음과 같다.

| 방향 | 토픽 | 용도 |
|---|---|---|
| 발행 | `/clock` | 모든 Nav2 노드의 시뮬레이션 시간 |
| 발행 | `/amr_a/scan` | `Example_Rotary_2D` RTX LiDAR LaserScan |
| 발행 | `/amr_a/odom` | 지게차 pose와 속도 |
| 발행 | `/tf` | `amr_a/odom → amr_a/base_link → amr_a/lidar_link` |
| 구독 | `/amr_a/drive_cmd_vel` | 안전 중재를 마친 최종 속도 |
| 구독 | `/amr_a/fork_up` | Fork 상승/하강 |

두 번째 터미널에서 입력이 정상인지 먼저 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/collaboration/cobot3-ws-c2/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic echo /clock --once
ros2 topic echo /amr_a/scan --once
ros2 topic echo /amr_a/odom --once
ros2 run tf2_ros tf2_echo amr_a/base_link amr_a/lidar_link
```

## 3. Nav2 실행

두 번째 터미널에서 다음을 실행한다.

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/run_ros2.sh
```

현재 지도는 Isaac 월드 원점과 일치하는 `40 m × 40 m` 정적 지도다.
실제 벽과 이동 장애물은 `/amr_a/scan`을 통해 costmap에 반영된다.

## 4. Start 후 목표 전송

Nav2는 실행 직후 정지 상태다. 세 번째 터미널에서 Start 신호를 보낸다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/collaboration/cobot3-ws-c2/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub --once /amr_a/navigation_enabled std_msgs/msg/Bool \
  "{data: true}"
```

월드 좌표 `(3.0, 1.5)`로 이동하는 예시:

```bash
ros2 action send_goal /amr_a/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

경로 앞에 장애물을 배치하면 RTX LiDAR가 장애물을 local/global costmap에
표시한다. Nav2는 새 경로를 계산하며, 지게차 바로 주변 정지 영역에 물체가
들어오면 Collision Monitor가 최종 명령을 0으로 만든다.

## 5. Stop, E-stop, 수동 운전

일시정지(기존 NavigateToPose action은 유지되므로 다시 Start하면 재개):

```bash
ros2 topic pub --once /amr_a/navigation_enabled std_msgs/msg/Bool \
  "{data: false}"
```

비상정지와 해제:

```bash
ros2 topic pub --once /amr_a/emergency_stop std_msgs/msg/Bool \
  "{data: true}"

ros2 topic pub --once /amr_a/emergency_stop std_msgs/msg/Bool \
  "{data: false}"
```

키보드 수동 운전은 Nav2를 종료하지 않고 별도 터미널에서 실행할 수 있다.
최근 0.5초 이내의 수동 명령이 Nav2보다 우선하며, 수동 입력이 끊기면
Nav2 명령으로 자동 복귀한다.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/amr_a/manual_cmd_vel
```

속도 명령 우선순위는 다음과 같다.

1. `/amr_a/emergency_stop=true`: 무조건 정지
2. `/amr_a/manual_cmd_vel`: 최신 수동 명령
3. `/amr_a/cmd_vel_safe`: Collision Monitor를 통과한 Nav2 명령
4. 나머지 경우: 정지

## 6. RViz 확인

RViz2를 `use_sim_time=true`, Fixed Frame을 `map`으로 열고 다음 display를
추가한다.

- Map: `/amr_a/map`
- LaserScan: `/amr_a/scan`
- Map: `/amr_a/global_costmap/costmap`
- Map: `/amr_a/local_costmap/costmap`
- Path: `/amr_a/plan`
- TF

## 현재 전제와 튜닝 지점

- Localization은 Isaac의 ground-truth pose를 사용하므로 `map → amr_a/odom`
  변환은 identity다. SLAM/AMCL은 다음 단계에서 바꾼다.
- 지게차 footprint는 전방 1.8 m, 후방 1.4 m, 좌우 0.9 m로 보수적으로
  설정했다. 실제 모델 치수를 확인한 뒤 `nav2_params.yaml`을 조정한다.
- 최소 회전 반경은 1.75 m, 후진은 허용했다. 제자리 회전은 경로 추종에서
  사용하지 않는다.
- `/scan`, `/odom`, 최종 속도 명령이 0.5초 이상 끊기면 안전 정지한다.
