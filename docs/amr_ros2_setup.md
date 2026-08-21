# ForkliftB 한 대 목표점 자율주행 실행 방법

이 단계에서는 `amr_a` 한 대가 Isaac Sim의 현재 위치를 ROS 2로 보내고,
ROS 2가 목표 XY 좌표까지 자동으로 주행 명령을 계산한다.

현재 구현은 정적 환경에서의 목표점 추종 MVP이다. LiDAR 장애물 감지,
SLAM, Nav2 경로 계획은 다음 단계에서 연결한다.

## 1. ROS 2 패키지 빌드

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/setup_ros.sh
```

## 2. Isaac Sim 실행

첫 번째 터미널:

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/run_isaac.sh
```

Isaac Sim은 다음 데이터를 발행하고 명령을 구독한다.

```text
발행: /amr_a/odom
발행: /tf  (amr_a/odom → amr_a/base_link)
구독: /amr_a/cmd_vel
구독: /amr_a/fork_up
```

## 3. 목표점 자율주행 노드 실행

두 번째 터미널:

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/run_ros2.sh
```

## 4. 목표 좌표 전송

세 번째 터미널에서 먼저 현재 위치를 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/collaboration/cobot3-ws-c2/ros2_ws/install/setup.bash

ros2 topic echo /amr_a/odom --once
```

현재 위치에서 월드 좌표 `(3.0, 1.5)`로 이동시키는 예시다.

```bash
ros2 topic pub --once /amr_a/goal_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'amr_a/odom'}, pose: {position: {x: 3.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}"
```

도착 여부는 다음 토픽으로 확인한다.

```bash
ros2 topic echo /amr_a/goal_reached
```

## 5. 이전 시간 기반 동작 테스트

기존 테스트 노드는 삭제하지 않았다. 필요하면 직접 실행한다.

```bash
ros2 launch amr_controller amr_controller.launch.py namespace:=amr_a
```

자율주행 노드와 시간 기반 테스트 노드를 동시에 실행하면 두 노드가 같은
`/amr_a/cmd_vel`을 발행하므로 동시에 실행하지 않는다.

## 현재 안전 동작

- 목표가 뒤쪽이면 후진 경로를 자동으로 선택한다.
- `odom`이 0.5초 이상 끊기면 즉시 정지한다.
- 새로운 `/cmd_vel`이 0.5초 이상 없으면 Isaac 측에서도 정지한다.
- 목표점 0.15 m 이내에 들어오면 정지하고 `goal_reached=true`를 발행한다.
