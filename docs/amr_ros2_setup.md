# Forklift AMR ROS 2 실행 방법

이 구성은 `.bashrc`를 수정하지 않는다.

- Isaac Sim은 내부 Python 3.11용 `rclpy`를 사용한다.
- 외부 ROS 2 노드는 `/opt/ros/jazzy`의 Python 3.12 환경을 사용한다.
- 두 프로그램은 ROS 2 토픽으로 통신한다.

## 1. ROS 2 패키지 빌드

프로젝트 루트에서 한 번 실행한다.

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/setup_ros.sh
```

다음과 같이 출력되면 정상이다.

```text
[ROS2] 빌드 완료
[ROS2] Package: .../ros2_ws/install/amr_controller
```

## 2. Isaac Sim 실행

새 터미널을 열고 실행한다.

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/run_isaac.sh
```

이 터미널에서는 `ros_set`을 먼저 실행할 필요가 없다.
스크립트가 Isaac Sim 내부 ROS 2 경로를 직접 설정한다.

Isaac Sim 설치 위치가 `~/isaacsim`이 아니라면 다음처럼 실행한다.

```bash
ISAAC_SIM_DIR=/Isaac/설치/경로 ./scripts/run_isaac.sh
```

## 3. AMR ROS 2 노드 실행

Isaac Sim이 완전히 실행된 후 다른 터미널에서 실행한다.

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/run_ros2.sh
```

기본 namespace는 `amr_a`이다. 따라서 다음 토픽을 사용한다.

```text
/amr_a/cmd_vel
/amr_a/fork_up
```

다른 namespace를 사용하려면 다음처럼 실행한다.

```bash
./scripts/run_ros2.sh namespace:=amr_in_01
```

이 경우 Isaac의 `ROS2_NAMESPACE`도 `amr_in_01`로 같아야 한다.

## 4. 기본 테스트 순서

ROS 2 노드를 실행하면 다음 동작을 한 번 수행한다.

1. 직진
2. 좌회전
3. 목표 위치까지 직진
4. 정지
5. Fork 상승
6. Fork 상승 상태 유지
7. Fork 하강
8. 후진
9. 후진하면서 회전
10. 시작 위치까지 후진
11. 정지

## 5. 토픽 확인

세 번째 터미널에서 확인할 수 있다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/collaboration/cobot3-ws-c2/ros2_ws/install/setup.bash

ros2 topic list | grep amr_a
ros2 topic echo /amr_a/cmd_vel
```
