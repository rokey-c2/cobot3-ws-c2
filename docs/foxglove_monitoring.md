# Foxglove로 ForkliftB 자율주행 모니터링

원본 Nova Carter 실습의 Foxglove 구성을 이 저장소의 ROS 2 Jazzy,
ForkliftB 한 대, `amr_a` namespace에 맞게 적용한다.

## 적용 내용

- Foxglove Bridge WebSocket: 기본 `0.0.0.0:8765`
- 지도, LiDAR, odometry, TF, Nav2 경로와 costmap 실시간 표시
- Foxglove 3D 패널에서 `/amr_a/goal_pose` 발행
- `foxglove_goal_bridge`가 PoseStamped를
  `/amr_a/navigate_to_pose` action으로 변환
- Teleop 패널은 안전 중재 입력인 `/amr_a/manual_cmd_vel` 사용

Nova Carter 실습의 `/cmd_vel`, `/goal_pose`, `/map`을 그대로 쓰면 이
프로젝트의 namespace 및 안전 명령 흐름을 우회한다. 반드시 아래의
`/amr_a/...` 토픽을 사용한다.

## 1. 설치와 빌드

```bash
cd ~/collaboration/cobot3-ws-c2
git switch feat/forkliftb-single-navigation
git pull

sudo apt update
sudo apt install ros-jazzy-foxglove-bridge

./scripts/setup_ros.sh
```

각 터미널은 같은 DDS 설정을 사용한다.

```bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Isaac Sim Python과 시스템 ROS 2 Python의 충돌을 피하기 위해 Isaac Sim,
Nav2, Foxglove Bridge는 각각 별도 터미널에서 실행한다.

## 2. 실행 순서

터미널 1 — Isaac Sim:

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/run_isaac.sh
```

터미널 2 — Nav2:

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/run_ros2.sh
```

터미널 3 — Foxglove Bridge와 목표 변환 노드:

```bash
cd ~/collaboration/cobot3-ws-c2
./scripts/run_foxglove.sh
```

정상 실행 확인:

```bash
ros2 node list | grep -E 'foxglove_bridge|foxglove_goal_bridge'
ss -ltn | grep ':8765'
ros2 action list | grep /amr_a/navigate_to_pose
```

## 3. Foxglove 연결

GPU PC에서 접속할 때는 `ws://localhost:8765`를 사용한다. 다른 팀 노트북은
GPU PC와 같은 네트워크에 연결하고 GPU PC의 주소를 확인한다.

```bash
hostname -I
```

예를 들어 GPU PC 주소가 `10.10.0.2`이면 다른 노트북에서
`ws://10.10.0.2:8765`로 접속한다.

1. Chrome에서 <https://app.foxglove.dev>를 연다.
2. **Open connection** → **Foxglove WebSocket**을 선택한다.
3. WebSocket URL을 입력하고 **Open**을 누른다.

UFW가 활성화돼 있고 다른 PC에서 연결되지 않을 때만 포트를 연다.

```bash
sudo ufw status
sudo ufw allow 8765/tcp
```

## 4. 3D 패널 설정

3D 패널의 **Fixed frame**과 **Display frame**을 모두 `map`으로 설정한다.
다음 토픽을 표시한다.

| 토픽 | 용도 |
|---|---|
| `/amr_a/map` | 정적 점유 지도 |
| `/amr_a/scan` | ForkliftB RTX LiDAR |
| `/amr_a/odom` | 위치와 속도 확인 |
| `/tf`, `/tf_static` | 좌표계 연결 |
| `/amr_a/plan` | Nav2 전역 경로 |
| `/amr_a/global_costmap/costmap` | 전역 장애물 비용 지도 |
| `/amr_a/local_costmap/costmap` | 근거리 장애물 회피 비용 지도 |
| `/amr_a/foxglove_goal_status` | Foxglove 목표 처리 상태 |

TF 이름이 너무 많이 보이면 3D 패널의 **Scene → Label scale**을 `0`으로
설정한다.

현재 브랜치는 Isaac ground-truth pose와 identity `map → amr_a/odom` 변환을
사용한다. Nova Carter 자료와 달리 AMCL용 `/initialpose`를 보낼 필요가 없다.
또한 현재 ForkliftB 구성에는 카메라 토픽이 없으므로 Image 패널은 제외한다.

## 5. Foxglove에서 목표 보내기

먼저 안전 중재기의 Nav2 입력을 활성화한다.

```bash
ros2 topic pub --once /amr_a/navigation_enabled std_msgs/msg/Bool \
  "{data: true}"
```

3D 패널의 Publish 설정에서 2D pose 토픽을 다음과 같이 지정한다.

- Schema: `geometry_msgs/PoseStamped`
- Topic: `/amr_a/goal_pose`
- Frame: `map`

그다음 3D 지도에서 목표 위치를 누른 채 드래그해 최종 방향을 정한다.
`foxglove_goal_bridge`가 이 메시지를 Nav2의
`/amr_a/navigate_to_pose` action으로 전달한다.

상태 확인:

```bash
ros2 topic echo /amr_a/foxglove_goal_status
```

주요 상태는 `queued`, `sending`, `accepted`, `finished:<status>`이다.

## 6. Teleop, Stop, E-stop

Foxglove Teleop 패널은 다음처럼 설정한다.

- Topic: `/amr_a/manual_cmd_vel`
- Publish rate: `10 Hz` 이상

이 프로젝트의 수동 명령 유효시간은 0.5초이므로 원본 자료의 `1 Hz`를
사용하면 주행이 끊긴다. `/amr_a/cmd_vel`이나
`/amr_a/drive_cmd_vel`로 직접 보내지 않는다.

Nav2 일시정지:

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

## 문제 해결

| 증상 | 확인 및 해결 |
|---|---|
| Foxglove Topics가 비어 있음 | Isaac과 Nav2가 실행 중인지, 세 터미널의 `ROS_DOMAIN_ID=110`인지 확인 |
| `foxglove_bridge`가 없음 | `sudo apt install ros-jazzy-foxglove-bridge` |
| 다른 노트북에서 연결 실패 | 같은 네트워크인지 확인하고 `hostname -I`, `ss -ltn`, UFW 8765 확인 |
| 지도나 로봇 위치가 어긋남 | Fixed/Display frame을 `map`으로 설정하고 `/tf`, `/tf_static` 활성화 |
| 목표를 눌러도 이동하지 않음 | `/amr_a/navigation_enabled=true`, action 목록, `/amr_a/foxglove_goal_status` 확인 |
| Teleop이 끊겨 움직임 | `/amr_a/manual_cmd_vel`, Publish rate 10 Hz 이상 사용 |

Foxglove의 직접 WebSocket 연결 기능은 계정 유형에 따라 Developer seat가
필요할 수 있다. 연결 메뉴에서 권한 안내가 나오면 계정 권한을 확인한다.

## 참고

- [Foxglove로 Nova Carter 자율주행 모니터링](https://sonmiran9.oopy.io/b87450ef-7c59-8385-b8c7-81b82149438a)
- [Foxglove ROS 2 연결 안내](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)
