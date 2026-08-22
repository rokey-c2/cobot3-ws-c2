# IW Hub 단일 AMR Nav2 자율주행

현재 `feat/iw-hub-single-navigation` 브랜치는 저장된 Warehouse USD에서 IW Hub 한 대(`amr_a`)를 실행하고, Isaac Sim 5.1의 RTX LiDAR/odometry와 ROS 2 Jazzy Nav2를 연결한다.

## 현재 기준 데이터 흐름

```text
Isaac Sim
  /clock
  /amr_a/odom
  /amr_a/scan_raw
        |
        v
scan_self_filter
  /amr_a/scan
        |
        +--> AMCL
        +--> local/global costmap
        +--> collision monitor

/amr_a/odom
        |
        v
odom_tf_bridge
  odom -> base_link

AMCL
  map -> odom

Nav2
  /amr_a/drive_cmd_vel -> IW Hub
```

`/amr_a/scan_raw`은 Isaac RTX LiDAR 원본이며 디버깅용이다. Nav2는 반드시 self-reflection이 제거된 `/amr_a/scan`을 사용한다.

## 왜 scan_self_filter가 필요한가

IW Hub의 360도 RTX LiDAR는 센서 주변의 로봇 차체 일부를 실제 장애물처럼 측정할 수 있다. 이 값을 그대로 costmap에 넣으면 로봇이 이동할 때 뒤쪽에 벽이 생기는 것처럼 보이고, planner/controller/collision monitor가 자기 몸체를 장애물로 오인할 수 있다.

`scan_self_filter`는 LaserScan 각 점을 `base_link` 좌표의 `(x, y)`로 변환하고 IW Hub 차체 footprint 내부의 점만 `inf`로 제거한다. 외부 장애물은 그대로 유지한다.

기본 self-filter 영역:

```text
x: -1.15 ~ 0.35 m
y: -0.35 ~ 0.35 m
```

필요하면 ROS parameter로 조정할 수 있다.

## 실행

터미널 1:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/setup_ros.sh
./scripts/run_isaac.sh
```

터미널 2:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/run_ros2.sh
```

`run_ros2.sh`는 다음을 자동으로 수행한다.

1. ROS 2 Jazzy와 프로젝트 workspace source
2. 필요 시 `~/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash` source
3. `/clock`, `/amr_a/odom`, `/amr_a/scan_raw` 확인
4. `scan_self_filter` 실행
5. `odom_tf_bridge` 실행
6. `/amr_a/scan` filtered stream 확인
7. `iw_hub_navigation.launch.py`를 custom warehouse map/params로 실행

## 확인

```bash
ros2 topic hz /amr_a/scan_raw
ros2 topic hz /amr_a/scan
ros2 topic echo /amr_a/odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

RViz2에서는 다음을 사용한다.

```text
Fixed Frame: map
Map: /map
LaserScan: /amr_a/scan
```

원본 self-hit를 비교하려면 별도 LaserScan display에 `/amr_a/scan_raw`을 추가한다.

## 위치 추정

AMCL 입력:

```text
scan_topic: /amr_a/scan
base_frame_id: base_link
odom_frame_id: odom
global_frame_id: map
```

초기 위치가 맞지 않으면 RViz2의 `2D Pose Estimate`를 사용한다.

정상 TF:

```text
map -> odom -> base_link
```

## 이동 테스트

Nav2가 active 상태인지 확인한다.

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

각 노드는 `active [3]`이어야 한다.

자동 테스트:

```bash
./scripts/test_iw_hub_avoidance.sh
```

또는 직접 목표를 보낸다.

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 6.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

## 현재 유지해야 할 부분

현재 정상 동작하는 아래 파일/구조는 불필요하게 수정하지 않는다.

- `isaac_sim/robots/iw_hub/iw_hub_agent.py`
- `isaac_sim/robots/iw_hub/iw_hub_v2.usda` embedded ActionGraph
- `/amr_a/odom` Isaac publisher
- `/clock` publisher

LiDAR self-hit 처리는 Isaac ActionGraph를 수정하지 않고 ROS 2 filter layer에서 해결한다.

## Warehouse 경로

`isaac_sim/main.py`는 repository 기준 상대 경로로 `World0.usd`를 찾는다. 따라서 사용자 홈 디렉터리가 달라도 같은 repository 구조라면 별도의 절대경로 수정이 필요 없다.
