# IW Hub 단일 AMR Nav2 자율주행

현재 `feat/iw-hub-single-navigation` 브랜치는 저장된 Warehouse USD에서 Isaac Sim 기본 IW Hub Sensor 에셋 한 대를 실행하고 ROS 2 Jazzy Nav2와 연결한다.

## 원칙

IW Hub Sensor 에셋에 포함된 센서와 ROS graph를 그대로 사용한다.

프로젝트 코드에서 다음 작업을 하지 않는다.

- 별도 RTX LiDAR 생성
- LiDAR near range 변경
- LaserScan self-filter 추가
- IW Hub 기본 odometry frame/topic 변경
- 별도 odom TF bridge 추가

NVIDIA 기본 IW Hub Navigation 설정에서 사용하는 토픽을 그대로 사용한다.

```text
/front_2d_lidar/scan
/back_2d_lidar/scan
/chassis/odom
/cmd_vel
```

## 좌표

출발 위치:

```text
x = 8.155903816223145
y = -5.628969192504883
z = 0.0
yaw = 0.0
```

목표 위치:

```text
x = -13.813100814819336
y = 0.7454315423965454
z = 0.0
```

추가 테스트 장애물과 cargo는 생성하지 않는다.

## 데이터 흐름

```text
Isaac Sim default IW Hub Sensor
    |
    +-- /front_2d_lidar/scan
    +-- /back_2d_lidar/scan
    +-- /chassis/odom
    +-- /tf
    |
    v
NVIDIA iw_hub_navigation
    |
    +-- AMCL
    +-- local/global costmap
    +-- planner/controller
    +-- collision monitor
    |
    v
/cmd_vel
    |
    v
IW Hub
```

## 실행

터미널 1:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/run_isaac.sh
```

터미널 2:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/run_ros2.sh
```

`run_ros2.sh`는 아래 기본 IW Hub 토픽이 실제로 publish되는지 확인한다.

```text
/clock
/front_2d_lidar/scan
/back_2d_lidar/scan
/chassis/odom
```

그 다음 NVIDIA `iw_hub_navigation.launch.py`를 프로젝트 warehouse map으로 실행하고 AMCL의 `/initialpose`를 출발 좌표로 설정한다.

## 주행 테스트

터미널 3:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/test_iw_hub_avoidance.sh
```

이 스크립트는 `/navigate_to_pose`에 다음 목표를 전송한다.

```text
(-13.813100814819336, 0.7454315423965454)
```

## 확인 명령

```bash
ros2 topic info /front_2d_lidar/scan -v
ros2 topic info /back_2d_lidar/scan -v
ros2 topic echo /chassis/odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

RViz2에서는 기본 IW Hub Navigation 설정을 그대로 사용한다.

## Warehouse 경로

`isaac_sim/main.py`는 repository 기준 상대 경로로 `World0.usd`를 찾는다. 사용자 홈 디렉터리가 달라도 동일한 repository 구조라면 `main.py`의 절대경로를 수정할 필요가 없다.
