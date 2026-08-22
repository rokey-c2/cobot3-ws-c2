# IW Hub 단일 AMR Nav2 자율주행

현재 `feat/iw-hub-single-navigation` 브랜치는 프로젝트 Warehouse USD에 NVIDIA Isaac Sim 5.1 공식 Navigation 샘플의 IW Hub 로봇 구성을 reference해서 사용한다.

공식 샘플 경로:

```text
/Isaac/Samples/ROS2/Scenario/iw_hub_warehouse_navigation.usd
```

## 원칙

프로젝트 코드에서 LiDAR 설정을 임의로 만들거나 수정하지 않는다.

다음 항목은 NVIDIA Navigation 샘플 값을 그대로 사용한다.

- front 2D LiDAR
- back 2D LiDAR
- LiDAR 위치와 방향
- LiDAR range/config
- ROS 2 LaserScan publisher
- `/cmd_vel`
- `/chassis/odom`

사용 토픽:

```text
/front_2d_lidar/scan
/back_2d_lidar/scan
/chassis/odom
/cmd_vel
```

프로젝트에서 변경하는 것은 Warehouse world와 AMR의 시작 위치뿐이다.

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

## 실행

터미널 1:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/run_isaac.sh
```

Isaac 실행 시 코드는 NVIDIA 공식 Navigation scene을 읽어서 `/cmd_vel`, `/chassis/odom`, front/back LaserScan을 모두 포함하는 IW Hub robot prim을 찾아 프로젝트 Warehouse에 reference한다.

터미널 2:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/run_ros2.sh
```

`run_ros2.sh`는 아래 토픽이 실제로 publish되는지 확인한다.

```text
/clock
/front_2d_lidar/scan
/back_2d_lidar/scan
/chassis/odom
```

그 다음 NVIDIA `iw_hub_navigation.launch.py`를 프로젝트 warehouse map으로 실행하고 AMCL 초기 위치를 출발 좌표로 설정한다.

터미널 3:

```bash
cd ~/collaboration/test/cobot3-ws-c2
./scripts/test_iw_hub_avoidance.sh
```

목표:

```text
(-13.813100814819336, 0.7454315423965454)
```

## 확인

```bash
ros2 topic list | grep -Ei "lidar|scan|odom|cmd_vel"
ros2 topic echo /front_2d_lidar/scan --once
ros2 topic echo /back_2d_lidar/scan --once
ros2 topic echo /chassis/odom --once
```

정상이라면 최소한 다음 토픽이 보여야 한다.

```text
/front_2d_lidar/scan
/back_2d_lidar/scan
/chassis/odom
/cmd_vel
```

## Warehouse 경로

`isaac_sim/main.py`는 repository 기준 상대 경로로 `World0.usd`를 찾는다.
