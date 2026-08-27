# Control Tower Live Monitor

Dashboard의 기존 영상 영역을 3단 Live Monitor로 사용한다.

```text
TOP VIEW -> P3020 IN -> P3020 OUT -> TOP VIEW ...
```

## 스트림 구조

| 화면 | ROS2 입력 | MJPEG |
|---|---|---|
| Warehouse Top View | `/top_view/rgb` | `:8092/stream.mjpg` |
| P3020 IN Laser Vision | `/rgb` | `:8091/stream.mjpg` |
| P3020 OUT Laser Vision | `/arm_b/rgb` | `:8093/stream.mjpg` |

P3020 IN/OUT은 같은 `box_detector_node.py`를 서로 다른 ROS2 파라미터로 실행한다.
따라서 두 P3020 모두 동일한 녹색 Laser HUD를 사용하면서 픽셀 결과는 각각
`/box_pixel`, `/arm_b/box_pixel`로 분리된다.

## 실행 순서

### 터미널 1 - Isaac Sim

```bash
cd ~/collaboration/cobot3-ws-c2
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_isaac_mission.sh
```

`run_isaac_mission.sh`는 기존 `main_mission.py`를 직접 수정하지 않고
`main_mission_live_view.py` 래퍼를 실행한다. 래퍼가 `/World/ControlTowerTopViewCamera`
고정 카메라를 추가하고 `/top_view/rgb`를 약 10 Hz로 발행한 뒤 기존 미션을 그대로 실행한다.

### 터미널 2 - Top View + P3020 IN/OUT 영상 서버

Isaac이 준비된 뒤:

```bash
cd ~/collaboration/cobot3-ws-c2
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_vision_streams.sh
```

스크립트는 세 프로세스를 같이 실행한다.

```text
8091  P3020 IN laser detection
8092  Warehouse top view
8093  P3020 OUT laser detection
```

`onnxruntime`가 설치된 프로젝트 `.venv`가 있으면 자동 사용한다. 다른 Python을
사용하려면 `VISION_PYTHON=/path/to/python`을 지정한다.

### 터미널 3 - Frontend

```bash
cd ~/collaboration/cobot3-ws-c2/frontend
npm run dev
```

브라우저 Dashboard의 `TOP VIEW`, `P3020 IN`, `P3020 OUT` 탭 또는 좌우 화살표로
영상을 전환한다. 활성 슬라이드 하나만 MJPEG 연결을 유지해 불필요한 브라우저
스트림 부하를 줄인다.

## 다른 PC에서 Frontend를 실행하는 경우

세 영상 서버가 Frontend PC와 다른 PC에서 실행되면 `frontend/.env`에 실제
Vision/ROS PC 주소를 지정한다.

```env
VITE_TOP_VIEW_STREAM_URL=http://10.10.0.2:8092/stream.mjpg
VITE_P3020_IN_CAMERA_STREAM_URL=http://10.10.0.2:8091/stream.mjpg
VITE_P3020_OUT_CAMERA_STREAM_URL=http://10.10.0.2:8093/stream.mjpg
```
