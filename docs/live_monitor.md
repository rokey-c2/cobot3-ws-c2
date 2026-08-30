# Control Tower Live Monitor

Dashboard의 기존 영상 영역은 3단 Live Monitor로 사용한다.

```text
TOP VIEW -> P3020 IN -> P3020 OUT -> TOP VIEW ...
```

AMR 전면 카메라는 Dashboard 슬라이드와 분리해서 기존 **AMR Control** 페이지의
`VISION / AMR Front Camera` 영역에 표시한다.

## 스트림 구조

| 화면 | ROS2 입력 | MJPEG | Web 위치 |
|---|---|---|---|
| AMR Front Camera | IW Hub `front_stereo_camera` 왼쪽 RGB | `:8090/stream.mjpg` | AMR Control |
| P3020 IN Laser Vision | `/rgb` | `:8091/stream.mjpg` | Dashboard |
| Warehouse Top View | `/top_view/rgb` | `:8092/stream.mjpg` | Dashboard |
| P3020 OUT Laser Vision | `/arm_b/rgb` | `:8093/stream.mjpg` | Dashboard |

P3020 IN/OUT은 같은 `box_detector_node.py`를 서로 다른 ROS2 파라미터로 실행한다.
따라서 두 P3020 모두 동일한 녹색 Laser HUD를 사용하면서 픽셀 결과는 각각
`/box_pixel`, `/arm_b/box_pixel`로 분리된다.

AMR 영상은 NVIDIA IW Hub에 원래 포함된 `front_stereo_camera`를 재사용한다.
별도 AMR Camera Prim을 추가하지 않아 로봇에 중복 렌더 카메라를 만들지 않는다.
`run_vision_streams.sh`는 Isaac 버전에 따라 달라질 수 있는 왼쪽 RGB 토픽을 다음
후보에서 자동 탐색한다.

```text
/front_stereo_camera/left/image_raw
/front_stereo_camera/left/image_rect_color
/front_stereo_camera/left_rgb/image_raw
```

필요하면 실행 전에 `AMR_CAMERA_TOPIC=/실제/토픽` 환경변수로 직접 지정할 수 있다.

## 실행 순서

### 터미널 1 - Isaac Sim

```bash
cd ~/collaboration/cobot3-ws-c2
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_isaac_mission.sh
```

`run_isaac_mission.sh`는 기존 `main_mission.py`를 직접 수정하지 않고
`main_mission_live_view.py` 래퍼를 실행한다. 래퍼는 두 가지 역할만 추가한다.

1. `/World/ControlTowerTopViewCamera` 고정 카메라를 추가하고 `/top_view/rgb`를 약 10 Hz로 발행
2. 기존 성능 최적화에서 꺼졌던 IW Hub `front_stereo_camera`를 관제용으로 활성 상태 유지

나머지 AMR/P3020/컨베이어/소터 미션 로직은 기존 `main_mission.py`를 그대로 사용한다.

### 터미널 2 - AMR + Top View + P3020 IN/OUT 영상 서버

Isaac이 준비된 뒤:

```bash
cd ~/collaboration/cobot3-ws-c2
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_vision_streams.sh
```

스크립트는 네 프로세스를 같이 실행한다.

```text
8090  AMR front camera
8091  P3020 IN laser detection
8092  Warehouse top view
8093  P3020 OUT laser detection
```

`onnxruntime`가 설치된 프로젝트 `.venv`가 있으면 자동 사용한다. 다른 Python을
사용하려면 `VISION_PYTHON=/path/to/python`을 지정한다.

정상 시작 시 AMR 카메라에 대해 다음 로그가 먼저 보인다.

```text
[VISION] AMR camera topic: /front_stereo_camera/left/...
[VISION] starting AMR front camera :8090
```

### 터미널 3 - Frontend

```bash
cd ~/collaboration/cobot3-ws-c2/frontend
npm run dev
```

- Dashboard: `TOP VIEW`, `P3020 IN`, `P3020 OUT` 탭 또는 좌우 화살표로 전환
- AMR Control: IW Hub 전면 카메라가 자동으로 `:8090`에 연결되고 상태를 `CONNECTING / LIVE / OFFLINE`으로 표시

활성 Dashboard 슬라이드 하나만 MJPEG 연결을 유지해 불필요한 브라우저 스트림 부하를 줄인다.
AMR Control 페이지는 해당 페이지를 열었을 때만 AMR MJPEG 연결이 생성된다.

## 다른 PC에서 Frontend를 실행하는 경우

영상 서버가 Frontend PC와 다른 PC에서 실행되면 `frontend/.env`에 실제
Vision/ROS PC 주소를 지정한다.

```env
VITE_AMR_CAMERA_STREAM_URL=http://10.10.0.2:8090/stream.mjpg
VITE_TOP_VIEW_STREAM_URL=http://10.10.0.2:8092/stream.mjpg
VITE_P3020_IN_CAMERA_STREAM_URL=http://10.10.0.2:8091/stream.mjpg
VITE_P3020_OUT_CAMERA_STREAM_URL=http://10.10.0.2:8093/stream.mjpg
```

같은 PC에서 Frontend와 영상 서버를 실행하면 위 환경변수들은 비워 둬도 된다.
