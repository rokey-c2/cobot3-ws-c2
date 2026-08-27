# Control Tower Live Monitor

Dashboard의 기존 영상 영역은 3단 Live Monitor로 사용한다.

```text
TOP VIEW -> P3020 IN -> P3020 OUT -> TOP VIEW ...
```

AMR 영상은 기존 **AMR Control** 페이지의 `VISION / AMR Front Camera` 영역에 표시한다.

## 최적화된 스트림 구조

| 화면 | ROS2 입력 | 기본 처리율 | MJPEG |
|---|---|---:|---|
| AMR Camera | `/amr_a/camera/rgb` | 640x360, 약 6.7 Hz | `:8090/stream.mjpg` |
| P3020 IN Laser Vision | `/rgb` | YOLO 최대 5 Hz | `:8091/stream.mjpg` |
| Warehouse Top View | `/top_view/rgb` | 960x540, 약 5 Hz | `:8092/stream.mjpg` |
| P3020 OUT Laser Vision | `/arm_b/rgb` | YOLO 최대 5 Hz | `:8093/stream.mjpg` |

P3020 IN/OUT은 같은 `box_detector_node.py`를 서로 다른 토픽으로 실행한다.
픽셀 결과는 각각 `/box_pixel`, `/arm_b/box_pixel`로 분리된다.

## 성능 최적화 내용

### AMR

기존 NVIDIA IW Hub의 `front_stereo_camera` 전체 리그를 웹 영상 때문에 다시 켜지 않는다.
기존 `iw_hub_agent.py`의 카메라 비활성화 최적화는 그대로 유지한다.

대신 `main_mission_live_view.py`가 AMR 위치/방향을 따라가는 관제 전용 단일 카메라를 만든다.

```text
/World/ControlTowerAmrCamera
  -> 640x360
  -> /amr_a/camera/rgb
  -> MJPEG :8090
```

스테레오/깊이 카메라 전체 리그보다 렌더 부하가 작다.

### Warehouse Top View

기존 1280x720 약 10 Hz에서 다음으로 낮췄다.

```text
960x540
약 5 Hz
JPEG quality 65
```

관제 화면 확인에는 충분하면서 RTX 렌더/ROS 전송/JPEG 압축량을 줄인다.

### P3020 객체 탐지

기존에는 들어오는 이미지 callback 안에서 매 프레임 ONNX 추론을 실행했다.
현재는 **최신 프레임 하나만 보관**하고 worker thread가 최대 5 Hz로 추론한다.

따라서 YOLO가 느려져도 예전 프레임이 ROS callback queue에 계속 쌓이지 않는다.

```text
camera frames
  -> latest frame only
  -> YOLO <= 5 Hz
  -> /box_pixel or /arm_b/box_pixel
  -> Laser HUD MJPEG <= 6 Hz
```

추가 최적화:

- ROS Image subscription depth = 1
- P3020 annotated ROS Image는 기본 비활성화
- JPEG quality 기본 65
- ONNX Runtime CPU thread 기본 2개/process
- `CUDAExecutionProvider`가 설치돼 있으면 자동 사용
- P3020 IN RGB source 자체도 약 5 Hz로 제한
- P3020 OUT depth는 ROS로 보내지 않고 Isaac 내부 pixel->world 계산에만 사용
- P3020 OUT 스캔 카메라 sampling도 약 5 Hz로 제한

## 실행

### 터미널 1 - Isaac Sim

```bash
cd ~/collaboration/cobot3-ws-c2
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_isaac_mission.sh
```

`main_mission.py`는 직접 수정하지 않는다.
`main_mission_live_view.py` wrapper에서 관제 카메라와 vision rate 제한만 적용한다.

### 터미널 2 - 영상 + 객체 탐지

Isaac이 준비된 뒤:

```bash
cd ~/collaboration/cobot3-ws-c2
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_vision_streams.sh
```

한 번에 다음 네 프로세스를 실행한다.

```text
8090  AMR camera
8091  P3020 IN laser detection
8092  Warehouse top view
8093  P3020 OUT laser detection
```

정상 로그 예:

```text
[VISION] perf: AMR=6.0fps TOP=5.0fps P3020 detect=5.0fps JPEG=65
```

각 P3020 detector 로그에서 ONNX provider도 확인할 수 있다.

```text
vision perf: provider=CPUExecutionProvider, ...
```

또는 `onnxruntime-gpu` 환경이면:

```text
vision perf: provider=CUDAExecutionProvider, ...
```

## 더 가볍게 돌리고 싶을 때

코드 수정 없이 실행 전에 환경변수만 낮출 수 있다.

```bash
export AMR_STREAM_FPS=4
export TOP_VIEW_STREAM_FPS=4
export P3020_DETECTION_FPS=4
export P3020_STREAM_FPS=5
export VISION_JPEG_QUALITY=60
export VISION_CPU_THREADS=2

./scripts/run_vision_streams.sh
```

객체 탐지 반응성을 우선하면 `P3020_DETECTION_FPS=5`를 유지하는 것을 권장한다.

## Frontend

```bash
cd ~/collaboration/cobot3-ws-c2/frontend
npm run dev
```

- Dashboard: `TOP VIEW`, `P3020 IN`, `P3020 OUT`
- AMR Control: AMR 추적 카메라
- Dashboard는 활성 슬라이드 하나만 MJPEG 연결
- AMR 페이지도 해당 페이지를 열었을 때만 AMR MJPEG 연결

## 다른 PC에서 Frontend를 실행하는 경우

```env
VITE_AMR_CAMERA_STREAM_URL=http://10.10.0.2:8090/stream.mjpg
VITE_TOP_VIEW_STREAM_URL=http://10.10.0.2:8092/stream.mjpg
VITE_P3020_IN_CAMERA_STREAM_URL=http://10.10.0.2:8091/stream.mjpg
VITE_P3020_OUT_CAMERA_STREAM_URL=http://10.10.0.2:8093/stream.mjpg
```

같은 PC면 비워 둬도 현재 Web hostname의 8090~8093 포트를 자동 사용한다.
