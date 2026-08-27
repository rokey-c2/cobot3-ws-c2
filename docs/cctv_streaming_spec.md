# CCTV 카메라 스트리밍 구현 스펙

작성일: 2026-08-27 · 브랜치: `hwi_video_concept` (base: `new_scenario`)

**이 문서를 읽는 에이전트에게**: 이 작업은 **Isaac Sim/ROS2 쪽**과 **웹(프론트엔드) 쪽**을
서로 다른 컴퓨터/다른 세션에서 나눠 작업합니다. 서로 실시간으로 대화하며 맞출 수 없으므로,
**2장(인터페이스 계약)에 적힌 값은 절대 임의로 바꾸지 말고 그대로 구현하세요.** 바꿔야 할
이유가 생기면 이 문서 자체를 먼저 수정하고, 상대 쪽도 그 수정을 반영해야 합니다.

**당신이 어느 쪽 작업을 맡았는지에 따라 읽을 부분이 다릅니다** (이 파일 하나를 양쪽에
똑같이 전달합니다 — 파일이 다르면 계약이 어긋날 수 있어서 일부러 하나로 유지합니다):

- **Isaac Sim / ROS2 담당이면**: 0장 → 1장 → 2장(계약) → **3장**을 구현하세요. 4장(웹
  쪽 할 일)은 참고만 하면 됩니다.
- **웹(프론트엔드) 담당이면**: 0장 → 1장 → 2장(계약) → **4장**을 구현하세요. 3장(Isaac
  Sim 쪽 할 일)은 참고만 하면 됩니다.
- **어느 쪽이든 공통**: 위 "시작 전 필수 확인 사항"(브랜치, 환경변수)과 5장(통합 시 최종
  확인)은 반드시 읽으세요.

**시작 전 필수 확인 사항 (양쪽 공통)**:
1. **브랜치**: 이 문서는 `hwi_video_concept` 브랜치(base: `new_scenario`) 기준으로
   작성됐습니다. 이 문서가 참조하는 파일 경로/클래스/코드가 **다른 브랜치에는 없거나
   다를 수 있습니다** (예: `26th-Aug` 브랜치는 소터 로직이 완전히 다름). 작업 전
   `git checkout hwi_video_concept` (로컬에 없으면 `git fetch origin hwi_video_concept`
   먼저)로 이 브랜치인지 확인하세요.
2. **환경변수**: 이 프로젝트는 `ROS_DOMAIN_ID=111`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를
   모든 터미널에서 동일하게 맞춰야 서로 통신됩니다 (README에 명시된 필수 설정). 이 값이
   안 맞으면 Isaac Sim은 정상 발행하는데 `ros2 topic list`/`web_video_server`에는
   토픽이 안 보이는, **가장 헷갈리기 쉬운 실패 모드**가 생깁니다. 새 터미널을 열 때마다
   ```bash
   export ROS_DOMAIN_ID=111
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```
   를 먼저 실행하고, `echo $ROS_DOMAIN_ID`로 111인지 확인하세요.

---

## 0. 배경 (왜 이 작업을 하는가)

데모 영상(`docs/video_shoot_guide.md`)에서 "CCTV 하드컷 → 줌아웃하면 사실 웹 관제 UI
안의 CCTV 패널이었다"는 리빌 연출을 씁니다. 이걸 진짜 기능으로 만들려는 것 — Isaac Sim
안에 고정 카메라 4대를 배치해서 각각 ROS2 토픽으로 영상을 발행하고, 그걸 웹 대시보드에서
실시간으로 받아서 보여줍니다.

- Isaac Sim 카메라 → ROS2 Image 토픽 → MJPEG 브리지 → 브라우저 `<img>` 태그
- 참고로 이미 `frontend/src/pages/AmrControlPage.jsx`에 카메라 패널 1개가 미리 설계돼
  있었지만(`VITE_AMR_CAMERA_STREAM_URL`), 실제로 그 스트림을 쏴주는 서버는 없는
  상태였습니다. 이번 작업은 그걸 **4대짜리로 확장**해서 실제로 채우는 것입니다.

---

## 1. 카메라 배치 (4대, 확정)

| 토픽 이름 | 위치 / 촬영 대상 | UI 라벨 |
|---|---|---|
| `/cctv_0/image_raw` | **Top View** — 맵 전체를 위에서 내려다보는 조감 카메라 | `TOP VIEW` |
| `/cctv_1/image_raw` | AMR 구동 구간 (AMR이 팔레트/박스 나르며 이동하는 경로) | `AMR` |
| `/cctv_2/image_raw` | 로봇팔(P3020) 구동 구간 (Pick & Place 지점) | `ARM` |
| `/cctv_3/image_raw` | 휠 소터 구동 구간 (분류 트랙) | `SORTER` |

이 번호(0~3)와 이름은 **양쪽(Isaac Sim / 웹)이 그대로 공유하는 식별자**입니다. 절대 다른
이름으로 바꾸지 마세요.

---

## 2. 인터페이스 계약 (양쪽이 반드시 지켜야 하는 값)

이 표가 이 문서에서 가장 중요한 부분입니다. Isaac Sim 쪽과 웹 쪽이 서로 마주치는 지점이
이 값들뿐이라, 여기서 어긋나면 통합할 때 안 됩니다.

| 항목 | 값 | 비고 |
|---|---|---|
| ROS2 토픽 이름 | `/cctv_0/image_raw` ~ `/cctv_3/image_raw` | 4개 고정 |
| ROS2 메시지 타입 | `sensor_msgs/msg/Image` | 표준 타입, 커스텀 메시지 아님 |
| 이미지 인코딩 | `rgb8` | `Image.encoding = "rgb8"`로 발행 |
| 해상도 | 640×480 | 4대 동시 렌더 시 성능 부담 고려해서 키우지 않음 (3장 성능 주의 참고) |
| 발행 주기 | 매 시뮬레이션 스텝이 아니라 **N스텝마다 1번** (권장 6스텝마다, ~10Hz 내외) | Isaac Sim 쪽 책임. 웹 쪽은 MJPEG라 프레임레이트를 신경 쓸 필요 없음 (오는 대로 표시) |
| ROS2→MJPEG 브리지 | `web_video_server` ROS2 패키지, 기본 포트 `8080` | 살아있는 모든 image 토픽을 자동으로 서빙함 |
| MJPEG URL 패턴 | `http://<HOST>:8080/stream?topic=/cctv_N/image_raw` | `<HOST>`는 Isaac Sim/ROS2가 실행되는 머신의 IP 또는 호스트명 |
| 웹 쪽 환경변수 | `VITE_WEB_VIDEO_SERVER_BASE=http://<HOST>:8080/stream?topic=` | 이 값 하나로 4개 URL을 조립 (`${BASE}/cctv_0/image_raw` 등) |

**`<HOST>` 관련 주의 (미정 항목)**: Isaac Sim이 도는 컴퓨터와 웹 브라우저가 같은
컴퓨터인지 다른 컴퓨터인지에 따라 `<HOST>` 값이 `localhost`일 수도, 실제 IP(예:
`192.168.x.x`)일 수도 있습니다. **이 값은 이 문서에서 확정하지 않습니다** — 실제 촬영
환경(Isaac Sim 머신과 웹 브라우저를 띄우는 머신)이 정해지면 그때 `.env`의
`VITE_WEB_VIDEO_SERVER_BASE`만 바꿔서 맞추면 됩니다. **웹 쪽 코드에 이 호스트를
하드코딩하지 마세요** — 반드시 환경변수로만 읽어야 나중에 값을 바꿔도 코드 수정이
필요 없습니다.

> **참고 — 기존 `.env.example`과의 차이**: `frontend/.env.example`에 이미
> `VITE_AMR_CAMERA_STREAM_URL` 변수와 예시 주석(`http://localhost:8090/camera/amr_in`,
> 포트 8090 + `/camera/<name>` 경로)이 있습니다. 이 문서는 `web_video_server`(포트
> 8080 기본값, `/stream?topic=...` 경로)를 기준으로 확정했습니다. `.env.example`의
> 주석/변수명은 이 스펙에 맞게 갱신해도 됩니다 (`VITE_AMR_CAMERA_STREAM_URL` 단일 변수 →
> `VITE_WEB_VIDEO_SERVER_BASE`로 교체).
>
> **포트 8080 확정 (2026-08-27 재확인)**: 프로젝트에서 실제 쓰는 포트를 전수 확인함 —
> 프론트엔드 5173, 백엔드 8000, MQTT 1883, PostgreSQL 5432. 8080·8090 둘 다 어디에도
> 쓰이지 않아 충돌 없음. `web_video_server` 기본값인 **8080으로 확정**, 더 고민할
> 필요 없음. `ros-jazzy-web-video-server` apt 패키지도 후보 버전(3.1.0)이 확인되어
> 설치 가능함.

---

## 3. Isaac Sim / ROS2 쪽 할 일

### 3-1. 카메라 프림 추가 (코드로 스폰하지 않음 — 확정)
- 대상 파일: `isaac_sim/usd/Parcel_Sorting_Map/Parcel_Sorting_Map.usd`
  (`isaac_sim/main_mission.py`의 `WORLD_USD`가 여는 파일).
- **4개 카메라는 코드로 런타임에 스폰하지 않고, Isaac Sim GUI에서 사람이 직접
  위치를 잡아 USD 파일에 저장합니다** (`Create > Camera`로 1장의 표에 나온 4개 위치에
  배치 후 USD 저장). 이유:
  - 전부 **고정 카메라**라 런타임 스폰의 이점이 없음.
  - 기존 P3020 카메라도 "USD에 이미 고정된 프림 transform을 그대로 읽기만" 하는
    방식(`isaac_sim/robots/p3020/vision/camera.py` 주석 참고) — 같은 관례를 따름.
  - 런타임 스폰은 이 프로젝트에서 자주 겪은 "OmniGraph/USD 속성이 world.reset() 전엔
    아직 준비 안 됨" 류의 레이스 컨디션을 새로 만들 뿐, 얻는 게 없음.
  - `cctv_0`(Top View)만 위에서 아래로 수직으로 내려다보는 각도, 나머지 3개는 각
    구간이 잘 보이는 낮은 각도로 고정.
- prim 이름은 자유롭게 지어도 되지만(`/World/Cctv0` 등), **ROS2 토픽 이름은 반드시
  2장 표의 이름으로 발행**해야 합니다. prim 이름과 토픽 이름이 같을 필요는 없습니다.
- **Python 코드가 할 일은 이미 USD에 있는 prim을 `Camera(prim_path=...)`로 참조해서
  프레임을 읽고 발행하는 것뿐**입니다 (아래 3-2). 카메라 생성/스폰 관련 코드는
  작성하지 않습니다.
- 카메라 추가 후 **`Parcel_Sorting_Map.usd` 파일 자체에 저장**하세요 (Isaac Sim에서
  다른 레이어/서브레이어로 저장하면 `main_mission.py`의 `open_stage(str(WORLD_USD))`가
  그 변경을 못 찾을 수 있습니다). 저장 후 `run_isaac_mission.sh`로 다시 띄워서 카메라가
  씬에 남아있는지 재확인하세요.

### 3-2. Python 퍼블리셔

> **⚠️ 반드시 읽을 것 — Isaac Sim rclpy 제약**: `isaac_sim/main_mission.py`의
> `ProcessEquipmentBridge` 클래스 docstring에 이렇게 적혀 있습니다:
> > "Attaches to a caller-supplied node instead of being its own Node --
> > Isaac Sim's bundled rclpy only bridges **the first Node created after
> > rclpy.init()** to the outside world, so a second Node's subscriptions
> > ... never receive anything even though discovery/matching looks fine."
>
> 즉 Isaac Sim 프로세스 안에서는 `rclpy.init()` 이후 **처음 생성한 Node 하나만** 실제
> ROS2 그래프에 연결됩니다. 그래서 이 파일의 모든 ROS2 클래스(`OptimizedP3020RosBridge`,
> `P3020OutRosBridge`, `ProcessEquipmentBridge`)는 **절대 자기 `Node()`를 새로 만들지
> 않고**, 공유 Node를 인자로 받아 그 위에 publisher를 붙입니다:
> ```python
> bridge = AmrMissionBridge(agents[0])   # 유일하게 진짜 rclpy.node.Node()로 생성되는 것
> equipment_bridge = ProcessEquipmentBridge(bridge, conveyor, sorter)
> ```
> **CCTV 퍼블리셔도 반드시 이 패턴을 따라야 합니다.** 새 `rclpy.node.Node()`를 만들면
> 토픽이 떠 있는 것처럼 보여도 외부 ROS2 그래프에 연결되지 않을 수 있습니다.

`isaac_sim/robots/p3020/vision/camera.py`의 `CameraInterface`(카메라 자체를 감싸는 부분)와
`main_mission.py`의 `ProcessEquipmentBridge`(공유 Node에 붙는 부분) 패턴을 합쳐서
씁니다:

```python
import numpy as np
from isaacsim.sensors.camera import Camera
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def rgb_to_imgmsg(rgb, header: Header) -> Image:
    """RGB(numpy) -> sensor_msgs/Image. 이 브랜치엔 반대 방향인 imgmsg_to_rgb만
    box_detector_node.py에 있어서 새로 작성이 필요함.

    주의: header는 반드시 std_msgs/Header 인스턴스여야 함. node.get_clock().now().to_msg()는
    builtin_interfaces/Time이라 타입이 다름 -- 아래 publish_all()처럼 Header()를 만들어
    .stamp에 넣어서 넘겨야 함."""
    out = Image()
    out.header = header
    out.height, out.width = rgb.shape[:2]
    out.encoding = "rgb8"
    out.is_bigendian = 0
    out.step = out.width * 3
    out.data = np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()
    return out


class CctvCameraBridge:
    """ProcessEquipmentBridge와 동일하게, 자기 Node를 만들지 않고 공유 Node(bridge)에
    publisher를 붙인다."""

    def __init__(self, node, cameras: dict):
        self._node = node
        self._cameras = cameras  # {"cctv_0": Camera(...), ...}
        self._pubs = {
            name: node.create_publisher(Image, f"/{name}/image_raw", 10)
            for name in cameras
        }
        for cam in cameras.values():
            cam.initialize()

    def publish_all(self):
        for name, cam in self._cameras.items():
            rgba = cam.get_rgba()
            if rgba is None or rgba.size == 0:
                continue
            rgb = rgba[:, :, :3]
            header = Header()
            header.stamp = self._node.get_clock().now().to_msg()
            header.frame_id = name
            msg = rgb_to_imgmsg(rgb, header)
            self._pubs[name].publish(msg)
```

`main_mission.py`에서 다른 브리지들과 같은 자리에서 공유 `bridge`를 넘겨 인스턴스화합니다.
**정확한 줄 번호 대신 아래 패턴을 코드에서 검색해서 찾으세요** (줄 번호는 코드가 조금만
바뀌어도 틀어짐) — `equipment_bridge = ProcessEquipmentBridge(bridge, conveyor, sorter)`
라인 바로 다음에 추가:

```python
cctv_cameras = {
    "cctv_0": Camera(prim_path="/World/Cctv0", resolution=(640, 480)),
    "cctv_1": Camera(prim_path="/World/Cctv1", resolution=(640, 480)),
    "cctv_2": Camera(prim_path="/World/Cctv2", resolution=(640, 480)),
    "cctv_3": Camera(prim_path="/World/Cctv3", resolution=(640, 480)),
}
cctv_bridge = CctvCameraBridge(bridge, cctv_cameras)   # bridge = 기존 공유 Node
```

시뮬레이션 루프 안에서 N스텝마다(아래 표 참고) `cctv_bridge.publish_all()`을 호출합니다.
기존 코드의 `VISION_RGB_PUBLISH_INTERVAL_STEPS = 6`과 동일한 값(6스텝마다, ~10Hz)을
그대로 재사용하는 걸 권장합니다 — 이미 검증된 값입니다.

### 3-3. 성능 주의 (중요)
- 이 프로젝트는 로봇을 1대→3대로 늘렸을 때 스폰 시점부터 느려지는 성능 문제를 조사한
  이력이 있습니다 (커밋되지 않은 조사 문서였어서 지금 저장소엔 파일 자체가 남아있지
  않음 — 필요하면 담당자에게 직접 물어보세요). **카메라 4대를 동시에 매 스텝
  렌더/발행하면 같은 종류의 부하가 생깁니다.**
- 반드시 발행 주기를 낮추세요 (예: `if step % 6 == 0:` 같은 패턴, P3020 카메라 코드가
  이미 이렇게 하고 있으니 그대로 참고).
- 카메라를 하나씩 추가하면서 시뮬레이션 프레임레이트가 눈에 띄게 떨어지지 않는지
  확인하고, 떨어지면 해상도를 더 낮추거나 발행 주기를 더 늦추세요.

### 3-4. ROS2 → MJPEG 브리지
```bash
sudo apt install ros-jazzy-web-video-server   # 없으면 설치
ros2 run web_video_server web_video_server     # 기본 8080 포트
```
- 이 노드 하나가 살아있는 모든 image 토픽을 자동으로 서빙합니다. 카메라를 더 추가해도
  이 노드는 재설정 없이 그대로 대응합니다.
- **프론트엔드 연결 전에 브라우저로 직접 검증**: `http://<HOST>:8080/stream?topic=/cctv_0/image_raw`
  같은 URL을 브라우저 주소창에 직접 쳐서 화면이 나오는지 먼저 확인하세요.
- **URL 형식이 안 맞으면**: 이 문서의 `/stream?topic=...` 패턴은 `web_video_server`의
  일반적인 사용법을 기준으로 적은 것이고, 실제 설치해서 검증한 값은 아닙니다. 안 되면
  브라우저로 `http://<HOST>:8080/`(포트만, 경로 없이) 접속해보세요 — `web_video_server`는
  기본적으로 현재 살아있는 모든 image 토픽 목록을 index 페이지에 클릭 가능한 링크로
  보여주므로, 거기서 정확한 URL을 바로 확인할 수 있습니다.

### 3-5. Isaac Sim 쪽 검증 체크리스트

시뮬레이션 실행은 `./scripts/run_isaac_mission.sh`로 합니다 (ROS_DOMAIN_ID/RMW_IMPLEMENTATION
설정 후 — 위 "시작 전 필수 확인 사항" 참고). README의 "전체 통합 미션" 절차와 동일합니다.

- [ ] Isaac Sim에서 카메라 4개 프림이 씬에 제대로 배치되어 보이는가
- [ ] `ros2 topic list`에 `/cctv_0/image_raw` ~ `/cctv_3/image_raw` 4개가 다 뜨는가
- [ ] `ros2 topic hz /cctv_0/image_raw` (나머지도 동일)로 프레임이 실제로 오는가
- [ ] `web_video_server` 실행 후 브라우저에서 4개 URL 모두 직접 접속해 화면이 나오는가
- [ ] 카메라 4대를 켠 상태에서 시뮬레이션 프레임레이트가 크게 떨어지지 않는가

---

## 4. 웹(프론트엔드) 쪽 할 일

### 4-1. 참고할 기존 코드
- `frontend/src/pages/AmrControlPage.jsx` — 카메라 패널 1개(`camera-frame`,
  `camera-placeholder`, `camera-footer` 마크업)가 이미 있습니다. 이 마크업/스타일을
  그대로 재사용하세요.
- `frontend/src/styles.css`의 `.camera-frame`, `.camera-placeholder`,
  `.camera-reticle`, `.camera-footer` 클래스.
- 스트림이 없을 때 보여주는 "STREAM WAITING" / "NOT CONFIGURED" 플레이스홀더 로직도
  그대로 재사용.

### 4-2. 만들 것

> **어디에 넣을지는 이쪽 판단에 맡김** — 새 페이지로 만들지, 기존 페이지에 추가할지는
> 정해두지 않았습니다. 다만 이 앱은 `react-router-dom` 기반이고, 라우트는
> `frontend/src/App.jsx`(`<Routes>`), 사이드바 메뉴는
> `frontend/src/components/Layout.jsx`(`navItems` 배열)에서 관리합니다. 새 페이지를
> 만든다면 이 두 파일도 같이 고쳐야 실제로 화면에 나타나고, 기존 경로(`/`, `/packages`,
> `/packages/:packageCode`, `/amr`)와 겹치지 않게 하세요.

- 카메라 4개를 그리드(2×2 등)로 보여주는 새 컴포넌트 (또는 기존 페이지에 패널 추가).
- `.env`에 아래 변수 하나만 추가:
  ```
  VITE_WEB_VIDEO_SERVER_BASE=http://<HOST>:8080/stream?topic=
  ```
- URL은 코드에서 조립:
  ```js
  const base = import.meta.env.VITE_WEB_VIDEO_SERVER_BASE || "";
  const cctvUrl = (n) => (base ? `${base}/cctv_${n}/image_raw` : "");
  ```
- 4개 패널 각각에 2장 표의 UI 라벨(`TOP VIEW` / `AMR` / `ARM` / `SORTER`)을 붙임.
- `<img src={cctvUrl(0)} />` 형태로 MJPEG 스트림을 그대로 `<img>` 태그에 연결 (MJPEG는
  일반 `<img>` 태그로 재생되는 포맷이라 별도 비디오 플레이어 라이브러리 불필요).

### 4-3. 웹 쪽 검증 체크리스트
- [ ] `.env`에 `VITE_WEB_VIDEO_SERVER_BASE` 설정 후 4개 패널이 각각 스트림을 표시하는가
- [ ] 스트림 서버가 꺼져 있을 때 깨진 이미지 아이콘이 아니라 "STREAM WAITING"
      플레이스홀더가 정상적으로 뜨는가
- [ ] 4개 패널이 레이아웃 안에서 겹치거나 깨지지 않는가

---

## 5. 통합 시 최종 확인

Isaac Sim 쪽과 웹 쪽이 각자 작업을 끝낸 뒤, 같은 네트워크에서 실제로 합쳐볼 때:

1. `<HOST>` 값을 실제 환경에 맞게 양쪽에서 일치시킨다 (Isaac Sim 머신의 실제 IP/호스트명).
2. 웹 쪽 `.env`의 `VITE_WEB_VIDEO_SERVER_BASE`만 그 값으로 바꾼다 (코드 수정 없음).
3. 브라우저에서 4개 CCTV 패널이 모두 실시간으로 뜨는지 확인한다.
4. 시뮬레이션에서 AMR/로봇팔/소터가 동작할 때 해당 패널 화면이 실시간으로 같이
   움직이는지 확인한다 (이게 확인되면 데모 영상의 "리빌" 연출이 실제 기능으로 완성됨).
