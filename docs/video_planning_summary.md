# 영상 기획용 프로젝트 요약

작성일: 2026-08-26 (데모 마감: 2026-08-28, D-2)
기준 브랜치: `26th-Aug` (원격 최신, `main`보다 앞선 상태 — 아래 "브랜치 상태" 참고)
작성 목적: 발표/데모 영상 기획을 위해 현재까지의 개발 현황을 정리. Windows 환경에서 영상 기획 작업 시 참고용.

---

## 0. 프로젝트 한 줄 요약

**AMR–협동로봇(Doosan P3020) 연계 택배 분류 자동화 및 실시간 관제 시스템**

```text
Input Zone → IW Hub AMR → Doosan P3020 Pick → Main Conveyor
→ Wheel Sorter → A/B Conveyor → P3020 A/B 적재 → Box Full
→ 출고 IW Hub → 배송지
```

Isaac Sim(디지털 트윈) 위에서 물류 로봇 전체 흐름을 시뮬레이션하고,
웹 기반 Control Tower로 실시간 관제/수동 제어까지 붙인 프로젝트.

---

## 1. 브랜치 상태 (중요 — 영상 기획 시 참고)

- 현재 로컬 작업 브랜치 `26th-Aug`가 origin 기준 **가장 최신** (2026-08-26 19:56 커밋).
- `main`은 2026-08-14 마지막 커밋 상태로 **정체**되어 있고, `26th-Aug`가 `main`보다
  75개 중 다수 커밋만큼 앞서 있음 (사실상 `main`을 아직 병합하지 않은 상태).
- 즉 데모/영상 녹화는 **`26th-Aug` 브랜치 기준**으로 진행해야 최신 기능(Control Tower,
  AMR Pose Sync, 수동 조작 등)이 전부 보임. `main`으로 녹화하면 최근 2일치 작업이 전부 빠짐.
- 커밋 안 된 로컬 변경사항 2개 존재 (성능 조사 문서, 코드 변경 아님):
  - `docs/main_mission_perf_investigation.md`
  - `docs/main_mission_physics_lightweighting.md`
  → 실행/실행결과에 영향 없는 "조사 기록" 문서라 데모 자체에는 무관.
- 참고로 원격에는 `hwi_video_concept`, `mock-presentation` 같은 브랜치도 존재하는데,
  전자는 hwi(정동휘)가 pick&place를 실험하다 만든 WIP 브랜치("뭔가 만들었는데 안됨" 커밋
  포함)라 현재 `26th-Aug`와 별개 계열이고 데모 기준으로 쓰기엔 부적합.

---

## 2. 커밋 로그 요약 (기능별 · 날짜별)

전체 75커밋 중 최근 활동은 8/24~8/26 사흘에 집중. 작성자: Euiseok Jeong(정의석, 36건 — 팀 리드/통합), rokey-c2·2rokeyc2-beep(정의석/팀 공용 계정으로 추정, 총 23건), hwi(정동휘, 4건), ygswo76(6건), injae(2건).

### 8/24 — 맵/워크스페이스 정리 + PR 병합
- `portable workspace with fixed paths and cleaned World1 structure`
- `feat: add env_warehouse_only_arms`, `feat: add warehouse_final_final map`
- `Merge pull request #12 from rokey-c2/injae-union-map` (injae 맵 작업 병합)
- README에 venv/터미널 실행법 문서화

### 8/25 — Isaac Sim 물리/비전 다듬기 + Control Tower 백엔드 첫 삽
- **Isaac Sim 카고/물리 튜닝** (정의석, 이 날짜 커밋의 절반 이상): 카고 가드(cargo guard)
  스폰·스케일·물리 바인딩을 20개 넘는 소규모 커밋으로 반복 수정, RTX LiDAR 자기 반사
  억제, 카메라 리그 정리, Nav2 속도 3배 상향 등 — **시뮬레이션 안정화 작업**.
- **Control Tower 백엔드 최초 구현** (`feat: add control tower backend`),
  **MQTT 리프트 Up/Down 연동** (`feat: MQTT Lift Up and Down`).
- 새 맵/AMR/P3020 액션 추가, 이전 로봇 위치에 박스 4개와 함께 새 로봇 배치.

### 8/26 — Control Tower 완성 + AMR 수동 제어 + 정식 Pose 동기화 (가장 최신, 데모 핵심)
- **Control Tower 프론트엔드 대시보드 구축** (`feat: build control tower frontend dashboard`)
- **AMR 수동 조작(Jog) 기능**: 수동 드라이브 API 등록 → deadman-safe 어댑터 →
  프론트엔드 API 클라이언트 → **누르고 있는 동안만 이동(press-and-hold)** 순서로 단계적 구현.
- **AMR MQTT 시작/정지 연동**, 프론트엔드 의존성 lockfile 추가.
- **Canonical Pose 동기화 (가장 최근·가장 완성도 높은 기능)**: 5단계로 순차 구현
  1. `add canonical map pose ownership steps 1-3`
  2. `complete canonical pose ownership steps 4-5`
  3. `restore Isaac AMR from canonical DB pose`
  4. `enforce pose sync control interlocks` ← **HEAD, 브랜치 최신 커밋**
  → Isaac Sim의 AMR 좌표를 DB를 기준(canonical)으로 삼아 ROS2/RViz/MQTT/Backend/
    PostgreSQL/Web까지 한 값으로 동기화. `docs/scenario/amr_pose_sync_complete.md`에
    **End-to-End 검증 완료**로 기록됨 (아래 3장 참고).
- `docs: updated new files` — 최종 문서 정리 커밋.

### 기능별로 다시 묶으면
| 기능 영역 | 상태 | 핵심 커밋/문서 |
|---|---|---|
| Isaac Sim 물류 시뮬레이션 (AMR+P3020+컨베이어+소터) | 완성, 안정화 중 | 8/24~8/25 다수 |
| Vision (YOLO 박스 검출 → 로봇팔 좌표 서비스) | 완성 (로컬 검증) | 8/22 `yolo 박스 검출 및 판단 후 서비스 전송 node` (hwi) |
| Control Tower (Backend+Frontend+DB+MQTT) | 완성 | 8/25~8/26 |
| AMR 수동 제어 (Jog) | 완성 | 8/26 |
| AMR Pose 동기화 (Canonical) | **8/26 완료, 최신·핵심** | 8/26 4연속 커밋 + 완료 문서 |
| main_mission 성능 최적화 | 조사만 완료, 수정 미적용 | 커밋 안 된 문서 2건 (오늘) |

---

## 3. 완성된 기능 vs 진행 중인 기능 (8/28 데모 기준)

### ✅ 데모 가능 (완성 / 검증됨)
1. **Isaac Sim 통합 미션 실행** — README에 명시된 "실제로 끝까지 검증된 경로".
   `run_isaac_mission.sh` → `run_ros2.sh` → P3020 액션 서버 → YOLO 비전 → 미션 트리거,
   5개 터미널로 AMR 이동부터 P3020 Pick&Place까지 풀 파이프라인 동작.
2. **AMR Pose 동기화 (Isaac→ROS2→MQTT→Backend→DB→Web)** — `amr_pose_sync_complete.md`에
   좌표/자세(x, y, yaw)가 각 레이어에서 일치함을 실측값으로 검증 완료. **영상에서 보여주기
   가장 좋은 "실시간성" 증거** (Isaac Sim 화면 ↔ 웹 화면 좌표가 동시에 움직이는 그림).
3. **Vision 박스 검출 → 로봇팔 좌표 전달 (LocateBox/ConfirmGrasp)** — YOLO(`best.onnx`)로
   박스 검출 후 depth 역투영으로 3D 좌표 계산, 로봇팔이 Pick 중 반복 조회해도 좌표가
   안정적으로 "잠기는" 구조까지 구현 및 로컬(WSL) 검증 완료.
4. **Control Tower 웹 대시보드** — Dashboard / Packages / AmrControl 3개 페이지, React +
   FastAPI + PostgreSQL + MQTT 풀스택 연동.
5. **AMR 수동 조작(Jog)** — 웹에서 버튼을 누르고 있는 동안만 이동하는 deadman-safe 방식.
6. **MQTT 기반 AMR 시작/정지, 리프트 Up/Down 원격 제어**.
7. **Conveyor / Wheel Sorter 기본 동작** — 컨베이어 속도 제어, Wheel Sorter 좌우 전환은
   동작하나 **아직 시간 기반 자동 토글**(시뮬레이션 스텝마다 자동 전환)이라 영상에서는
   "분류 동작이 실제로 일어난다"까지만 보여줄 수 있고, "바코드/비전 인식 결과에 따라
   분류"는 다음 항목(진행 중)에 해당.

### 🚧 진행 중 / 미완성 (8/28까지 리스크 있음 — 영상 기획 시 이 부분은 "구현 예정" 또는
    미리 촬영된 부분만 보여주는 식으로 완급 조절 필요)
1. **ConfirmGrasp(흡착 성공 판단) 로직이 placeholder** — 현재는 "박스가 화면에서
   사라지면 성공"으로 임시 처리. 실제 그리퍼 occupancy/force 센서 연동 미확정.
2. **Wheel Sorter A/B/C 실제 분기 조건** — `control_tower_progress_2026-08-25.md`에
   시나리오(순차 판단: A 아니면 B, B 아니면 C, 다 아니면 Exception)는 설계돼 있으나,
   비전/바코드 기반 실제 판단 로직이 구현됐는지는 코드 레벨 재확인 필요.
3. **3-AMR/3-P3020 확장 버전의 성능 저하 문제** — 로봇을 1대→3대로 늘린 뒤 스폰 시점부터
   느려지는 문제를 오늘(8/26) 코드 리딩만으로 원인 조사(USD 전수 스캔 중복 실행, Fabric
   미사용 등 후보 도출)했으나, **실제 Isaac Sim PC에서 수정/검증 전 단계**. 데모를
   3-로봇 풀스케일로 할 계획이면 리스크 요소.
4. **IW Hub 단일 자율주행 테스트 경로** — README에 "이 저장소만으로는 끝까지 실행 안 됨"
   이라고 명시(목표 좌표를 Nav2에 보내는 트리거 스크립트 부재). 이 경로 단독으로는
   데모 불가, 반드시 "전체 통합 미션" 경로로 진행해야 함.
5. **Exception 시나리오 처리** (AMR 충돌, Pick 실패, E-Stop 등) — `exception_scenario.md`에
   시나리오 목록만 정의돼 있고 실제 예외처리 구현 여부는 별도 코드 확인 필요 (영상에
   "예외 대응"을 넣고 싶다면 우선 실제 동작 여부부터 점검 권장).

**정리**: 8/28 데모 영상은 "①Isaac Sim 통합 미션 + ②Vision 박스검출 + ③P3020 Pick&Place +
④AMR Pose 동기화(웹 실시간 반영) + ⑤Control Tower 대시보드/수동조작" 조합이 가장 안전하게
완성도 있게 보여줄 수 있는 축. 3-로봇 동시 운용이나 바코드 기반 분류, 예외 상황 대응은
사전에 실제 동작을 확인한 뒤 영상에 넣을지 결정하는 것을 권장.

---

## 4. 시스템 구조 (README + docs/architecture 기준, 최신 확인용)

### 전체 아키텍처 (`docs/architecture/system_architecture.md`)
```text
React Dashboard
→ FastAPI Backend
→ ROS2 Mission Manager
→ AMR / P3020 / Sorter Controllers
→ Isaac Sim ROS2 Bridge
→ Digital Twin
```

### Pose 동기화 상세 흐름 (`amr_pose_sync_complete.md`, 8/26 완료)
```text
Isaac Sim → /amr_a/map_pose → ROS2/TF/RViz2 → Pose Sync Manager
→ MQTT → Backend API → PostgreSQL → React Web
```

### Control Tower 데이터 흐름 (`control_tower_development_plan.md`)
```text
Isaac Sim → ROS2 Bridge → ROS2 Jazzy → ROS2 Adapter → FastAPI
  ├─ REST API
  └─ WebSocket
→ PostgreSQL → React Control Tower
```

### 개발 환경
- Isaac Sim 5.1.0 / ROS 2 Jazzy / `ROS_DOMAIN_ID=110` / `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- Nav2, RTX LiDAR, PhysX
- Backend: FastAPI + PostgreSQL, Frontend: React (Vite, `localhost:5173`)
- MQTT(Mosquitto), Docker Compose로 Backend/DB/MQTT 일괄 기동

### ROS2 네임스페이스 (`docs/architecture/ros2_architecture.md`)
`/amr_in_01`, `/amr_in_02`, `/amr_out_a_01`, `/amr_out_a_02`, `/amr_out_b_01`,
`/amr_out_b_02`, `/arm_in_01`, `/arm_a_01`, `/arm_b_01`, `/sorter_01`

### FSM (전체 미션 상태 흐름, `docs/scenario/fsm.md`)
```text
IDLE → READY_CHECK → INBOUND_PICKUP → ROBOT_LOADING → MAIN_CONVEYOR
→ SORTING → REGION_CONVEYOR → BOX_LOADING → BOX_CHECK
→ OUTBOUND / INBOUND_PICKUP → BOX_REPLACE → COMPLETE
(에러 시 ERROR → RECOVERY / E_STOP)
```

### 폴더 구조 핵심
- `isaac_sim/` — 시뮬레이션 본체 (robots/{p3020, iw_hub, forklift_b}, equipment/{conveyor,
  wheel_sorter}, world_setup.py, main.py/main_mission.py)
- `ros2_ws/src/` — `arm_controller`(P3020 Pick&Place 액션 서버), `vision_node`
  (`locate_box_node.py`, YOLO), `logistics_interfaces`(`LocateBox.srv`, `ConfirmGrasp.srv`),
  `mission_manager`, `sorter_controller` (일부는 별도 브랜치에만 존재)
- `backend/app/` — FastAPI (api/models/services), `backend/db/migrations`
- `frontend/src/` — `pages/{Dashboard,Packages,AmrControl}Page.jsx`, `components/`
- `models/parcel_box_yolo_model/` — 학습된 YOLO 박스 검출 모델 (`best.pt`/`best.onnx`)
- `docs/` — architecture / scenario / team 문서 (이번 요약의 근거 자료 원본)

---

## 5. 담당 파트 요약

`docs/team/role_assignment.md` 기준 공식 역할 분담:

| 담당 | 역할 |
|---|---|
| 정의석 | 팀 리드 / ForkliftB / FSM / 전체 통합 |
| 황인재 | P3020 Pick & Place / 흡착 |
| 서진우 | Isaac Sim Scene / P3020 A/B 적재 통합 |
| 정동휘 | Conveyor / Wheel Sorter |
| 전원 | End-to-End 통합 테스트 |

> ⚠️ 참고: 위 문서는 초기 역할 분담표이고, 실제 커밋 이력을 보면 **정동휘(git 계정 `hwi`)의
> 실작업은 문서상 역할(Conveyor/Sorter)보다 넓게 "Vision + 로봇팔 연동 인터페이스"까지
> 확장돼 있음**. 영상에서 정동휘 파트를 강조하려면 아래 실제 구현 내용을 기준으로 삼는 것을 권장.

### 🎯 정동휘 파트 상세 (비전 + 로봇팔 제어 연동) — 강조 포인트

**핵심 커밋**: `67082af — yolo 박스 검출 및 판단 후 서비스 전송 node` (2026-08-22)

이 한 커밋으로 아래 구조를 신규 구현:

1. **`ros2_ws/src/vision_node/vision_node/locate_box_node.py`** — 비전 노드 본체
   - `/rgb`, `/depth` 토픽을 `ApproximateTimeSynchronizer`로 동기화
   - `object_detector.py`(YOLO, `best.onnx`, 클래스 "box")로 박스 검출
   - depth 픽셀값으로 pinhole 역투영해 카메라 좌표계 3D 좌표 계산
   - **좌표 잠금(lock) 메커니즘**: Pick 동작 도중 로봇팔이 좌표를 여러 번 물어봐도
     카메라 노이즈로 값이 흔들리지 않도록, 한 번 검출한 좌표를 Pick이 끝날 때까지 고정
2. **`ros2_ws/src/logistics_interfaces/`** — 로봇팔↔비전 통신 인터페이스(서비스) 정의
   - `LocateBox.srv`: 로봇팔이 박스 좌표를 요청 → 비전이 3D 좌표 + 잠금 상태 응답
   - `ConfirmGrasp.srv`: 로봇팔이 흡착 시도 후 성공 여부를 비전에게 확인 요청
3. **`docs/architecture/locate_box_pipeline.md`** — 위 파이프라인 설계 문서 (본인 작성,
   시퀀스 다이어그램 포함, "영상인식(vision) ↔ 로봇팔(P3020) 간 박스 위치 인식 서비스"로
   명시)

→ 즉 정동휘 파트는 **"카메라가 본 박스를 로봇팔이 정확히 집을 수 있는 좌표로 바꿔주고,
Pick 시도 중 좌표가 흔들리지 않게 잠가주는" 비전-로봇팔 연동 계층**. 로봇팔 자체의 관절
모션/흡착 제어(`arm_controller`)는 주로 팀 공용/정의석 쪽에서 이어받아 구현했지만, **그
모션이 참조하는 "어디를 집을지" 좌표를 정동휘의 vision_node가 공급**하는 구조.

영상에서 이 파트를 보여줄 때 추천 흐름:
카메라 뷰(RGB+YOLO 박스 표시) → LocateBox 응답 좌표 → P3020 팔이 그 좌표로 이동해
흡착하는 장면을 연결해서 보여주면 "비전이 로봇팔을 제어한다"는 인과관계가 명확히 드러남.

### 다른 팀원 최근 실작업 (커밋 기준 보강)
- **정의석**: 전체 커밋의 절반 가까이 차지. Isaac Sim 물리/카고 튜닝, Nav2 속도 조정,
  Control Tower 백엔드/프론트엔드, AMR 수동 제어, Canonical Pose 동기화까지 — 사실상
  최근 통합 작업 대부분을 주도.
- **rokey-c2 / 2rokeyc2-beep** (공용 계정으로 추정): 맵 작업, MQTT 리프트, Control Tower
  백엔드 초기 버전.
- **ygswo76**: 박스 데이터 학습 모델 관련 커밋 (`박스 데이터 학습 모델`) — YOLO 모델 학습 쪽.
- **injae**: 색체(색상) 탐지 노드, 맵 병합(PR #12).

---

## 6. 데모 캡처 포인트 체크리스트

**저장소 안에는 실제 데모 스크린샷/영상 파일이 없음** (git에 들어있는 이미지는 전부
Isaac Sim 3D 에셋 텍스처(.png)이며 캡처 자료가 아님). 아래는 8/28 촬영 시 새로 찍어야
할 "보여주면 좋은 장면" 목록 — 각 항목에 대응하는 실행 스크립트/화면을 붙였다.

| # | 캡처 포인트 | 확인 방법 |
|---|---|---|
| 1 | Isaac Sim 창고 전경 + AMR/P3020 스폰 | `./scripts/run_isaac_mission.sh` 실행 직후 뷰포트 |
| 2 | YOLO 박스 검출 오버레이 (bbox) | `box_detector_node.py` / RViz 카메라 토픽 |
| 3 | P3020 로봇팔이 LocateBox 좌표로 이동 → 흡착(Pick) | Isaac Sim 뷰포트, 팔 이동 클로즈업 |
| 4 | AMR Nav2 경로 추적 | RViz2 (`run_ros2.sh` 이후) |
| 5 | Conveyor → Wheel Sorter 분류 이동 | Isaac Sim 뷰포트, 컨베이어 라인 |
| 6 | Control Tower 웹 대시보드 (`localhost:5173`) — Dashboard/Packages/AmrControl 페이지 전환 | `npm run dev` 후 브라우저 |
| 7 | **Isaac Sim 화면 ↔ 웹 화면 좌표 동시 갱신** (Pose Sync 증거) | Isaac Sim 뷰포트 + 웹 화면 나란히, `amr_pose_sync_complete.md` 5장 수치 참고 |
| 8 | AMR 수동 조작(Jog) 버튼 누르는 즉시 반응 | AmrControl 페이지, 버튼 누르고 있는 동안만 이동하는 것 확인 |
| 9 | MQTT 리프트 Up/Down 원격 제어 | 웹에서 명령 → Isaac Sim 리프트 반응 |
| 10 | Backend API/DB 값 = 웹 화면 값 일치 (선택, 기술 신뢰도용) | `curl http://localhost:8000/api/equipment` 결과와 웹 화면 나란히 |

**촬영 순서 제안**: 1 → 2 → 3(정동휘 파트 강조 구간) → 4 → 5 → 6/7(Control Tower + 실시간성)
→ 8/9(수동 제어) 순으로 이어 찍으면 "자동 미션 전체 흐름 → 관제 시스템"의 스토리라인이 됨.
