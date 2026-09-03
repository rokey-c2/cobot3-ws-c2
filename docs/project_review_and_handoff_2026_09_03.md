# 프로젝트 점검 및 작업 인수인계 (2026-09-03)

## 1. 문서 목적

이 문서는 현재 저장소의 전체 점검 결과와 대화에서 확인된 후속 개발 요구사항을 한곳에 기록한다.

특히 다음 요구사항을 잊지 않고 이어서 구현하기 위한 인수인계 문서다.

> IW Hub를 수동 주행으로 P3020 앞에 이동시킨 뒤, Control Tower Dashboard의 버튼을 눌러 기존 자동 임무의 `P3020 도착 완료` 신호와 동일한 ROS2/MQTT 흐름을 호출한다.

이 기능은 2026-09-03에 Frontend → FastAPI → PostgreSQL → MQTT → ROS2
Mission 연결까지 구현했다. 단, 실제 Isaac Sim에서 수동 주행부터 P3020 동작
완료까지 이어지는 전체 Runtime E2E 검증은 남아 있다.

### 1.1 프로젝트 실행 기준 문서

앞으로 프로젝트의 실행 순서, 환경 변수, 시스템 구성과 검증 기준은 다음
Notion Portfolio 문서를 우선 기준으로 삼는다.

- [AMR–협동로봇 연계 대형 택배 분류 자동화 및 실시간 관제 시스템 | Portfolio](https://app.notion.com/p/466fe8937ec3824e93e801b6c9c8c22b?pvs=204)
- 통합 ROS Domain: `ROS_DOMAIN_ID=110`
- RMW: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- Isaac 통합 실행: `bash scripts/run_isaac_mission.sh`
- 실행 순서: Notion 문서의 Terminal 1~10
- Canonical AMR Pose: `/amr_a/map_pose`

Notion 문서와 코드가 충돌하면 현재 작업 브랜치의 코드를 확인하고 차이를
기록한 뒤 수정한다. `scripts/run_isaac.sh`는 현재 저장소에 없는 구형 USD
경로를 사용하므로 통합 실행에는 사용하지 않는다.

---

## 2. 현재 Git 상태

- 점검 당시 브랜치: `fix/environment-naming-alignment`
- 점검 당시 HEAD: `833f055 fix: decouple manual jog from Nav2 pose sync`
- 원격 추적 브랜치: `origin/fix/environment-naming-alignment`
- Git 추적 파일 변경: P3020 수동 도착 확인 기능 구현 변경 존재
- 저장소의 추적 파일 수: 약 1,658개
- 주요 대용량 항목: Isaac Sim USD/재질 자산과 YOLO 모델

최근 변경은 주로 환경 명칭 정렬과 수동 AMR 제어를 Nav2 pose sync에서 분리하는 작업이다.

---

## 3. 프로젝트 구성

현재 프로젝트는 다음 계층으로 구성된다.

1. Isaac Sim 5.1 디지털 트윈
2. ROS 2 Jazzy 로봇·비전·장비 제어
3. ROS 2 ↔ MQTT 어댑터
4. Mosquitto MQTT broker
5. FastAPI Control Tower backend
6. PostgreSQL 상태·임무·이벤트 저장소
7. React/Vite Control Tower frontend

주요 데이터 흐름은 다음과 같다.

```text
Isaac Sim
  → ROS 2 topics/actions
  → ROS2/MQTT adapter
  → Mosquitto
  → FastAPI/PostgreSQL
  → React Control Tower
```

영상은 ROS Image/MJPEG stream을 통해 Dashboard에 표시된다.

---

## 4. 점검 및 검증 결과

### 4.1 성공한 검증

- 독립 Python 단위 테스트: `13 passed`
- React production build: 성공
- Vite 변환 모듈: 42개
- JavaScript bundle: 약 197 KB, gzip 약 62 KB
- 전체 Python 파일 문법 검사: 성공
- Git 추적 작업 트리: 깨끗함

단위 테스트는 저장소의 `tests/`만 지정해 실행했다.

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -p no:cacheprovider -q tests
```

프런트엔드 결과물은 프로젝트 파일을 만들지 않도록 `/tmp`에 출력했다.

```bash
cd frontend
npm run build -- --outDir /tmp/cobot3-frontend-audit --emptyOutDir
```

### 4.2 전체 pytest 실행 제한

저장소 루트에서 pytest 전체 수집을 실행하면 다음 파일이 일반 Python 환경에서 `isaacsim`을 즉시 import한다.

- `isaac_sim/test_full_pipeline.py`

Isaac Sim 전용 Python 환경이 아니기 때문에 아래 오류로 수집이 중단된다.

```text
ModuleNotFoundError: No module named 'isaacsim'
```

이는 단위 테스트 실패라기보다 테스트 환경이 분리되지 않은 문제다. 일반 단위 테스트와 Isaac Sim 통합 테스트에 pytest marker 또는 별도 실행 설정이 필요하다.

### 4.3 캐시 관련 참고

문법 검사 중 Git이 무시하는 `__pycache__`가 갱신됐을 가능성이 있다. 기존 캐시와 새 캐시를 안전하게 구분할 수 없어 임의 삭제하지 않았다. Git 추적 파일에는 변경이 없다.

---

## 5. 구현 완성도 평가

### 5.1 비교적 완성도가 높은 부분

- FastAPI 장비·AMR·임무·택배·이벤트 API
- MQTT 상태 수신 및 PostgreSQL 반영
- AMR odometry와 pose sync 저장·복구
- 장비 START/STOP 명령과 결과 반영
- AMR navigate/lift/manual 명령 발행
- 공정 이벤트 해석과 임무/택배 추적 레코드 생성
- React Dashboard, 택배 조회, AMR 제어 화면
- 주요 ROS2/MQTT adapter 로직
- IW Hub 및 P3020의 실제 mission agent 계열 코드

### 5.2 스캐폴딩 또는 미완성 부분

다음 ROS 2 디렉터리는 소스는 있지만 완성된 설치 패키지가 아니다.

- `ros2_ws/src/mission_manager`
- `ros2_ws/src/sorter_controller`
- `ros2_ws/src/logistics_bringup`

현재 `package.xml`이 확인되는 자체 패키지는 다음과 같다.

- `amr_controller`
- `arm_controller`
- `logistics_interfaces`
- `vision_node`

중앙 Mission FSM은 상태 enum만 있고 실제 전이 로직이 없다.

```python
def step(self):
    # TODO: transition logic
    return self.state
```

통합 launch 파일도 TODO 상태다.

```text
# TODO:
# mission_manager
# amr_controller
# arm_controller
# sorter_controller
# integrated launch file
```

Forklift B, 일반 P3020 agent, 일부 motion/controller 파일에도 TODO 또는 `pass`가 남아 있다. 다만 실제 시연 흐름은 별도의 구체적인 mission agent와 실행 스크립트가 담당하므로, 이 골격 파일들만 보고 전체 기능이 없다고 판단하면 안 된다.

---

## 6. 주요 기술 부채와 위험

### 6.1 ConfirmGrasp 판정

현재 비전 흐름은 요청받은 RGB 영상에서 박스가 더 이상 검출되지 않으면 흡착 성공으로 간주한다. Depth 영상은 최종 성공 판정에 사용하지 않는다.

가능한 오판 원인:

- 박스 가림
- 조명 변화
- 순간적인 YOLO 미검출
- 카메라 흔들림
- 박스가 시야 밖으로 밀린 경우

향후 depth, gripper 상태, 접촉 또는 압력 상태를 함께 사용하는 방식이 필요하다.

### 6.2 테스트 구조

- 일반 단위 테스트와 Isaac Sim 전용 테스트가 분리되지 않음
- ROS 2 package-level 테스트 부족
- 프런트엔드 테스트와 lint script 없음
- 실제 MQTT/PostgreSQL/ROS 2를 함께 검증하는 자동화 통합 테스트 부족

### 6.3 운영 보안

현재 구성은 로컬 시연 환경을 전제로 한다.

- FastAPI 제어 API 인증 없음
- 장비 제어 권한 구분 없음
- MQTT 계정/TLS 미구성
- PostgreSQL과 MQTT 포트가 compose에서 host에 공개됨

외부 네트워크나 운영 환경에 배포하려면 인증, 권한 검사, 네트워크 제한과 MQTT TLS가 필요하다.

### 6.4 배포 구성

`compose.yaml`에는 PostgreSQL, Mosquitto, FastAPI만 포함된다.

- Frontend: 별도 Vite 실행
- ROS 2: host에서 별도 실행
- Isaac Sim: host에서 별도 실행
- Nav2/NVIDIA workspace: 외부 workspace 필요

따라서 현재 구성은 전체 시스템 원클릭 실행 환경이 아니다.

### 6.5 문서 정합성

루트 `README.md`는 상세하지만, 일부 설명은 스캐폴딩까지 완성된 것처럼 읽힐 수 있다. `docs/architecture/system_architecture.md`는 실제 구현 규모에 비해 너무 짧다.

문서에서는 다음 상태를 명확히 구분하는 것이 좋다.

- 구현 및 검증 완료
- 시연 환경에서만 동작
- 외부 workspace 필요
- placeholder/실험 코드
- 향후 계획

---

## 7. Dashboard의 `P3020 도착 완료` 버튼 요구사항

### 7.1 사용자 시나리오

1. 운영자가 Dashboard의 AMR 수동 조작 기능을 사용한다.
2. IW Hub를 P3020 작업 위치 앞까지 이동시킨다.
3. 운영자가 Dashboard에서 `P3020 도착 완료` 버튼을 누른다.
4. Backend가 기존 자동 운행에서 사용하는 도착 이벤트와 동일한 명령 또는 이벤트를 발행한다.
5. 기존 ROS2/MQTT adapter가 이를 수신한다.
6. P3020 pick/place 작업을 포함한 다음 mission stage가 시작된다.

핵심 원칙은 Dashboard만을 위한 별도 우회 로직을 만들지 않고, 기존 자동 임무의 도착 신호 처리 경로를 재사용하는 것이다.

### 7.2 확정된 구현 경로

- 버튼 위치: `AMR Control` 화면
- Backend API: `POST /api/missions/current/confirm-p3020-arrival`
- 명령 상태 조회: `GET /api/missions/commands/{command_id}`
- MQTT topic: `controltower/command/mission/p3020-arrival`
- ROS2 topic: `/amr_a/p3020_arrival_confirm`
- Mission 실행 옵션: `manual_delivery:=true`
- 허용 단계: 활성 Mission의 `current_stage == AMR_NAVIGATION`
- 합류 상태: `REQUEST_CONVEYOR_DOCK`
- 이후 흐름: 자동 임무와 동일한 Dock → Cargo 하강 → `P3020_START`
- 중복 요청: 같은 Mission의 PENDING/RUNNING/SUCCESS 명령이 있으면 HTTP 409
- UI 결과: DB command의 PENDING/RUNNING/SUCCESS/FAILED 상태 polling

### 7.3 구현 결과

구현된 흐름은 다음과 같다.

```text
Dashboard button
  → FastAPI manual-arrival endpoint
  → DB에서 현재 mission/stage 검증
  → P3020 arrival MQTT command 발행
  → ROS2/MQTT adapter
  → 기존 P3020 mission trigger
  → 결과 MQTT event
  → DB 상태 갱신
  → Dashboard refresh/status 표시
```

사용 API는 다음과 같다.

```text
POST /api/missions/current/confirm-p3020-arrival
```

적용된 안전 조건:

- 활성 mission이 있어야 함
- 현재 stage가 P3020 도착 대기 상태여야 함
- 대상 AMR과 P3020 equipment code가 유효해야 함
- 이미 처리된 도착 확인은 중복 실행하지 않아야 함
- backend의 MQTT publish 실패 시 DB command/event를 실패 상태로 남겨야 함

### 7.4 현재 상태

- 요구사항 확인: 완료
- 기존 자동 도착 이후 흐름 조사: 완료
- Backend endpoint와 DB command 기록: 완료
- Frontend 버튼과 결과 상태 표시: 완료
- MQTT → ROS2 adapter와 Mission 합류 로직: 완료
- 중복 클릭, stage, AMR/P3020 상태 검증: 완료
- 독립 Python 테스트: `13 passed`
- React production build: 성공
- 실제 FastAPI → DB → MQTT topic/payload 검증: 성공
- 중복 요청 HTTP 409 검증: 성공
- 실제 Isaac Sim 수동 주행 → P3020 작업 E2E 검증: 미착수

---

## 8. 권장 후속 작업 순서

1. 모든 프로세스를 `ROS_DOMAIN_ID=110`으로 통일해 실행
2. Notion Terminal 1~8 순서로 Docker, Isaac, Nav2, Pose Sync, Adapter, P3020 Action, Vision, Frontend 실행
3. Mission을 `manual_delivery:=true`, `simulate_p3020:=false`로 실행
4. Isaac Sim에서 수동 주행 → 버튼 → P3020 작업 전체 검증
5. 명령이 PENDING → RUNNING → SUCCESS로 바뀌는지 DB/UI 확인
6. 일반 pytest와 Isaac 전용 테스트 분리

프로젝트 전체 기술 부채는 그다음 순서로 처리한다.

1. Mission FSM/통합 launch의 사용 여부 확정
2. 사용하지 않는 스캐폴딩 정리 또는 실제 구현
3. ConfirmGrasp 판정 강화
4. 프런트엔드 테스트와 lint 추가
5. 인증·권한·MQTT TLS 등 운영 보안 추가
6. README와 실제 구현 상태 정합성 개선

---

## 9. 작업 시 주의사항

프로젝트 실행 시 Notion Portfolio의 Terminal 순서와 Domain 110 설정을 먼저 확인한다. 현재 저장소에서는 별도의 `AGENTS.md` 또는 `agent.md`를 확인하지 못했으므로, 이후 해당 파일이 추가되면 그 지침도 함께 적용한다.

특히 navigation map, USD, 설정 파일이 누락되거나 오래돼 보이더라도 임의로 재생성하지 않는다. 먼저 해당 경로의 다음 정보를 확인한다.

```bash
git status -- <path>
git diff -- <path>
git log --all -- <path>
```

그 후 변경 내용과 이유를 설명하고 사용자 승인을 받은 뒤 수정한다.

