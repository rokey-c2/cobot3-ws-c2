# Control Tower 작업 인수인계 프롬프트

아래 프로젝트를 이전 대화에서 이어서 작업해줘. 이 문서에 적힌 상태를 기준으로 진행하고, 이미 실제 검증이 끝난 기능은 불필요하게 다시 작성하거나 망가뜨리지 마.

---

## 1. 프로젝트 기본 정보

- 프로젝트: **AMR–협동로봇 연계 택배 분류 자동화 및 실시간 관제 시스템**
- GitHub 저장소: `https://github.com/rokey-c2/cobot3-ws-c2.git`
- 작업 브랜치: `euiseok-control-tower`
- 로컬 경로: `~/collaboration/cobot3-ws-c2`
- 현재 기준 최신 커밋: `03608b9f6878dea6e7b42897d1e29ba05b5534a9`
- 기준 커밋 메시지: `fix: resolve equipment code type in command result query`
- 이전 주요 커밋:
  - `5a869912a8e3f362bd52af511e5218a718bfdad8`
    - AMR Navigate와 Lift Up/Down의 MQTT–Backend 연결
  - `6a4febbd076bb3445a27c985a03c1491d8aeb25a`
    - AMR 실제 Start/Stop 연결
  - `03608b9f6878dea6e7b42897d1e29ba05b5534a9`
    - Start/Stop 결과 DB 반영 SQL 오류 수정

### 개발 환경

- Ubuntu 24.04
- Isaac Sim 5.1.0
- ROS2 Jazzy
- `ROS_DOMAIN_ID=110`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- FastAPI
- PostgreSQL 16
- Mosquitto MQTT
- Docker Compose

---

## 2. 매우 중요한 작업 원칙

1. 사용자가 직접 코드를 수정하다 기존 정상 기능이 고장 나는 것을 원하지 않는다.
2. 코드는 Assistant가 `euiseok-control-tower` 브랜치의 최신 원격 상태를 기준으로 수정한다.
3. 수정 후 안전한 테스트를 수행하고 기능 단위로 커밋한 뒤 같은 브랜치에 푸시한다.
4. `force push`는 사용하지 않는다.
5. 작업 전에 원격 브랜치 HEAD가 예상 커밋과 일치하는지 확인한다.
6. 이미 정상인 Navigate와 Lift 코드는 꼭 필요한 경우가 아니면 건드리지 않는다.
7. 기능을 “완료”라고 표현하려면 다음 전체 흐름이 실제로 검증돼야 한다.

```text
FastAPI
→ PostgreSQL PENDING
→ MQTT
→ ROS2 Adapter
→ Nav2 또는 Isaac 실제 동작
→ 실제 상태/결과 피드백
→ MQTT 결과
→ PostgreSQL RUNNING/SUCCESS/FAILED 반영
```

8. Backend나 MQTT 규격만 만든 기능은 “Backend 완료” 또는 “기반 구현”이라고 표현하고, 실제 장비 E2E 완료라고 말하지 않는다.
9. `~/.bashrc`는 수정하지 않는다.
10. 명령어를 안내할 때 반드시 다음 내용을 함께 적는다.
    - 실행할 터미널 번호
    - 다른 터미널을 계속 켜둘지 여부
    - 명령 실행 전 필요한 현재 상태
    - 실행 폴더
    - 명령이 바로 종료되는지 계속 실행되는지
    - 정상 예상 로그 또는 결과

---

## 3. 현재 주요 파일

### Backend

- `backend/app/api/amr.py`
  - AMR Navigate API
  - AMR Lift Up/Down API
- `backend/app/api/equipment.py`
  - 개별 Equipment Start/Stop API
  - 현재 실제 제어 지원 종류는 `AMR`
- `backend/app/api/system.py`
  - 전체 System Start/Stop API
  - 현재 실제 Adapter가 연결된 장비만 제어
  - 미연결 장비는 `unsupported_equipment`에 표시
- `backend/app/mqtt_client.py`
  - Navigate/Lift/Start/Stop MQTT 발행
  - AMR odom/Lift/상태 수신 및 DB 반영
  - 명령 결과 `PENDING → RUNNING → SUCCESS/FAILED` 반영
- `backend/db/init.sql`
  - `equipment`, `equipment_state`, `equipment_command`
  - `mission`, `mission_stage`
  - `package`, `package_event`, `zone`
- `backend/db/seed.sql`
  - `AMR_IN`, `P3020_IN`, `MAIN_CONVEYOR`, `SORTER_A/B/C`
  - Zone 기본 데이터

### ROS2 MQTT Adapter

- `scripts/ros2_mqtt_adapter.py`
  - `/navigate_to_pose` Action 연결
  - `/amr_a/lift_command` 발행
  - `/amr_a/lift_state` 구독
  - `/chassis/odom` 구독
  - `/cmd_vel` 정지 Twist 발행
  - AMR Start/Stop 제어 게이트
  - Stop 상태에서 Navigate/Lift 차단
  - 이동 중 Stop 시 Nav2 목표 취소

### 테스트

- `tests/test_backend_control.py`
- `tests/test_ros2_mqtt_adapter_control.py`
- 최근 자동 테스트 4개 통과
- 관련 Python 파일 `py_compile` 통과

---

## 4. MQTT 및 ROS2 연결 규격

### MQTT 명령 Topic

```text
controltower/command/amr/AMR_IN/navigate
controltower/command/amr/AMR_IN/lift
controltower/command/equipment/AMR_IN/control
```

### MQTT 상태 및 결과 Topic

```text
controltower/amr/AMR_IN/odom
controltower/amr/AMR_IN/lift
controltower/amr/AMR_IN/status
controltower/result/command
```

### ROS2

```text
/navigate_to_pose
/chassis/odom
/cmd_vel
/amr_a/lift_command
/amr_a/lift_state
```

### Start/Stop MQTT Payload 예시

```json
{
  "command_id": 31,
  "action": "STOP"
}
```

마지막 Start/Stop 명령은 MQTT retain으로 보존된다. Adapter를 재시작하면 마지막 STOP 상태가 다시 적용되어 임의로 제어가 풀리지 않는다.

---

## 5. 실제 E2E 검증 완료 기능

### 5.1 특정 위치까지 AMR 이동

상태: **완료**

```text
FastAPI Navigate
→ DB PENDING
→ MQTT
→ ROS2 Adapter
→ /navigate_to_pose
→ Nav2 실제 이동
→ RUNNING/SUCCESS 또는 FAILED
→ DB 반영
```

검증된 목표 좌표 예시:

```json
{
  "x": 1.30104,
  "y": -0.06065,
  "yaw": 0.0
}
```

### 5.2 AMR Lift Up/Down

상태: **완료**

```text
FastAPI Lift
→ DB PENDING
→ MQTT
→ ROS2 Adapter
→ /amr_a/lift_command
→ Isaac 실제 Lift 동작
→ /amr_a/lift_state
→ RUNNING/SUCCESS
→ DB 반영
```

실제 검증 기록:

- command `#23`: `LIFT_UP SUCCESS`
- command `#24`: `LIFT_DOWN SUCCESS`
- command `#25`: `LIFT_UP SUCCESS`
- 같은 Lift 상태가 반복 발행될 때 MQTT/DB 업데이트가 반복되지 않도록 상태 변경 시에만 발행하도록 최적화 완료

### 5.3 AMR 개별 Start/Stop

상태: **완료**

검증된 기능:

- 개별 START
- 개별 STOP
- STOP 상태에서 새 Navigate 차단
- STOP 상태에서 새 Lift 차단
- 이동 중 STOP 시 Nav2 목표 취소
- `/cmd_vel`에 zero Twist 지속 발행
- 실제 감속 후 완전 정지
- 명령 및 장비 상태 DB 반영
- Adapter 재시작 시 retain된 STOP 복원

실제 검증 기록:

- command `#26`: 개별 `STOP SUCCESS`
- command `#27`: 개별 `START SUCCESS`
- STOP 상태에서 Navigate API 호출 시 `409 Conflict`

```json
{
  "detail": "AMR must be RUNNING before navigation"
}
```

### 5.4 이동 중 STOP 실제 검증

- command `#28`: `NAVIGATE RUNNING`
- command `#29`: `STOP SUCCESS`
- Nav2 취소 결과:

```text
Nav2 cancel response: goals_canceling=1
Nav2 CANCELED: command_id=28
```

- DB 결과:

```text
#29 STOP      SUCCESS
#28 NAVIGATE  FAILED  Navigation canceled by STOP
```

- 속도 설정의 감속 때문에 STOP 직후 약 0.58m 제동거리가 발생했으나 이후 완전히 정지함
- 3초 간격 정지 확인 좌표:

```text
POSITION 1: x=-4.9428839684, y=-0.5414083004
POSITION 2: x=-4.9428820610, y=-0.5414091349
```

- 3초 동안 변화량이 약 0.000002m이므로 실제 정지 완료

---

## 6. System 전체 Start/Stop 상태

### 현재 연결된 AMR 기준

상태: **완료**

- command `#30`: System `START SUCCESS`
- command `#31`: System `STOP SUCCESS`
- 최종 AMR 상태:

```text
AMR_IN | STOPPED | AUTO | DOWN
```

### 전체 프로젝트 장비 기준

상태: **부분 완료**

현재 System API에서 실제 제어되는 장비:

```text
AMR_IN
```

아직 실제 Start/Stop Adapter가 없는 장비:

```text
P3020_IN
MAIN_CONVEYOR
SORTER_A
SORTER_B
SORTER_C
```

System API는 이 장비들을 거짓으로 `SUCCESS` 처리하지 않고 `unsupported_equipment`로 반환한다.

---

## 7. 현재 기능 진행 표

| 기능 | 상태 | 정확한 범위 |
|---|---|---|
| 특정 위치까지 AMR 이동 | ✅ 완료 | 실제 Nav2 이동과 결과 DB 반영까지 검증 |
| AMR Lift Up/Down | ✅ 완료 | Isaac 실제 동작과 상태 DB 반영까지 검증 |
| AMR 개별 Start/Stop | ✅ 완료 | 차단·허용, 이동 취소, 실제 정지까지 검증 |
| System Start/Stop | 🟡 부분 완료 | 현재 연결된 AMR만 실제 제어 |
| 기계 위치 및 상태 | 🟡 부분 완료 | AMR odom, Lift, RUNNING/STOPPED 저장 완료 |
| Mission 실시간 단계 | ❌ 미완료 | Backend/MQTT 규격부터 구현 가능 |
| 박스 실시간 위치 추적 | ❌ 미완료 | Zone 기반 Backend 구현 가능 |
| P3020 MQTT/상태 | ❌ 미완료 | 팀원 실제 동작 코드 필요 |
| Conveyor/Sorter MQTT/상태 | ❌ 미완료 | 팀원 실제 동작 코드 필요 |

---

## 8. 중요한 제한사항과 팀원 의존성

### 8.1 AMR 위치

현재 DB에 저장되는 AMR 위치는 `/chassis/odom`의 **로컬 odom 좌표**다.

```text
초기: x≈0, y≈0
이동 후 마지막 확인: x≈-4.943, y≈-0.541
```

Dashboard에서 실제 지도 좌표로 표시하려면 나중에 `map → base_link` 또는 `map → amr_a/base_link` TF를 사용해야 한다. 이 작업에는 팀원이 완성한 실제 Nav2 지도와 TF가 필요하다.

### 8.2 Mission 실시간 단계

다음 Backend 기능은 지금 구현할 수 있다.

- Mission MQTT 이벤트 규격
- `mission.current_stage` 갱신
- `mission_stage` 시작/완료/실패 이력
- API 조회
- 테스트 MQTT 이벤트로 실시간 검증

하지만 전체 물류 Mission을 실제 E2E 완료로 부르려면 AMR뿐 아니라 P3020, Conveyor, Sorter 코드가 실제 단계 이벤트를 발행해야 한다.

### 8.3 박스 실시간 위치 추적

map 없이 Zone 방식으로 구현할 수 있다.

```text
INPUT_ZONE
→ AMR_IN
→ P3020_IN
→ MAIN_CONVEYOR
→ SORTER_A/B/C
→ REGION_A/B/C
→ COMPLETE
```

현재 구현 가능한 Backend 범위:

- Package MQTT 이벤트 규격
- `package.current_zone_id` 갱신
- `package.status` 갱신
- `package_event` 이력 저장
- API 조회
- 테스트 이벤트 검증

실제 전체 경로 자동 추적은 팀원들의 P3020/Conveyor/Sorter 작업 완료 후 최종 연결한다.

---

## 9. 현재 상태에서 완전히 마무리 가능한 독립 작업

전체 장비 코드가 없어도 다음 AMR 범위는 실제 E2E까지 완성할 수 있다.

1. AMR 명령 이력 조회 API
2. AMR 명령 실패 원인 조회
3. AMR Online/Offline 감지
   - odom과 `last_seen_at` 활용
4. AMR 제어 자동 회귀 테스트
   - Start
   - Lift Up/Down
   - Navigate
   - 이동 중 Stop
5. AMR 전용 Mission

```text
START
→ LIFT_UP
→ NAVIGATE
→ ARRIVED
→ LIFT_DOWN
→ STOP
→ COMPLETE
```

6. AMR 구간 박스 추적

```text
INPUT_ZONE → AMR_IN → P3020_IN
```

다만 AMR 전용 Mission이나 AMR 구간 추적을 전체 물류 Mission 완료라고 표현하면 안 된다.

---

## 10. 추천 다음 작업 순서

### 실제 E2E 완료 항목을 늘리는 방향

1. **AMR Online/Offline 및 명령 이력 API**
   - 다른 팀 작업 의존성 없음
   - 실제 데이터로 완전 검증 가능
2. **AMR 자동 E2E 회귀 테스트 스크립트**
   - 기존 완료 기능의 회귀 방지
3. **AMR 전용 Mission 상태 전환**
   - 현재 완성된 Start/Lift/Navigate 결과로 실제 검증 가능
4. **AMR 구간 Package Zone 추적**
   - `INPUT_ZONE → AMR_IN → P3020_IN`

### 전체 Control Tower 기반을 먼저 만드는 방향

1. Mission MQTT 이벤트 계약과 Backend 처리
2. Package Zone 이벤트 계약과 Backend 처리
3. 테스트 MQTT 발행기로 DB/API 검증
4. 팀원 작업 완료 후 실제 P3020/Conveyor/Sorter 이벤트 연결
5. 전체 System Start/Stop 대상 확장
6. map 기준 AMR 위치 표시

완료 상태를 엄격하게 유지하려면 첫 번째 방향을 우선 추천한다.

---

## 11. 전체 실행 방법

모든 터미널을 껐다가 다시 시작할 때 사용하는 순서다.

### 실행 순서

```text
터미널 5: 최신 코드 pull
→ 터미널 4: Docker/MQTT/Backend
→ 터미널 1: Isaac Sim
→ 터미널 2: Nav2
→ 터미널 3: ROS2 MQTT Adapter
→ 터미널 5: API/DB 테스트
```

### 터미널 5 — 최신 코드 확인

```bash
cd ~/collaboration/cobot3-ws-c2
git switch euiseok-control-tower
git status --short
git pull --ff-only origin euiseok-control-tower
git log -1 --oneline
```

### 터미널 4 — Docker, MQTT, Backend

```bash
cd ~/collaboration/cobot3-ws-c2
sudo docker compose up -d --build
sudo docker compose ps
sudo docker compose logs -f backend
```

### 터미널 1 — Isaac Sim

```bash
cd ~/collaboration/cobot3-ws-c2
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_isaac_mission.sh
```

다음 문구가 나올 때까지 기다린다.

```text
IW HUB CARGO + NAV2 + P3020 MISSION READY
```

### 터미널 2 — Nav2

터미널 1 준비 완료 후 실행한다.

```bash
cd ~/collaboration/cobot3-ws-c2
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./scripts/run_ros2.sh
```

### 터미널 3 — ROS2 MQTT Adapter

터미널 2의 Nav2가 완전히 올라온 뒤 실행한다.

```bash
cd ~/collaboration/cobot3-ws-c2
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/usr/bin/python3 scripts/ros2_mqtt_adapter.py
```

정상 구독 로그:

```text
controltower/command/amr/AMR_IN/navigate
controltower/command/amr/AMR_IN/lift
controltower/command/equipment/AMR_IN/control
```

---

## 12. 마지막 관찰 상태

마지막 테스트 당시:

- 터미널 1 Isaac Sim 실행 중
- 터미널 2 Nav2 실행 중
- 터미널 3 MQTT Adapter 실행 중
- 터미널 4 Docker/Backend 실행 중
- AMR 상태: `STOPPED`
- AMR 모드: `AUTO`
- Lift 상태: `DOWN`
- 로컬 odom 위치: 약 `x=-4.943`, `y=-0.541`
- yaw: 약 `3.040 rad`
- 마지막 명령:
  - `#30 START SUCCESS`
  - `#31 STOP SUCCESS`

다른 창에서 작업을 시작할 때는 실제 프로세스가 아직 살아 있는지 다시 확인하고, 죽어 있다면 위 실행 순서를 따른다.

---

## 13. 새 대화에서 바로 해야 할 일

1. GitHub `euiseok-control-tower` 브랜치 HEAD가 `03608b9f6878dea6e7b42897d1e29ba05b5534a9`인지 확인한다.
2. 사용자가 원하는 다음 방향을 확인한다.
   - 실제 E2E로 완전히 끝낼 수 있는 AMR Online/Offline·명령 이력
   - 또는 Mission/Package Backend 기반 구현
3. 코드를 직접 수정하고 테스트한 뒤 `euiseok-control-tower`에 커밋·푸시한다.
4. 사용자가 명령을 실행해야 할 때는 반드시 터미널 번호, 현재 상태, 실행 경로를 함께 알려준다.
5. 기존 Navigate, Lift, Start/Stop의 실제 E2E 동작을 보존한다.

