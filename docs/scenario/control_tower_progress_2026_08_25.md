# Control Tower Dashboard 개발 정리

작성 기준: 2026-08-25  
Repository: `https://github.com/rokey-c2/cobot3-ws-c2.git`  
Local path: `~/collaboration/cobot3-ws-c2`  
기준 Commit: `fd31fe0735cb278429088278d159915d89058b30`

---

# 1. 프로젝트 개요

본 프로젝트는 Isaac Sim + ROS2 기반 물류 자동화 시스템에 웹 기반 Control Tower Dashboard를 연결하는 것을 목표로 한다.

전체 시스템은 다음 흐름으로 구성한다.

```text
Isaac Sim
  ↓ ROS2 Bridge
ROS2 Jazzy
  ↓
ROS2 Adapter
  ↓
FastAPI
  ├─ REST API
  └─ WebSocket
  ↓
PostgreSQL
  ↓
React Control Tower
```

Control Tower의 핵심 역할은 크게 세 가지다.

1. 전체 물류 설비 상태 확인 및 제어
2. Mission / Package 진행 상황 추적
3. AMR 직접 제어

---

# 2. 현재 실제 물류 시나리오

현재 기준 시나리오는 다음과 같다.

```text
[Input Zone]
    ↓
Inbound AMR
    ↓
Package / Cargo 운반
    ↓
P3020 Manipulator 작업 위치 도착
    ↓
Manipulator Pick
    ↓
Manipulator Place
    ↓
Main Conveyor
    ↓
Sorter A
    ├─ Region A → DIVERT → REGION_A
    └─ 아닐 경우 PASS
          ↓
       Sorter B
          ├─ Region B → DIVERT → REGION_B
          └─ 아닐 경우 PASS
                ↓
             Sorter C
                ├─ Region C → DIVERT → REGION_C
                └─ 아닐 경우 PASS
                      ↓
                  EXCEPTION
```

중요한 점은 Sorter A/B/C가 하나의 분기장치처럼 동시에 선택되는 구조가 아니라는 것이다.

실제 분류는 순차적으로 수행한다.

```text
A 판단
→ 아니면 B 판단
→ 아니면 C 판단
→ 모두 아니면 Exception
```

예시:

```text
Region A Package
Sorter A → DIVERT

Region B Package
Sorter A → PASS
Sorter B → DIVERT

Region C Package
Sorter A → PASS
Sorter B → PASS
Sorter C → DIVERT

Unknown Package
Sorter A → PASS
Sorter B → PASS
Sorter C → PASS
→ EXCEPTION
```

---

# 3. Control Tower 최종 기능 범위

초기에는 여러 기능을 고려했지만, 1차 버전에서는 실제 프로젝트에서 의미 있는 기능만 남겼다.

최종 기능은 총 6개다.

| 기능 | 우선순위 | 적용 |
|---|---|---|
| 전체 / 개별 장비 Start / Stop | P0 | 반드시 |
| 장비 위치 및 현재 상태 | P0 | 반드시 |
| Mission 진행 상황 / 단계 | P0 | 반드시 |
| Package 실시간 상태 / 위치 추적 | P0 | 반드시 |
| 특정 위치까지 AMR 보내기 | P1 | 적용 |
| AMR Lift Up / Down | P1 | 적용 |

1차 버전에서 제외하거나 뒤로 미룬 기능:

- Manual Mode
- AMR 자유 수동 조종
- Top View Camera
- AMR Camera
- 정확한 Package 실시간 XY Tracking
- 장식용 Chart / Analytics
- AI Prediction
- Login / Signup
- 가짜 Battery 값
- 과도한 통계 기능

핵심은 “실제 로봇 상태와 실제 제어 흐름을 웹에서 보여주는 것”이다.

---

# 4. Package Tracking 방식

Package 위치를 매 프레임 XY 좌표로 추적하는 방식은 사용하지 않는다.

대신 Zone 단위로 추적한다.

최종 Zone 후보:

```text
INPUT_ZONE
AMR_IN
P3020_IN
MAIN_CONVEYOR
SORTER_A
SORTER_B
SORTER_C
REGION_A
REGION_B
REGION_C
EXCEPTION
```

예:

```text
PKG-001
Region: B
Current Zone: SORTER_B
Status: IN_PROGRESS
```

이 방식은 실제 공정 흐름을 표현하기 쉽고, Dashboard 구현도 단순하며, 포트폴리오 설명에도 적합하다.

---

# 5. 전체 Dashboard 페이지 구조

최종적으로 Control Tower는 총 3페이지로 구성한다.

```text
CONTROL TOWER

[ Dashboard ]   [ Packages ]   [ AMR Control ]
```

각 페이지의 역할을 명확히 분리한다.

---

# 6. Page 1 - Dashboard

Dashboard는 메인 페이지다.

가장 중요한 P0 기능인

- 전체 / 개별 Start / Stop
- Mission 진행 상황

을 가장 크게 보여준다.

권장 레이아웃:

```text
┌────────────────────────────────────────────────────────────┐
│ CONTROL TOWER                SYSTEM STATUS : RUNNING       │
├────────────────────────────────────────────────────────────┤
│                                                            │
│   [ SYSTEM START ]             [ SYSTEM STOP ]             │
│                                                            │
├────────────────────────────────────────────────────────────┤
│ EQUIPMENT STATUS                                           │
│                                                            │
│ AMR_IN          ● RUNNING    [START] [STOP]               │
│ P3020_IN        ● RUNNING    [START] [STOP]               │
│ MAIN_CONVEYOR   ● RUNNING    [START] [STOP]               │
│ SORTER_A        ● RUNNING    [START] [STOP]               │
│ SORTER_B        ● RUNNING    [START] [STOP]               │
│ SORTER_C        ● RUNNING    [START] [STOP]               │
├────────────────────────────────────────────────────────────┤
│ CURRENT MISSION                                            │
│                                                            │
│ AMR_PICKUP → AMR_NAVIGATION → MANIPULATOR_PICK → ...      │
│     ✓              ●                 ○                    │
│                                                            │
│ Progress ███████████░░░░░░ 45%                            │
├────────────────────────────────────────────────────────────┤
│ 최근 Package 요약          │ 최근 Event Log               │
└────────────────────────────────────────────────────────────┘
```

Dashboard에서 가장 먼저 확인해야 하는 질문은 두 가지다.

```text
1. 시스템이 현재 정상 동작 중인가?
2. 현재 Mission이 어느 단계까지 진행됐는가?
```

따라서 이 두 항목을 메인 페이지에서 가장 크게 보여준다.

---

# 7. Dashboard 장비 상태 표현

장비 상태는 단순하고 직관적으로 표현한다.

```text
초록색  ● = RUNNING
회색    ● = STOPPED
주황색  ● = PENDING / IN_PROGRESS
빨간색  ● = ERROR
```

현재 관리 대상 장비는 6개다.

```text
AMR_IN
P3020_IN
MAIN_CONVEYOR
SORTER_A
SORTER_B
SORTER_C
```

AMR은 추가 정보를 표시한다.

```text
AMR_IN
Status : RUNNING
Mode   : AUTO
Lift   : DOWN
```

---

# 8. Page 2 - Packages

Package Tracking 전용 페이지다.

Package가 여러 개로 증가할 경우 Dashboard에 모든 Package를 표시하면 화면이 복잡해지므로 별도 페이지로 분리한다.

기본 화면:

```text
PACKAGES

┌──────────────────────────────────────────────────────┐
│ Package       Region      Current Zone       Status  │
├──────────────────────────────────────────────────────┤
│ PKG-001         A         REGION_A           DONE    │
│ PKG-002         B         SORTER_B           MOVING  │
│ PKG-003         C         MAIN_CONVEYOR      MOVING  │
│ PKG-004       UNKNOWN     SORTER_C           MOVING  │
└──────────────────────────────────────────────────────┘
```

Package를 선택하면 상세 이동 이력을 표시한다.

```text
PKG-002

Region : B
Status : IN_PROGRESS

INPUT_ZONE
    ↓
AMR_IN
    ↓
P3020_IN
    ↓
MAIN_CONVEYOR
    ↓
SORTER_A
    PASS
    ↓
SORTER_B
    CURRENT
    ↓
REGION_B
```

Package 상세에서 확인할 항목:

- Package Code
- Region
- Current Zone
- Current Status
- Mission Code
- 이동 이력
- Sorter PASS / DIVERT 기록
- 최종 분류 결과

---

# 9. Page 3 - AMR Control

AMR Navigate와 Lift 기능은 서로 다른 페이지로 나누지 않는다.

두 기능 모두 같은 AMR를 직접 제어하는 기능이기 때문에 한 페이지에 묶는다.

기본 구조:

```text
AMR CONTROL

AMR_IN
● RUNNING
Mode : AUTO
Lift : DOWN

Current Position
X   : -
Y   : -
Yaw : -

──────────────────────────────

Navigation

Target X
[ 1.30 ]

Target Y
[ -0.06 ]

Yaw
[ 0.00 ]

[ NAVIGATE ]

──────────────────────────────

Lift Control

Current Lift : DOWN

[ ▲ LIFT UP ]     [ ▼ LIFT DOWN ]

──────────────────────────────

Command Status

NAVIGATE
Target : (1.30, -0.06)
Status : PENDING
```

AMR Control 페이지에서 제공할 기능:

- 현재 AMR 상태 확인
- 실제 Position X / Y / Yaw 표시
- Target X 입력
- Target Y 입력
- Target Yaw 입력
- Navigate 명령
- Lift Up
- Lift Down
- 최근 Command 상태 확인

---

# 10. 디자인 방향

전체 UI는 “산업용 관제실 + 물류 Control Tower” 느낌으로 구성한다.

과도한 SF 스타일보다 실제 산업 Dashboard에 가까운 형태를 목표로 한다.

스타일:

```text
Theme        Dark Industrial Dashboard

Background   짙은 Navy / Black
Panel        Dark Gray
Primary      Blue
RUNNING      Green
STOPPED      Gray
PENDING      Orange
ERROR        Red
```

핵심 원칙:

- 한눈에 상태가 보여야 함
- 버튼 위치가 명확해야 함
- 불필요한 그래프를 넣지 않음
- 정보 밀도는 높지만 복잡해 보이지 않게 구성
- Dashboard와 실제 로봇 상태가 연결된다는 느낌을 강조

---

# 11. ERD 구조

Control Tower용 Database는 총 8개 Table로 구성했다.

```text
equipment
equipment_state
equipment_command
mission
mission_stage
package
zone
package_event
```

관계:

```text
equipment ||--|| equipment_state
equipment ||--o{ equipment_command

mission ||--o{ mission_stage
mission ||--o{ package

zone ||--o{ package
package ||--o{ package_event
zone ||--o{ package_event
```

---

# 12. equipment

장비 기본 정보.

주요 Field:

```text
id
code
name
type
enabled
created_at
```

현재 등록 장비:

```text
AMR_IN
P3020_IN
MAIN_CONVEYOR
SORTER_A
SORTER_B
SORTER_C
```

---

# 13. equipment_state

장비의 최신 상태만 저장한다.

주요 Field:

```text
equipment_id
status
mode
position_x
position_y
yaw
lift_state
last_seen_at
updated_at
```

AMR 예시:

```text
status     = RUNNING
mode       = AUTO
position_x = 실제 ROS 값
position_y = 실제 ROS 값
yaw        = 실제 ROS 값
lift_state = DOWN
```

`yaw`는 AMR이 어느 방향을 보고 있는지를 나타낸다.

ROS에서는 일반적으로 radian을 사용한다.

```text
0       ≈ 0°
1.57    ≈ 90°
3.14    ≈ 180°
-1.57   ≈ -90°
```

---

# 14. equipment_command

Dashboard에서 발생한 장비 제어 명령을 저장한다.

Field:

```text
id
equipment_id
command_type
command_payload
status
requested_at
completed_at
error_message
```

현재 Command Type:

```text
START
STOP
NAVIGATE
LIFT_UP
LIFT_DOWN
```

예:

```text
NAVIGATE

command_payload:
{
    "x": 1.3,
    "y": -0.06,
    "yaw": 0.0
}
```

---

# 15. mission

전체 작업 단위.

Field:

```text
id
mission_code
status
current_stage
started_at
completed_at
created_at
```

현재 테스트 Mission:

```text
MISSION-TEST-B001
```

---

# 16. mission_stage

Mission 내부의 세부 단계.

현재 B Region 테스트 Mission Stage:

```text
1. AMR_PICKUP
2. AMR_NAVIGATION
3. MANIPULATOR_PICK
4. MANIPULATOR_PLACE
5. MAIN_CONVEYOR
6. SORTER_A
7. SORTER_B
8. REGION_B
9. COMPLETE
```

Stage Status:

```text
WAITING
RUNNING
COMPLETED
FAILED
```

---

# 17. package

Package의 현재 상태를 저장한다.

Field:

```text
id
package_code
mission_id
current_zone_id
region
status
created_at
updated_at
```

현재 테스트 Package:

```text
PKG-TEST-B001
Region: B
```

---

# 18. zone

Package가 위치할 수 있는 공정 Zone을 정의한다.

현재 Zone:

```text
INPUT_ZONE
AMR_IN
P3020_IN
MAIN_CONVEYOR
SORTER_A
SORTER_B
SORTER_C
REGION_A
REGION_B
REGION_C
EXCEPTION
```

---

# 19. package_event

Package가 각 Zone을 이동할 때 발생한 이벤트를 기록한다.

예:

```text
PKG-TEST-B001
INPUT_ZONE
ENTER
SUCCESS
```

이후 예:

```text
SORTER_A
PASS
SUCCESS

SORTER_B
DIVERT
SUCCESS

REGION_B
ENTER
SUCCESS
```

---

# 20. Database 구현 완료 내용

Docker 기반 PostgreSQL 환경을 구축했다.

사용 환경:

```text
PostgreSQL 16
Docker
Docker Compose
```

Container:

```text
control_tower_postgres
```

Database:

```text
control_tower
```

User:

```text
controltower
```

관련 파일:

```text
backend/db/
├── init.sql
├── seed.sql
└── test_mission.sql
```

`init.sql`

- 8개 Table 생성
- Foreign Key 설정
- Index 생성

`seed.sql`

- 6개 Equipment
- Equipment State 초기값
- 11개 Zone

`test_mission.sql`

- 테스트 Mission
- Mission Stage
- Package
- Package Event

생성 완료.

---

# 21. Backend 구성

현재 Backend는 FastAPI 기반이다.

관련 구조:

```text
backend/
├── app/
│   ├── api/
│   │   ├── equipment.py
│   │   ├── events.py
│   │   ├── missions.py
│   │   ├── packages.py
│   │   ├── system.py
│   │   └── amr.py
│   ├── database.py
│   └── main.py
│
├── db/
│   ├── init.sql
│   ├── seed.sql
│   └── test_mission.sql
│
├── Dockerfile
└── requirements.txt
```

Backend Container:

```text
control_tower_backend
```

Port:

```text
8000
```

Swagger:

```text
http://localhost:8000/docs
```

---

# 22. 현재 구현된 Read API

현재까지 확인 완료된 조회 API:

```text
GET /api/equipment
GET /api/missions/current
GET /api/packages
GET /api/packages/{package_code}
GET /api/events
```

---

# 23. Equipment 조회 API

```text
GET /api/equipment
```

현재 6개 장비와 상태를 반환한다.

예:

```json
{
    "code": "AMR_IN",
    "status": "RUNNING",
    "mode": "AUTO",
    "position_x": null,
    "position_y": null,
    "yaw": null,
    "lift_state": "DOWN"
}
```

현재 position이 `null`인 이유는 아직 ROS2 실제 위치 데이터를 Backend에 연결하지 않았기 때문이다.

---

# 24. 개별 Equipment Start / Stop

현재 구현 완료:

```text
POST /api/equipment/{equipment_code}/start
POST /api/equipment/{equipment_code}/stop
```

예:

```text
POST /api/equipment/AMR_IN/start
```

결과:

```text
AMR_IN
status = RUNNING
```

STOP:

```text
POST /api/equipment/AMR_IN/stop
```

결과:

```text
AMR_IN
status = STOPPED
```

현재 이 기능은 Mock Control이다.

즉 아직 ROS2 장비를 실제로 움직이는 것이 아니라:

```text
HTTP API
→ equipment_command
→ equipment_state
```

까지만 연결되어 있다.

---

# 25. System Start / Stop

현재 구현 완료:

```text
POST /api/system/start
POST /api/system/stop
```

System Start 실행 시 현재 6개 장비:

```text
AMR_IN
P3020_IN
MAIN_CONVEYOR
SORTER_A
SORTER_B
SORTER_C
```

모두 `RUNNING`으로 변경된다.

테스트 결과:

```json
{
    "command": "START",
    "status": "SUCCESS",
    "equipment_count": 6
}
```

System Stop 실행 시:

```text
6개 장비 → STOPPED
```

정상 확인 완료.

주의:

현재 System Start / Stop 역시 실제 ROS2 제어가 아닌 Mock 상태 전환이다.

---

# 26. AMR Navigate API

현재 구현 완료:

```text
POST /api/amr/{equipment_code}/navigate
```

예:

```text
POST /api/amr/AMR_IN/navigate
```

Payload:

```json
{
    "x": 1.3,
    "y": -0.06,
    "yaw": 0.0
}
```

현재 테스트 결과:

```text
command_type = NAVIGATE
status       = PENDING
```

Database 저장 결과:

```text
AMR_IN
NAVIGATE
{"x": 1.3, "y": -0.06, "yaw": 0.0}
PENDING
```

중요:

Navigate API를 호출했다고 해서

```text
equipment_state.position_x = 1.3
equipment_state.position_y = -0.06
```

로 변경하지 않는다.

그 값은 목표 좌표일 뿐 실제 AMR 위치가 아니다.

실제 위치는 이후:

```text
Isaac Sim
→ ROS2 /odom
→ ROS2 Adapter
→ equipment_state
```

경로로 들어와야 한다.

---

# 27. AMR Lift API

현재 구현 완료:

```text
POST /api/amr/{equipment_code}/lift
```

Lift Up:

```json
{
    "action": "UP"
}
```

Database Command:

```text
LIFT_UP
PENDING
```

실제 확인된 결과:

```text
AMR_IN
LIFT_UP
{"action": "UP"}
PENDING
```

현재 `lift_state`는 계속:

```text
DOWN
```

으로 유지한다.

이유는 아직 실제 Isaac Sim / ROS2에서 Lift가 올라간 것을 확인하지 않았기 때문이다.

최종 구조:

```text
Dashboard
→ LIFT_UP Command
→ PENDING
→ ROS2 Adapter
→ 실제 AMR Lift 동작
→ 성공 확인
→ Command SUCCESS
→ equipment_state.lift_state = UP
```

---

# 28. Command 상태 설계

현재 중요한 원칙:

```text
START / STOP
현재 Mock 단계에서는 SUCCESS 처리

NAVIGATE
실제 ROS2 실행 전이므로 PENDING

LIFT_UP / LIFT_DOWN
실제 ROS2 실행 전이므로 PENDING
```

향후 ROS2 Adapter가 연결되면 Start / Stop 역시 즉시 SUCCESS 처리하지 않고 실제 장비 응답을 확인하는 구조로 변경할 예정이다.

최종 흐름:

```text
Command 요청
    ↓
PENDING
    ↓
ROS2 Adapter 실행
    ↓
실제 장비 동작
    ↓
성공
  ↙   ↘
SUCCESS ERROR
```

---

# 29. 현재 발견된 개선사항

## 29.1 동일 AMR에 NAVIGATE Command 중복

테스트 과정에서 동일한 AMR에 NAVIGATE 요청을 두 번 보내자:

```text
id 18 NAVIGATE PENDING
id 19 NAVIGATE PENDING
```

처럼 여러 Pending Command가 저장됐다.

현재 테스트 단계에서는 문제없지만, 실제 ROS2 연결 전에 다음 로직이 필요하다.

```text
AMR_IN에 이미 NAVIGATE PENDING / RUNNING 존재
        ↓
새 NAVIGATE 요청
        ↓
차단 또는 기존 Command 취소
```

즉 실제 운영 단계에서는 Command Queue 또는 Command Lock이 필요하다.

---

# 30. React Frontend 현재 상태

기준 Commit에서 `frontend/`는 사실상 초기 Placeholder 상태다.

구조:

```text
frontend/
├── package.json
└── src/
    └── README.md
```

현재 `package.json`은 최소 정보만 존재한다.

따라서 React UI는 기존 화면을 수정하는 방식이 아니라 처음부터 Control Tower 기준으로 구성하면 된다.

예정 기술:

```text
React
Vite
REST API
WebSocket
```

---

# 31. Frontend 구현 순서

최종 3페이지 기준으로 진행한다.

## STEP 3-1

React + Vite 초기 환경 생성

## STEP 3-2

공통 Layout

```text
Header
Navigation
Page Container
Status Color
```

## STEP 3-3

Dashboard 페이지

- System Start / Stop
- Equipment Cards
- Mission Progress
- Package Summary
- Event Log

## STEP 3-4

Packages 페이지

- Package List
- Package Detail
- Zone History
- Sorter Result

## STEP 3-5

AMR Control 페이지

- Current Position
- Navigate Input
- Lift Control
- Command Status

## STEP 3-6

FastAPI REST API 연결

## STEP 3-7

WebSocket 실시간 갱신

---

# 32. 이후 ROS2 연결 구조

Frontend와 Backend가 완성된 이후 ROS2 Adapter를 연결한다.

Robot → Dashboard:

```text
Isaac Sim
    ↓
ROS2 Bridge
    ↓
ROS2 Topic / Action
    ↓
ROS2 Adapter
    ↓
FastAPI / Database
    ↓
WebSocket
    ↓
React Dashboard
```

Dashboard → Robot:

```text
React
    ↓
FastAPI
    ↓
equipment_command
    ↓
ROS2 Adapter
    ↓
ROS2 Topic / Action
    ↓
Isaac Sim
```

---

# 33. 향후 실제 데이터 연결 대상

AMR:

```text
Position X
Position Y
Yaw
Status
Lift State
Navigation Result
```

Manipulator:

```text
Status
Pick State
Place State
Action Result
```

Conveyor:

```text
Running / Stopped
```

Sorter:

```text
Running / Stopped
PASS
DIVERT
```

Mission:

```text
current_stage
stage status
progress
```

Package:

```text
current_zone
region
status
event history
```

---

# 34. 최종 포트폴리오 설명 방향

이 Control Tower는 단순한 웹 Dashboard가 아니다.

핵심 설명은 다음과 같이 가져갈 수 있다.

> Isaac Sim에서 동작하는 AMR, 협동로봇, Conveyor, Sorter의 상태를 ROS2를 통해 수집하고, FastAPI와 PostgreSQL을 통해 상태와 작업 이력을 관리하며, React 기반 Control Tower에서 전체 물류 공정의 상태, Mission 진행 상황, Package 위치를 실시간으로 확인하고 AMR를 직접 제어할 수 있도록 구성한 물류 자동화 관제 시스템이다.

핵심 포인트:

```text
1. 실제 로봇 상태 수집
2. Mission 진행 상태 관리
3. Package Zone Tracking
4. Web 기반 Start / Stop
5. AMR Navigation
6. AMR Lift Control
7. Command 이력 저장
8. ROS2 ↔ Backend ↔ React 통합
```

---

# 35. 현재까지 완료 상태

```text
[완료] Control Tower 기능 범위 결정

[완료] 최종 3페이지 구조 결정
        Dashboard
        Packages
        AMR Control

[완료] ERD 설계

[완료] PostgreSQL Docker 환경

[완료] 8개 Table 생성

[완료] Equipment / Zone Seed

[완료] Test Mission / Package 생성

[완료] FastAPI 기본 Backend

[완료] DB Health Check

[완료] Equipment 조회

[완료] Mission 조회

[완료] Package 조회

[완료] Package Detail 조회

[완료] Event 조회

[완료] 개별 Equipment Start / Stop

[완료] 전체 System Start / Stop

[완료] AMR Navigate Command 등록

[완료] AMR Lift Up Command 등록

[진행 예정] React Frontend

[진행 예정] WebSocket

[진행 예정] ROS2 Adapter

[진행 예정] 실제 AMR Position 연동

[진행 예정] 실제 Navigation 실행

[진행 예정] 실제 Lift 실행

[진행 예정] Mission 자동 상태 갱신

[진행 예정] Package Zone 자동 Tracking

[진행 예정] 최종 통합 테스트
```

---

# 36. 현재 개발 단계 요약

현재 위치는 다음과 같다.

```text
DB
✓

FastAPI Read API
✓

FastAPI Mock Control
✓

AMR Command Queue 저장
✓

React
← 다음 단계

WebSocket
아직

ROS2 Adapter
아직

실제 Isaac Sim 제어
아직
```

즉 현재까지는 Control Tower의 Backend 기반 구조와 Command 설계까지 완료했고, 다음 작업은 **3페이지 React UI를 생성하고 현재 FastAPI API와 연결하는 단계**다.
