# AMR Pose Sync 연동 완료 정리

## 1. 작업 개요

Isaac Sim의 AMR 위치 정보를 ROS2, RViz2, MQTT, Backend, PostgreSQL, Web까지 동기화하는 작업을 완료했다.

최종 확인 흐름:

```text
Isaac Sim
   ↓
/amr_a/map_pose
   ↓
ROS2 / TF / RViz2
   ↓
Pose Sync Manager
   ↓
MQTT
   ↓
Backend API
   ↓
PostgreSQL
   ↓
React Web
```

---

## 2. 기준 브랜치

현재 작업 기준은 `main` 브랜치이다.

현재 `main`은 기존 `euiseok-control-tower` 최신 상태를 기준으로 맞춰져 있다.

```text
5d93eb2 feat: enforce pose sync control interlocks
```

기존 `main`은 아래 백업 브랜치에 보존했다.

```text
backup/main-before-control-tower-20260826
```

---

## 3. 실행 순서

### 터미널 1 — Docker / Backend / DB / MQTT

```bash
cd ~/collaboration/cobot3-ws-c2
sudo docker compose up -d --build
```

상태 확인:

```bash
sudo docker compose ps
```

Docker Compose에서 실행되는 주요 서비스:

- PostgreSQL
- Mosquitto MQTT
- Backend

Backend API 포트:

```text
8000
```

---

### 터미널 2 — Web Frontend

```bash
cd ~/collaboration/cobot3-ws-c2/frontend
npm run dev
```

처음 실행하거나 `node_modules`가 없을 경우:

```bash
npm install
npm run dev
```

웹 접속 주소:

```text
http://localhost:5173
```

---

### 터미널 3 — Isaac Sim

```bash
cd ~/collaboration/cobot3-ws-c2

export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

./scripts/run_isaac_mission.sh
```

Isaac Sim과 미션 환경이 완전히 준비된 뒤 다음 단계로 진행한다.

---

### 터미널 4 — ROS2 / Nav2 / RViz2

Isaac 실행 완료 후:

```bash
cd ~/collaboration/cobot3-ws-c2

export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

./scripts/run_ros2.sh
```

---

### 터미널 5 — Pose Sync

Isaac + ROS2가 모두 정상 실행된 뒤:

```bash
cd ~/collaboration/cobot3-ws-c2

./scripts/run_pose_sync.sh
```

정상 실행 시 주요 로그:

```text
[POSE SYNC] ROS_DOMAIN_ID=110
[POSE SYNC] Waiting for Isaac /amr_a/map_pose
[POSE SYNC] Canonical frame: map
[POSE SYNC][MQTT] Connected reason_code=Success

Canonical pose input: /amr_a/map_pose
Isaac session input: /amr_a/pose_source_session
Isaac restore output: /amr_a/restore_pose
Canonical MQTT pose: controltower/amr/AMR_IN/pose
```

---

## 4. Pose Sync 주요 인터페이스

### Isaac → Pose Sync

```text
/amr_a/map_pose
```

Canonical frame:

```text
map
```

### Pose Restore

```text
/amr_a/restore_pose
```

### Isaac Session Source

```text
/amr_a/pose_source_session
```

### MQTT Pose Topic

```text
controltower/amr/AMR_IN/pose
```

### MQTT Restore Response Topic

```text
controltower/amr/AMR_IN/pose_restore/response
```

---

## 5. 최종 검증 결과

최종 테스트 당시 Isaac Pose:

```text
x = 2.207003
y = -0.660002
yaw ≈ 2.487 rad
frame = map
```

Web 화면:

```text
X    = 2.208 m
Y    = -0.660 m
Yaw  = 2.487 rad

Pose Sync = SYNCED
Frame     = map
Source    = ISAAC_WORLD
Mode      = AUTO
Lift      = UP
```

Backend API:

```text
position_x = 2.207
position_y = -0.660
yaw        = 2.4869

pose_frame  = map
pose_source = ISAAC_WORLD
sync_status = SYNCED
lift_state  = UP
```

PostgreSQL:

```text
AMR_IN
position_x = 2.207
position_y = -0.660
yaw        = 2.4869
pose_frame = map
pose_source = ISAAC_WORLD
sync_status = SYNCED
```

Web의 X 좌표 `2.208`과 Backend/DB의 `2.207` 사이 약 1 mm 차이는
각 데이터를 확인한 시점 차이로 인해 발생한 정상 범위의 차이다.

---

## 6. TF / RViz2 검증

확인 명령:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

테스트 당시:

```text
Translation:
x = 2.222
y = -0.641

Yaw:
2.474 rad
141.768 deg
```

`/amr_a/map_pose` 조회 시점과 TF 조회 시점 사이에 시간이 있었기 때문에
AMR 이동에 따른 소폭의 차이가 존재했으며 정상적으로 판단했다.

---

## 7. Backend 확인 명령

```bash
curl -s http://localhost:8000/api/equipment | python3 -m json.tool
```

정상 상태 기준:

```text
code        = AMR_IN
pose_frame  = map
pose_source = ISAAC_WORLD
sync_status = SYNCED
```

---

## 8. DB 확인 명령

```bash
cd ~/collaboration/cobot3-ws-c2

sudo docker compose exec -T postgres sh -lc '
psql -U "$POSTGRES_USER" -d "$POSTGRES_DB" -P pager=off -c "
SELECT
    e.code,
    es.position_x,
    es.position_y,
    es.yaw,
    es.pose_frame,
    es.pose_source,
    es.pose_seq,
    es.sync_status,
    es.pose_updated_at
FROM equipment e
JOIN equipment_state es ON es.equipment_id = e.id
WHERE e.code = '\''AMR_IN'\'';
"
'
```

---

## 9. 최종 완료 상태

다음 항목을 모두 확인했다.

- Isaac Sim AMR 위치 정상
- `/amr_a/map_pose` 정상 발행
- `map → base_link` TF 정상
- RViz2 위치 정상
- Pose Sync Manager 정상
- MQTT 연결 정상
- Backend API 반영 정상
- PostgreSQL 저장 정상
- Web 화면 좌표 반영 정상
- `sync_status = SYNCED`
- `pose_source = ISAAC_WORLD`
- Lift 상태 Web 반영 정상

## 결론

**AMR Pose Sync 전체 연동 작업 완료.**

Isaac Sim의 AMR 위치 및 상태가 ROS2/RViz2를 거쳐 관제 Backend, DB, Web까지 정상적으로 동기화되는 것을 최종 확인했다.
