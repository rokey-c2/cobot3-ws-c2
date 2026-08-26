-- ============================================================
-- Control Tower v1 Database Schema
-- PostgreSQL 16
-- ============================================================

-- ------------------------------------------------------------
-- 1. equipment
-- 장비의 고정 기본 정보
-- ------------------------------------------------------------
CREATE TABLE IF NOT EXISTS equipment (
    id BIGSERIAL PRIMARY KEY,
    code VARCHAR(50) NOT NULL UNIQUE,
    name VARCHAR(100) NOT NULL,
    type VARCHAR(30) NOT NULL,
    enabled BOOLEAN NOT NULL DEFAULT TRUE,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);


-- ------------------------------------------------------------
-- 2. zone
-- Package가 위치할 수 있는 논리적인 공정 구역
-- ------------------------------------------------------------
CREATE TABLE IF NOT EXISTS zone (
    id BIGSERIAL PRIMARY KEY,
    zone_code VARCHAR(50) NOT NULL UNIQUE,
    name VARCHAR(100) NOT NULL,
    zone_type VARCHAR(30) NOT NULL,
    map_x NUMERIC(10, 3),
    map_y NUMERIC(10, 3)
);


-- ------------------------------------------------------------
-- 3. mission
-- 하나의 물류 작업 단위
-- ------------------------------------------------------------
CREATE TABLE IF NOT EXISTS mission (
    id BIGSERIAL PRIMARY KEY,
    mission_code VARCHAR(100) NOT NULL UNIQUE,
    status VARCHAR(30) NOT NULL DEFAULT 'READY',
    current_stage VARCHAR(50),
    started_at TIMESTAMPTZ,
    completed_at TIMESTAMPTZ,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);


-- ------------------------------------------------------------
-- 4. equipment_state
-- 각 장비의 현재 상태 Snapshot
-- equipment와 1:1 관계
-- ------------------------------------------------------------
CREATE TABLE IF NOT EXISTS equipment_state (
    equipment_id BIGINT PRIMARY KEY,

    status VARCHAR(30) NOT NULL DEFAULT 'IDLE',
    mode VARCHAR(30),

    position_x NUMERIC(10, 3),
    position_y NUMERIC(10, 3),
    yaw NUMERIC(10, 4),

    -- Canonical AMR pose metadata.
    -- position_x / position_y / yaw are official only when pose_frame='map'.
    pose_frame VARCHAR(20),
    pose_source VARCHAR(30),
    pose_seq BIGINT NOT NULL DEFAULT 0,
    pose_session_id VARCHAR(100),
    pose_session_epoch_ms BIGINT NOT NULL DEFAULT 0,
    pose_updated_at TIMESTAMPTZ,
    sync_status VARCHAR(20) NOT NULL DEFAULT 'OFFLINE',

    lift_state VARCHAR(30),

    last_seen_at TIMESTAMPTZ,
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT fk_equipment_state_equipment
        FOREIGN KEY (equipment_id)
        REFERENCES equipment(id)
        ON DELETE CASCADE
);


-- Legacy /chassis/odom updates do not carry canonical pose metadata.
-- Once a canonical map pose exists, protect it from any writer that changes
-- x/y/yaw without advancing pose_seq/session_epoch.
CREATE OR REPLACE FUNCTION protect_canonical_map_pose()
RETURNS TRIGGER AS $$
BEGIN
    IF
        OLD.pose_frame = 'map'
        AND NEW.pose_seq = OLD.pose_seq
        AND NEW.pose_session_epoch_ms = OLD.pose_session_epoch_ms
        AND (
            NEW.position_x IS DISTINCT FROM OLD.position_x
            OR NEW.position_y IS DISTINCT FROM OLD.position_y
            OR NEW.yaw IS DISTINCT FROM OLD.yaw
        )
    THEN
        NEW.position_x := OLD.position_x;
        NEW.position_y := OLD.position_y;
        NEW.yaw := OLD.yaw;
    END IF;

    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

DROP TRIGGER IF EXISTS trg_protect_canonical_map_pose
    ON equipment_state;

CREATE TRIGGER trg_protect_canonical_map_pose
BEFORE UPDATE ON equipment_state
FOR EACH ROW
EXECUTE FUNCTION protect_canonical_map_pose();


-- ------------------------------------------------------------
-- 5. equipment_command
-- Dashboard에서 장비로 전달한 명령 기록
-- START / STOP / NAVIGATE / LIFT_UP / LIFT_DOWN
-- ------------------------------------------------------------
CREATE TABLE IF NOT EXISTS equipment_command (
    id BIGSERIAL PRIMARY KEY,

    equipment_id BIGINT NOT NULL,

    command_type VARCHAR(30) NOT NULL,
    command_payload JSONB,

    status VARCHAR(30) NOT NULL DEFAULT 'PENDING',

    requested_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    completed_at TIMESTAMPTZ,

    error_message TEXT,

    CONSTRAINT fk_equipment_command_equipment
        FOREIGN KEY (equipment_id)
        REFERENCES equipment(id)
        ON DELETE CASCADE
);


-- ------------------------------------------------------------
-- 6. mission_stage
-- Mission의 단계별 진행상황
-- ------------------------------------------------------------
CREATE TABLE IF NOT EXISTS mission_stage (
    id BIGSERIAL PRIMARY KEY,

    mission_id BIGINT NOT NULL,

    stage_code VARCHAR(50) NOT NULL,
    sequence_no INTEGER NOT NULL,

    status VARCHAR(30) NOT NULL DEFAULT 'WAITING',

    started_at TIMESTAMPTZ,
    completed_at TIMESTAMPTZ,

    CONSTRAINT fk_mission_stage_mission
        FOREIGN KEY (mission_id)
        REFERENCES mission(id)
        ON DELETE CASCADE,

    CONSTRAINT uq_mission_stage_sequence
        UNIQUE (mission_id, sequence_no)
);


-- ------------------------------------------------------------
-- 7. package
-- Package의 현재 상태 및 현재 Zone
-- ------------------------------------------------------------
CREATE TABLE IF NOT EXISTS package (
    id BIGSERIAL PRIMARY KEY,

    package_code VARCHAR(100) NOT NULL UNIQUE,

    mission_id BIGINT,
    current_zone_id BIGINT,

    region VARCHAR(30) NOT NULL DEFAULT 'UNKNOWN',
    status VARCHAR(30) NOT NULL DEFAULT 'WAITING',

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT fk_package_mission
        FOREIGN KEY (mission_id)
        REFERENCES mission(id)
        ON DELETE SET NULL,

    CONSTRAINT fk_package_zone
        FOREIGN KEY (current_zone_id)
        REFERENCES zone(id)
        ON DELETE SET NULL
);


-- ------------------------------------------------------------
-- 8. package_event
-- Package가 지나온 Zone과 분류 결과 History
-- ------------------------------------------------------------
CREATE TABLE IF NOT EXISTS package_event (
    id BIGSERIAL PRIMARY KEY,

    package_id BIGINT NOT NULL,
    zone_id BIGINT,

    event_type VARCHAR(50) NOT NULL,
    result VARCHAR(30),

    occurred_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT fk_package_event_package
        FOREIGN KEY (package_id)
        REFERENCES package(id)
        ON DELETE CASCADE,

    CONSTRAINT fk_package_event_zone
        FOREIGN KEY (zone_id)
        REFERENCES zone(id)
        ON DELETE SET NULL
);


-- ============================================================
-- Indexes
-- ============================================================

CREATE INDEX IF NOT EXISTS idx_equipment_type
    ON equipment(type);

CREATE INDEX IF NOT EXISTS idx_equipment_command_equipment_id
    ON equipment_command(equipment_id);

CREATE INDEX IF NOT EXISTS idx_equipment_command_status
    ON equipment_command(status);

CREATE INDEX IF NOT EXISTS idx_mission_status
    ON mission(status);

CREATE INDEX IF NOT EXISTS idx_mission_stage_mission_id
    ON mission_stage(mission_id);

CREATE INDEX IF NOT EXISTS idx_package_mission_id
    ON package(mission_id);

CREATE INDEX IF NOT EXISTS idx_package_current_zone_id
    ON package(current_zone_id);

CREATE INDEX IF NOT EXISTS idx_package_region
    ON package(region);

CREATE INDEX IF NOT EXISTS idx_package_event_package_id
    ON package_event(package_id);

CREATE INDEX IF NOT EXISTS idx_package_event_occurred_at
    ON package_event(occurred_at);
