-- Canonical Map Pose migration for an existing Control Tower database.
-- Safe to run more than once.

ALTER TABLE equipment_state
    ADD COLUMN IF NOT EXISTS pose_frame VARCHAR(20);

ALTER TABLE equipment_state
    ADD COLUMN IF NOT EXISTS pose_source VARCHAR(30);

ALTER TABLE equipment_state
    ADD COLUMN IF NOT EXISTS pose_seq BIGINT NOT NULL DEFAULT 0;

ALTER TABLE equipment_state
    ADD COLUMN IF NOT EXISTS pose_session_id VARCHAR(100);

ALTER TABLE equipment_state
    ADD COLUMN IF NOT EXISTS pose_session_epoch_ms BIGINT NOT NULL DEFAULT 0;

ALTER TABLE equipment_state
    ADD COLUMN IF NOT EXISTS pose_updated_at TIMESTAMPTZ;

ALTER TABLE equipment_state
    ADD COLUMN IF NOT EXISTS sync_status VARCHAR(20) NOT NULL DEFAULT 'OFFLINE';


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
