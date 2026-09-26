-- Insert the arrival milestone into existing routes without resetting history.
LOCK TABLE mission_stage IN SHARE ROW EXCLUSIVE MODE;
DO $$
DECLARE
    route RECORD;
    later_stage RECORD;
BEGIN
    FOR route IN
        SELECT s.mission_id, s.sequence_no, s.started_at
        FROM mission_stage s
        WHERE s.stage_code = 'MANIPULATOR_PICK'
          AND NOT EXISTS (
              SELECT 1 FROM mission_stage a
              WHERE a.mission_id = s.mission_id AND a.stage_code = 'AMR_ARRIVAL'
          )
    LOOP
        -- Descending updates preserve the unique mission/sequence constraint.
        FOR later_stage IN
            SELECT id FROM mission_stage
            WHERE mission_id = route.mission_id AND sequence_no >= route.sequence_no
            ORDER BY sequence_no DESC
        LOOP
            UPDATE mission_stage SET sequence_no = sequence_no + 1
            WHERE id = later_stage.id;
        END LOOP;
        INSERT INTO mission_stage (mission_id, stage_code, sequence_no, status)
        VALUES (route.mission_id, 'AMR_ARRIVAL', route.sequence_no,
                CASE WHEN route.started_at IS NOT NULL THEN 'COMPLETED' ELSE 'WAITING' END);
    END LOOP;
END $$;
