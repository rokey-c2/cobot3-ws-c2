-- ============================================================
-- Control Tower DB Integration Test
-- Scenario: Region B Package
-- ============================================================

-- 1. Mission 생성
INSERT INTO mission (
    mission_code,
    status,
    current_stage,
    started_at
)
VALUES (
    'MISSION-TEST-B001',
    'RUNNING',
    'AMR_PICKUP',
    NOW()
)
ON CONFLICT (mission_code) DO NOTHING;


-- 2. Mission Stage 생성
INSERT INTO mission_stage (
    mission_id,
    stage_code,
    sequence_no,
    status
)
SELECT
    m.id,
    stage.stage_code,
    stage.sequence_no,
    CASE
        WHEN stage.sequence_no = 1 THEN 'RUNNING'
        ELSE 'WAITING'
    END
FROM mission m
CROSS JOIN (
    VALUES
        (1, 'AMR_PICKUP'),
        (2, 'AMR_NAVIGATION'),
        (3, 'MANIPULATOR_PICK'),
        (4, 'MANIPULATOR_PLACE'),
        (5, 'MAIN_CONVEYOR'),
        (6, 'SORTER_A'),
        (7, 'SORTER_B'),
        (8, 'REGION_B'),
        (9, 'COMPLETE')
) AS stage(sequence_no, stage_code)
WHERE m.mission_code = 'MISSION-TEST-B001'
ON CONFLICT (mission_id, sequence_no) DO NOTHING;


-- 3. Package 생성
INSERT INTO package (
    package_code,
    mission_id,
    current_zone_id,
    region,
    status
)
SELECT
    'PKG-TEST-B001',
    m.id,
    z.id,
    'B',
    'WAITING'
FROM mission m
JOIN zone z
    ON z.zone_code = 'INPUT_ZONE'
WHERE m.mission_code = 'MISSION-TEST-B001'
ON CONFLICT (package_code) DO NOTHING;


-- 4. 최초 Package Event
INSERT INTO package_event (
    package_id,
    zone_id,
    event_type,
    result
)
SELECT
    p.id,
    z.id,
    'ENTER',
    'SUCCESS'
FROM package p
JOIN zone z
    ON z.zone_code = 'INPUT_ZONE'
WHERE p.package_code = 'PKG-TEST-B001';
