-- ============================================================
-- Control Tower v1 Initial Seed Data
-- ============================================================

-- ------------------------------------------------------------
-- 1. Equipment
-- 현재 Control Tower v1에서 관제할 실제 장비
-- ------------------------------------------------------------

INSERT INTO equipment (code, name, type, enabled)
VALUES
    ('AMR_IN',        'Inbound AMR',          'AMR',         TRUE),
    ('P3020_IN',      'Inbound Manipulator',  'MANIPULATOR', TRUE),
    ('MAIN_CONVEYOR', 'Main Conveyor',        'CONVEYOR',    TRUE),
    ('SORTER_A',      'Region A Sorter',      'SORTER',      TRUE),
    ('SORTER_B',      'Region B Sorter',      'SORTER',      TRUE),
    ('SORTER_C',      'Region C Sorter',      'SORTER',      TRUE)
ON CONFLICT (code) DO NOTHING;


-- ------------------------------------------------------------
-- 2. Equipment Initial State
-- 현재 상태 Snapshot
-- ------------------------------------------------------------

INSERT INTO equipment_state (
    equipment_id,
    status,
    mode,
    lift_state
)
SELECT
    id,
    'IDLE',
    CASE
        WHEN type = 'AMR' THEN 'AUTO'
        ELSE NULL
    END,
    CASE
        WHEN type = 'AMR' THEN 'DOWN'
        ELSE NULL
    END
FROM equipment
WHERE code IN (
    'AMR_IN',
    'P3020_IN',
    'MAIN_CONVEYOR',
    'SORTER_A',
    'SORTER_B',
    'SORTER_C'
)
ON CONFLICT (equipment_id) DO NOTHING;


-- ------------------------------------------------------------
-- 3. Zones
-- Package Zone Tracking 기준
-- ------------------------------------------------------------

INSERT INTO zone (
    zone_code,
    name,
    zone_type,
    map_x,
    map_y
)
VALUES
    ('INPUT_ZONE',     'Input Zone',             'INPUT',       NULL, NULL),
    ('AMR_IN',         'Inbound AMR Transport',  'AMR',         NULL, NULL),
    ('P3020_IN',       'Manipulator Station',    'MANIPULATOR', NULL, NULL),

    ('MAIN_CONVEYOR',  'Main Conveyor',          'CONVEYOR',    NULL, NULL),

    ('SORTER_A',       'Sorter A',               'SORTER',      NULL, NULL),
    ('SORTER_B',       'Sorter B',               'SORTER',      NULL, NULL),
    ('SORTER_C',       'Sorter C',               'SORTER',      NULL, NULL),

    ('REGION_A',       'Region A Conveyor',      'DESTINATION', NULL, NULL),
    ('REGION_B',       'Region B Conveyor',      'DESTINATION', NULL, NULL),
    ('REGION_C',       'Region C Conveyor',      'DESTINATION', NULL, NULL),

    ('EXCEPTION',      'Exception Conveyor',     'EXCEPTION',   NULL, NULL)
ON CONFLICT (zone_code) DO NOTHING;
