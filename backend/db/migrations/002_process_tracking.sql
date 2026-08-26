-- Live process-event tracking and replay protection.
ALTER TABLE package_event
    ADD COLUMN IF NOT EXISTS event_key VARCHAR(150),
    ADD COLUMN IF NOT EXISTS detail JSONB;

CREATE UNIQUE INDEX IF NOT EXISTS uq_package_event_event_key
    ON package_event(event_key)
    WHERE event_key IS NOT NULL;
