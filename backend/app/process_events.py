"""Apply live ROS2 process events to the Control Tower database.

The ROS2 bridge emits one small, normalised event whenever an AMR, P3020,
conveyor, or sorter state changes.  This module turns that event into the
three snapshots used by the web UI: equipment state, mission progress, and
package position/history.
"""

from datetime import datetime, timezone
import json
import re

from psycopg.rows import dict_row

from app.database import get_db_connection


VALID_REGIONS = {"A", "B", "C"}
CODE_PATTERN = re.compile(r"^[A-Za-z0-9_-]{1,100}$")


def normalize_region(region):
    value = str(region or "A").strip().upper()
    return value if value in VALID_REGIONS | {"D"} else "UNKNOWN"


def build_route(region):
    """Return the physical route for one package.

    A Region-B package passes Sorter A before it is diverted by Sorter B;
    Region C similarly passes A and B first.  Unknown destinations travel
    through every sorter to the exception lane.
    """

    region = normalize_region(region)
    route = [
        "INPUT_ZONE",
        "AMR_PICKUP",
        "AMR_NAVIGATION",
        "AMR_ARRIVAL",
        "MANIPULATOR_PICK",
        "MANIPULATOR_PLACE",
        "MAIN_CONVEYOR",
    ]

    if region in VALID_REGIONS:
        for sorter_region in ("A", "B", "C"):
            route.append(f"SORTER_{sorter_region}")
            if sorter_region == region:
                break
        route.append(f"REGION_{region}")
    else:
        route.extend(["SORTER_A", "SORTER_B", "SORTER_C", "EXCEPTION"])

    route.append("COMPLETE")
    return route


def _stage(code, zone=None, *, done=False, complete=False, failed=False, completed=()):
    return {"stage_code": code, "zone_code": zone, "stage_completed": done,
            "failed": failed, "complete": complete, "completed_stages": list(completed)}


def interpret_process_event(payload):
    """Only verified physical milestones complete stages 5 through 10."""
    event_type = str(payload.get("event_type", "")).strip().upper()
    state = str(payload.get("state", "")).strip().upper()
    region = normalize_region(payload.get("region"))
    if event_type == "PACKAGE_REGISTERED":
        return _stage("INPUT_ZONE", "INPUT_ZONE")
    if event_type == "AMR_STATE":
        if state == "ERROR":
            return _stage(None, "AMR_IN", failed=True)
        if state in {"ROTATE_TO_DOCK", "ENTER_CARGO", "LIFTING"}:
            return _stage("AMR_PICKUP", "AMR_IN")
        if state == "PICKUP_DONE":
            return _stage("AMR_NAVIGATION", "AMR_IN")
        if state == "CONVEYOR_DOCK_DONE":
            return _stage("AMR_ARRIVAL", "P3020_IN")
    if event_type == "P3020_STATE":
        if state.startswith("DONE_FAIL"):
            return _stage(None, "P3020_IN", failed=True)
        if state in {"SCANNING", "APPROACHING", "GRASPING"}:
            return _stage("MANIPULATOR_PICK", "P3020_IN")
        if state == "PICK_CONFIRMED":
            return _stage("MANIPULATOR_PICK", "P3020_IN", done=True)
        if state in {"MOVING", "PLACING"}:
            return _stage("MANIPULATOR_PLACE", "P3020_IN")
        if state in {"PLACE_CONFIRMED", "DONE_SUCCESS"}:
            return _stage("MAIN_CONVEYOR", "MAIN_CONVEYOR", completed=("MANIPULATOR_PLACE",))
    if event_type == "CONVEYOR_STATE" and state in {"PACKAGE_ENTERED", "TRANSPORTING"}:
        return _stage("MAIN_CONVEYOR", "MAIN_CONVEYOR")
    if event_type == "SORTER_STATE":
        name, _, sorter = state.partition(":")
        if name in {"ENTERED", "ROUTING", "PASSED"} and sorter in VALID_REGIONS:
            return _stage(f"SORTER_{sorter}", f"SORTER_{sorter}",
                          done=name == "PASSED",
                          completed=("MAIN_CONVEYOR",) if name == "ENTERED" and sorter == "A" else ())
        if name == "ARRIVED" and sorter in VALID_REGIONS:
            return _stage(f"REGION_{sorter}", f"REGION_{sorter}", done=True)
        if state == "SORTING_COMPLETE" and region in VALID_REGIONS:
            return _stage("COMPLETE", f"REGION_{region}", done=True, complete=True)
    if event_type == "P3020_OUT_STATE":
        if state == "BIN_PLACED":
            return _stage("EXCEPTION", "EXCEPTION", done=True)
        if state == "DONE_SUCCESS":
            return _stage("COMPLETE", "EXCEPTION", done=True, complete=True)
        if state in {"DONE_FAIL:RETREAT_FAILED", "DONE_FAIL:HOME_RETURN_FAILED"}:
            return _stage("COMPLETE", "EXCEPTION", failed=True)
    return _stage(None)


def progress_ignore_reason(payload, interpretation, stages):
    """Prevent rescans/return failures from rewriting a delivered package."""
    by_code = {row["stage_code"]: row for row in stages}
    conveyor = by_code.get("MAIN_CONVEYOR", {})
    handed_off = conveyor.get("status") != "WAITING" and bool(conveyor)
    if handed_off and payload["event_type"] in {"AMR_STATE", "P3020_STATE"}:
        return "package already handed to conveyor"
    target = by_code.get(interpretation["stage_code"])
    if interpretation["stage_code"] and target is None:
        return "event stage is outside package route"
    if target and target["status"] == "COMPLETED" and not interpretation["failed"]:
        return "stage already completed"
    latest = max((s["sequence_no"] for s in stages if s["status"] != "WAITING"), default=0)
    if target and target["sequence_no"] < latest:
        return "stale stage event"
    if interpretation["complete"] and any(
        s["stage_code"] != "COMPLETE" and s["status"] != "COMPLETED" for s in stages
    ):
        return "physical completion milestones missing"
    return None


def _generated_code(prefix):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S-%f")
    return f"{prefix}-{stamp}"


def _validate_code(value, label):
    if not CODE_PATTERN.fullmatch(value):
        raise ValueError(f"{label} must contain only letters, numbers, '-' or '_'")


def _create_tracking_records(cursor, package_code, region, mission_code=None):
    package_code = str(package_code or _generated_code("PKG-LIVE")).strip()
    mission_code = str(mission_code or _generated_code("MISSION-LIVE")).strip()
    region = normalize_region(region)
    _validate_code(package_code, "package_code")
    _validate_code(mission_code, "mission_code")

    cursor.execute(
        """
        INSERT INTO mission (mission_code, status, current_stage, started_at)
        VALUES (%s, 'RUNNING', 'INPUT_ZONE', NOW())
        RETURNING id, mission_code, status, current_stage, started_at;
        """,
        (mission_code,),
    )
    mission = cursor.fetchone()

    for sequence_no, stage_code in enumerate(build_route(region), start=1):
        cursor.execute(
            """
            INSERT INTO mission_stage (
                mission_id, stage_code, sequence_no, status,
                started_at, completed_at
            )
            VALUES (
                %s, %s, %s,
                CASE WHEN %s = 1 THEN 'RUNNING' ELSE 'WAITING' END,
                CASE WHEN %s = 1 THEN NOW() ELSE NULL END,
                NULL
            );
            """,
            (mission["id"], stage_code, sequence_no, sequence_no, sequence_no),
        )

    cursor.execute("SELECT id FROM zone WHERE zone_code = 'INPUT_ZONE';")
    input_zone = cursor.fetchone()
    if input_zone is None:
        raise RuntimeError("INPUT_ZONE is missing; apply backend/db/seed.sql")

    cursor.execute(
        """
        INSERT INTO package (
            package_code, mission_id, current_zone_id, region, status
        )
        VALUES (%s, %s, %s, %s, 'IN_PROGRESS')
        RETURNING id, package_code, region, status;
        """,
        (package_code, mission["id"], input_zone["id"], region),
    )
    package = cursor.fetchone()

    cursor.execute(
        """
        INSERT INTO package_event (
            package_id, zone_id, event_type, result, event_key, detail
        )
        VALUES (%s, %s, 'MISSION_STARTED', 'SUCCESS', %s, %s::jsonb)
        ON CONFLICT (event_key) WHERE event_key IS NOT NULL DO NOTHING;
        """,
        (package["id"], input_zone["id"], f"mission-start:{mission_code}", "{}"),
    )
    return mission, package


def create_tracking_mission(package_code, region, mission_code=None):
    """Create one active mission and its package route."""

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:
                cursor.execute(
                    """
                    SELECT mission_code
                    FROM mission
                    WHERE status IN ('READY', 'RUNNING', 'PAUSED')
                    ORDER BY created_at DESC
                    LIMIT 1;
                    """
                )
                active = cursor.fetchone()
                if active is not None:
                    raise ValueError(f"Active mission already exists: {active['mission_code']}")

                mission, package = _create_tracking_records(
                    cursor, package_code, region, mission_code
                )
                conn.commit()
                return {"mission": mission, "package": package, "route": build_route(region)}
    except ValueError:
        raise


def _find_or_create_active_tracking(cursor, payload):
    cursor.execute(
        """
        SELECT m.id, m.mission_code, m.status, p.id AS package_id,
               p.package_code, p.region
        FROM mission m
        LEFT JOIN package p ON p.mission_id = m.id
        WHERE m.status IN ('READY', 'RUNNING', 'PAUSED')
          AND (%s::text IS NULL OR p.package_code = %s)
        ORDER BY m.created_at DESC, p.created_at ASC
        LIMIT 1;
        """,
        (payload.get("package_code"), payload.get("package_code")),
    )
    active = cursor.fetchone()
    if active is not None and active["package_id"] is not None:
        return active

    # Only registration or initial AMR pickup may create a package. Late
    # rescan, OUT completion and AMR return events must not invent missions.
    if payload.get("event_type") != "PACKAGE_REGISTERED" and not (
        payload.get("event_type") == "AMR_STATE"
        and payload.get("state") in {"ROTATE_TO_DOCK", "ENTER_CARGO", "LIFTING"}
    ):
        return None

    mission, package = _create_tracking_records(
        cursor,
        payload.get("package_code"),
        payload.get("region"),
        payload.get("mission_code"),
    )
    return {
        "id": mission["id"],
        "mission_code": mission["mission_code"],
        "status": mission["status"],
        "package_id": package["id"],
        "package_code": package["package_code"],
        "region": package["region"],
    }


def _equipment_status_from_event(payload, interpretation):
    explicit = str(payload.get("equipment_status", "")).strip().upper()
    if explicit:
        return explicit
    if interpretation["failed"]:
        return "ERROR"
    state = str(payload.get("state", "")).strip().upper()
    if state in {"STOPPED", "IDLE", "RUNNING", "ERROR"}:
        return state
    if state == "DONE_SUCCESS":
        return "IDLE"
    return "RUNNING"


def handle_process_event(payload):
    """Atomically apply one normalised process event."""

    if not isinstance(payload, dict):
        raise ValueError("process event payload must be an object")
    event_type = str(payload.get("event_type", "")).strip().upper()
    if not event_type:
        raise ValueError("event_type is required")

    interpretation = interpret_process_event(payload)
    if interpretation["stage_code"] is None and not interpretation["failed"]:
        return {"ignored": True, "reason": "no package milestone"}
    event_key = str(payload.get("event_key", "")).strip() or None
    equipment_code = str(payload.get("equipment_code", "")).strip().upper() or None

    with get_db_connection() as conn:
        with conn.cursor(row_factory=dict_row) as cursor:
            if event_key:
                cursor.execute("SELECT id FROM package_event WHERE event_key = %s;", (event_key,))
                if cursor.fetchone() is not None:
                    return {"duplicate": True, "event_key": event_key}
            active = _find_or_create_active_tracking(cursor, payload)
            if active is None:
                return {"ignored": True, "reason": "batch status without active package"}
            region = normalize_region(active["region"])
            if payload.get("region") is None:
                payload = dict(payload, region=region)
                interpretation = interpret_process_event(payload)

            zone_id = None
            zone_code = interpretation["zone_code"]
            if zone_code:
                cursor.execute("SELECT id FROM zone WHERE zone_code = %s;", (zone_code,))
                zone = cursor.fetchone()
                if zone is not None:
                    zone_id = zone["id"]

            if equipment_code:
                cursor.execute(
                    """
                    UPDATE equipment_state es
                    SET status = %s, last_seen_at = NOW(), updated_at = NOW()
                    FROM equipment e
                    WHERE es.equipment_id = e.id AND e.code = %s;
                    """,
                    (_equipment_status_from_event(payload, interpretation), equipment_code),
                )

            cursor.execute(
                "SELECT stage_code, sequence_no, status FROM mission_stage WHERE mission_id = %s ORDER BY sequence_no;",
                (active["id"],),
            )
            stages = cursor.fetchall()
            ignored = progress_ignore_reason(payload, interpretation, stages)
            if ignored:
                conn.commit()  # Keep equipment errors even when package progress is unaffected.
                return {"ignored": True, "reason": ignored}

            mission_status = "RUNNING"
            package_status = "IN_PROGRESS"
            if interpretation["failed"]:
                mission_status = "FAILED"
                package_status = "FAILED"
            elif interpretation["complete"]:
                mission_status = "COMPLETED"
                package_status = "COMPLETED"

            stage_code = interpretation["stage_code"]
            if interpretation["failed"] and not stage_code:
                cursor.execute(
                    """
                    UPDATE mission_stage
                    SET status = 'FAILED',
                        started_at = COALESCE(started_at, NOW()),
                        completed_at = COALESCE(completed_at, NOW())
                    WHERE mission_id = %s AND status = 'RUNNING'
                    RETURNING stage_code;
                    """,
                    (active["id"],),
                )
                failed_stage = cursor.fetchone()
                if failed_stage is not None:
                    stage_code = failed_stage["stage_code"]

            if stage_code:
                cursor.execute(
                    """
                    SELECT sequence_no FROM mission_stage
                    WHERE mission_id = %s AND stage_code = %s;
                    """,
                    (active["id"], stage_code),
                )
                target = cursor.fetchone()
                if target is not None:
                    sequence_no = target["sequence_no"]
                    cursor.execute(
                        """
                        UPDATE mission_stage
                        SET status = 'COMPLETED',
                            started_at = COALESCE(started_at, NOW()),
                            completed_at = COALESCE(completed_at, NOW())
                        WHERE mission_id = %s AND sequence_no < %s
                          AND (sequence_no <= 4 OR stage_code = ANY(%s))
                          AND status <> 'FAILED';
                        """,
                        (active["id"], sequence_no, interpretation["completed_stages"]),
                    )
                    cursor.execute(
                        """
                        UPDATE mission_stage
                        SET status = %s,
                            started_at = COALESCE(started_at, NOW()),
                            completed_at = CASE WHEN %s IN ('COMPLETED', 'FAILED')
                                                THEN COALESCE(completed_at, NOW())
                                                ELSE completed_at END
                        WHERE mission_id = %s AND sequence_no = %s;
                        """,
                        (
                            "FAILED" if interpretation["failed"] else (
                                "COMPLETED" if interpretation["stage_completed"] else "RUNNING"
                            ),
                            "FAILED" if interpretation["failed"] else (
                                "COMPLETED" if interpretation["stage_completed"] else "RUNNING"
                            ),
                            active["id"],
                            sequence_no,
                        ),
                    )

            cursor.execute(
                """
                UPDATE mission
                SET status = %s,
                    current_stage = COALESCE(%s, current_stage),
                    started_at = COALESCE(started_at, NOW()),
                    completed_at = CASE WHEN %s IN ('COMPLETED', 'FAILED') THEN NOW()
                                        ELSE completed_at END
                WHERE id = %s;
                """,
                (mission_status, stage_code, mission_status, active["id"]),
            )
            cursor.execute(
                """
                UPDATE package
                SET current_zone_id = COALESCE(%s, current_zone_id),
                    status = %s, updated_at = NOW()
                WHERE id = %s;
                """,
                (zone_id, package_status, active["package_id"]),
            )
            cursor.execute(
                """
                INSERT INTO package_event (
                    package_id, zone_id, event_type, result, event_key, detail
                )
                VALUES (%s, %s, %s, %s, %s, %s::jsonb)
                ON CONFLICT (event_key) WHERE event_key IS NOT NULL DO NOTHING
                RETURNING id;
                """,
                (
                    active["package_id"], zone_id, event_type,
                    "FAILED" if interpretation["failed"] else "SUCCESS",
                    event_key, json.dumps(payload, ensure_ascii=False),
                ),
            )
            event = cursor.fetchone()
            conn.commit()

    return {
        "duplicate": event is None,
        "mission_code": active["mission_code"],
        "package_code": active["package_code"],
        "stage_code": stage_code,
        "zone_code": zone_code,
        "mission_status": mission_status,
        "package_status": package_status,
    }
