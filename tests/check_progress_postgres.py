"""Run in the backend container; use a temporary schema, never live missions.

docker compose exec -T backend python < tests/check_progress_postgres.py
"""
from pathlib import Path
import uuid

from psycopg import sql
from psycopg.rows import dict_row
from app import process_events as events
from app.database import get_db_connection


schema = "test_progress_" + uuid.uuid4().hex


def isolated_connection():
    conn = get_db_connection()
    conn.execute(sql.SQL("SET search_path TO {}").format(sql.Identifier(schema)))
    return conn


def stages(package_code):
    with isolated_connection() as conn:
        rows = conn.execute(
            "SELECT s.stage_code, s.status FROM mission_stage s JOIN package p ON p.mission_id=s.mission_id "
            "WHERE p.package_code=%s ORDER BY s.sequence_no", (package_code,),
        ).fetchall()
    return dict(rows)


def emit(package_code, region, kind, state):
    return events.handle_process_event({
        "package_code": package_code, "region": region, "event_type": kind,
        "state": state, "event_key": uuid.uuid4().hex,
    })


try:
    with get_db_connection() as conn:
        conn.execute(sql.SQL("CREATE SCHEMA {}").format(sql.Identifier(schema)))
    with isolated_connection() as conn:
        for name in ("init.sql", "seed.sql"):
            conn.execute((Path("/app/db") / name).read_text())
    events.get_db_connection = isolated_connection
    for region, retreat_failed, omit_pick in [("A", False, False), ("B", False, False),
                                               ("C", False, False), ("D", False, False),
                                               ("D", True, False), ("D", False, True)]:
        package = "test-" + uuid.uuid4().hex
        send = lambda kind, state: emit(package, region, kind, state)
        registration = {"package_code": package, "region": region,
                        "event_type": "PACKAGE_REGISTERED", "state": "REGISTERED",
                        "event_key": "register:" + package}
        events.handle_process_event(registration)
        for state in ("ENTER_CARGO", "PICKUP_DONE", "CONVEYOR_DOCK_DONE"):
            send("AMR_STATE", state)
        # IN/AMR topics have no package identifier in the live adapter.
        events.handle_process_event({"event_type": "P3020_STATE", "state": "SCANNING"})
        if not omit_pick:
            send("P3020_STATE", "PICK_CONFIRMED")
        send("P3020_STATE", "MOVING")
        assert stages(package)["MANIPULATOR_PICK"] == ("RUNNING" if omit_pick else "COMPLETED")
        send("P3020_STATE", "PLACE_CONFIRMED")
        before = stages(package)
        assert before["MAIN_CONVEYOR"] == "RUNNING"
        assert send("P3020_STATE", "SCANNING")["ignored"]
        assert send("AMR_STATE", "ERROR")["ignored"]
        assert stages(package) == before
        for sorter in "ABC":
            send("SORTER_STATE", "ENTERED:" + sorter)
            assert stages(package)["MAIN_CONVEYOR"] == "COMPLETED"
            assert stages(package)["SORTER_" + sorter] == "RUNNING"
            send("SORTER_STATE", "PASSED:" + sorter)
            assert stages(package)["SORTER_" + sorter] == "COMPLETED"
            if sorter == region:
                break
        if region == "D":
            assert stages(package)["EXCEPTION"] == "WAITING"
            send("P3020_OUT_STATE", "BIN_PLACED")
            assert stages(package)["EXCEPTION"] == "COMPLETED"
            assert stages(package)["COMPLETE"] == "WAITING"
            result = send("P3020_OUT_STATE", "DONE_FAIL:RETREAT_FAILED" if retreat_failed else "DONE_SUCCESS")
        else:
            send("SORTER_STATE", "ARRIVED:" + region)
            assert stages(package)["REGION_" + region] == "COMPLETED"
            result = send("SORTER_STATE", "SORTING_COMPLETE")
        if omit_pick:
            assert result["reason"] == "physical completion milestones missing"
            assert stages(package)["COMPLETE"] == "WAITING"
        elif retreat_failed:
            assert result["mission_status"] == "FAILED"
            assert stages(package)["COMPLETE"] == "FAILED"
            assert all(v == "COMPLETED" for k, v in stages(package).items() if k != "COMPLETE")
        else:
            assert result["mission_status"] == "COMPLETED"
            assert all(v == "COMPLETED" for v in stages(package).values())
            assert send("P3020_OUT_STATE", "DONE_SUCCESS")["ignored"]
        assert events.handle_process_event(registration)["duplicate"]
        print(f"PASS region={region} retreat_failed={retreat_failed} missing_pick={omit_pick}")
    with isolated_connection() as conn:
        assert conn.execute("SELECT count(*) FROM mission").fetchone()[0] == 6
    print("PASS six PostgreSQL scenarios; no phantom missions")
finally:
    with get_db_connection() as conn:
        conn.execute(sql.SQL("DROP SCHEMA {} CASCADE").format(sql.Identifier(schema)))
