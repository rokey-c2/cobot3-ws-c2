from json import dumps

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from psycopg.rows import dict_row

from app.database import get_db_connection
from app.process_events import create_tracking_mission
from app.mqtt_client import publish_p3020_arrival_command


router = APIRouter(
    prefix="/api/missions",
    tags=["missions"],
)


class MissionCreate(BaseModel):
    package_code: str = Field(min_length=1, max_length=100)
    region: str = Field(default="A", min_length=1, max_length=30)
    mission_code: str | None = Field(default=None, max_length=100)


@router.post("")
def create_mission(request: MissionCreate):
    """Create the active tracking mission used by live ROS2 events."""

    try:
        return create_tracking_mission(
            package_code=request.package_code,
            region=request.region,
            mission_code=request.mission_code,
        )
    except ValueError as error:
        raise HTTPException(status_code=409, detail=str(error))
    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to create mission: {error}",
        )


@router.get("/current")
def get_current_mission():
    """
    진행 중인 Mission을 우선 조회하고, 없으면 가장 최근 완료/실패 Mission과
    해당 Stage 목록을 반환한다.
    """

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:

                # Active Mission 우선, 없으면 가장 최근 종료 Mission 조회
                cursor.execute(
                    """
                    SELECT
                        id,
                        mission_code,
                        status,
                        current_stage,
                        started_at,
                        completed_at,
                        created_at
                    FROM mission
                    ORDER BY
                        CASE WHEN status IN ('READY', 'RUNNING', 'PAUSED')
                             THEN 0 ELSE 1 END,
                        created_at DESC
                    LIMIT 1;
                    """
                )

                mission = cursor.fetchone()

                if mission is None:
                    return {
                        "mission": None,
                        "stages": [],
                    }

                # 해당 Mission의 진행 단계 조회
                cursor.execute(
                    """
                    SELECT
                        id,
                        stage_code,
                        sequence_no,
                        status,
                        started_at,
                        completed_at
                    FROM mission_stage
                    WHERE mission_id = %s
                    ORDER BY sequence_no;
                    """,
                    (mission["id"],),
                )

                stages = cursor.fetchall()

        return {
            "mission": mission,
            "stages": stages,
        }

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to load current mission: {error}",
        )


@router.post("/current/confirm-p3020-arrival")
def confirm_p3020_arrival():
    """Continue a manual-delivery mission through the normal P3020 path."""
    command_id = None
    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:
                cursor.execute("""
                    SELECT id, mission_code, status, current_stage
                    FROM mission
                    WHERE status IN ('READY', 'RUNNING', 'PAUSED')
                    ORDER BY created_at DESC LIMIT 1 FOR UPDATE;
                """)
                mission = cursor.fetchone()
                if mission is None:
                    raise HTTPException(status_code=409, detail="No active mission")
                if mission["status"] != "RUNNING":
                    raise HTTPException(status_code=409, detail="Mission must be RUNNING")
                if mission["current_stage"] != "AMR_NAVIGATION":
                    raise HTTPException(
                        status_code=409,
                        detail=("P3020 arrival requires AMR_NAVIGATION "
                                f"(current: {mission['current_stage']})"),
                    )

                cursor.execute("""
                    SELECT e.id, e.code, e.enabled, es.status
                    FROM equipment e LEFT JOIN equipment_state es ON es.equipment_id = e.id
                    WHERE e.code = 'AMR_IN';
                """)
                amr = cursor.fetchone()
                if amr is None or not amr["enabled"]:
                    raise HTTPException(status_code=409, detail="AMR_IN is unavailable")
                if amr["status"] != "RUNNING":
                    raise HTTPException(status_code=409, detail="AMR_IN must be RUNNING")

                cursor.execute("""
                    SELECT e.id, e.enabled, es.status
                    FROM equipment e LEFT JOIN equipment_state es ON es.equipment_id = e.id
                    WHERE e.code = 'P3020_IN';
                """)
                p3020 = cursor.fetchone()
                if p3020 is None or not p3020["enabled"]:
                    raise HTTPException(status_code=409, detail="P3020_IN is unavailable")
                if p3020["status"] not in {"IDLE", "RUNNING"}:
                    raise HTTPException(status_code=409, detail="P3020_IN is not ready")

                cursor.execute("""
                    SELECT id, status FROM equipment_command
                    WHERE equipment_id = %s
                      AND command_type = 'CONFIRM_P3020_ARRIVAL'
                      AND command_payload->>'mission_id' = %s
                    ORDER BY id DESC LIMIT 1;
                """, (amr["id"], str(mission["id"])))
                previous = cursor.fetchone()
                if previous is not None and previous["status"] in {"PENDING", "RUNNING", "SUCCESS"}:
                    raise HTTPException(
                        status_code=409,
                        detail="P3020 arrival was already confirmed for this mission",
                    )

                payload = {
                    "mission_id": mission["id"],
                    "mission_code": mission["mission_code"],
                    "equipment_code": amr["code"],
                }
                cursor.execute("""
                    INSERT INTO equipment_command (
                        equipment_id, command_type, command_payload, status
                    ) VALUES (%s, 'CONFIRM_P3020_ARRIVAL', %s::jsonb, 'PENDING')
                    RETURNING id, command_type, status, requested_at;
                """, (amr["id"], dumps(payload)))
                command = cursor.fetchone()
                command_id = command["id"]
                conn.commit()

        publish_p3020_arrival_command(
            command_id=command_id, mission_id=mission["id"],
            mission_code=mission["mission_code"], equipment_code=amr["code"],
        )
        return {"mission": mission, "command": command}
    except HTTPException:
        raise
    except Exception as error:
        if command_id is not None:
            try:
                with get_db_connection() as conn:
                    with conn.cursor() as cursor:
                        cursor.execute("""
                            UPDATE equipment_command SET status = 'FAILED',
                                completed_at = NOW(), error_message = %s WHERE id = %s;
                        """, (str(error), command_id))
                        conn.commit()
            except Exception:
                pass
        raise HTTPException(
            status_code=500, detail=f"Failed to confirm P3020 arrival: {error}"
        )


@router.get("/commands/{command_id}")
def get_mission_command(command_id: int):
    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:
                cursor.execute("""
                    SELECT id, command_type, status, requested_at, completed_at, error_message
                    FROM equipment_command WHERE id = %s;
                """, (command_id,))
                command = cursor.fetchone()
        if command is None:
            raise HTTPException(status_code=404, detail="Command not found")
        return {"command": command}
    except HTTPException:
        raise
    except Exception as error:
        raise HTTPException(status_code=500, detail=f"Failed to load command: {error}")
