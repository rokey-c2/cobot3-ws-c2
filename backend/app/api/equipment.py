from fastapi import APIRouter, HTTPException
from psycopg.rows import dict_row

from app.database import get_db_connection
from app.mqtt_client import publish_equipment_control_command


router = APIRouter(
    prefix="/api/equipment",
    tags=["equipment"],
)


# 현재 실제 MQTT/ROS2 제어 Adapter가 연결된 장비 종류다.
# P3020, Conveyor, Sorter Adapter가 추가되면 여기에 종류를 추가한다.
SUPPORTED_CONTROL_TYPES = {"AMR"}


@router.get("")
def get_equipment():
    """전체 장비와 각 장비의 현재 상태를 조회한다."""

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:
                cursor.execute(
                    """
                    SELECT
                        e.id,
                        e.code,
                        e.name,
                        e.type,
                        e.enabled,
                        es.status,
                        es.mode,
                        es.position_x,
                        es.position_y,
                        es.yaw,
                        es.lift_state,
                        es.last_seen_at,
                        es.updated_at
                    FROM equipment e
                    LEFT JOIN equipment_state es
                        ON es.equipment_id = e.id
                    ORDER BY e.id;
                    """
                )
                equipment = cursor.fetchall()

        return {
            "count": len(equipment),
            "equipment": equipment,
        }

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to load equipment: {error}",
        )


def _mark_publish_failed(command_id: int, error: Exception):
    """DB에 저장한 명령이 MQTT 발행에 실패했음을 기록한다."""

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cursor:
                cursor.execute(
                    """
                    UPDATE equipment_command
                    SET
                        status = 'FAILED',
                        completed_at = NOW(),
                        error_message = %s
                    WHERE id = %s;
                    """,
                    (str(error), command_id),
                )
                conn.commit()

    except Exception as db_error:
        print(
            f"[CONTROL] Failed to mark command as FAILED: {db_error}",
            flush=True,
        )


def change_equipment_state(equipment_code: str, command_type: str):
    """START/STOP 명령을 저장하고 실제 MQTT Adapter에 전달한다.

    장비 상태는 명령 생성 시 미리 바꾸지 않는다. Adapter가 실제 제어를
    수행하고 SUCCESS 결과를 보낸 뒤 mqtt_client.update_command_result()가
    equipment_state를 RUNNING 또는 STOPPED로 변경한다.
    """

    action = str(command_type).strip().upper()

    if action not in {"START", "STOP"}:
        raise HTTPException(
            status_code=400,
            detail="Unsupported command",
        )

    target_status = "RUNNING" if action == "START" else "STOPPED"
    command = None
    equipment = None

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:
                cursor.execute(
                    """
                    SELECT
                        e.id,
                        e.code,
                        e.name,
                        e.type,
                        e.enabled,
                        es.status,
                        es.mode,
                        es.position_x,
                        es.position_y,
                        es.yaw,
                        es.lift_state,
                        es.updated_at
                    FROM equipment e
                    LEFT JOIN equipment_state es
                        ON es.equipment_id = e.id
                    WHERE e.code = %s;
                    """,
                    (equipment_code,),
                )
                equipment = cursor.fetchone()

                if equipment is None:
                    raise HTTPException(
                        status_code=404,
                        detail="Equipment not found",
                    )

                if not equipment["enabled"]:
                    raise HTTPException(
                        status_code=409,
                        detail="Equipment is disabled",
                    )

                if equipment["type"] not in SUPPORTED_CONTROL_TYPES:
                    raise HTTPException(
                        status_code=409,
                        detail=(
                            f"Actual START/STOP adapter is not connected "
                            f"for equipment type {equipment['type']}"
                        ),
                    )

                if equipment["status"] == target_status:
                    raise HTTPException(
                        status_code=409,
                        detail=f"Equipment is already {target_status}",
                    )

                cursor.execute(
                    """
                    SELECT id, command_type, status
                    FROM equipment_command
                    WHERE
                        equipment_id = %s
                        AND command_type IN ('START', 'STOP')
                        AND status IN ('PENDING', 'RUNNING')
                    ORDER BY id DESC
                    LIMIT 1;
                    """,
                    (equipment["id"],),
                )
                active_command = cursor.fetchone()

                if active_command is not None:
                    raise HTTPException(
                        status_code=409,
                        detail=(
                            "Another START/STOP command is already active: "
                            f"id={active_command['id']} "
                            f"status={active_command['status']}"
                        ),
                    )

                cursor.execute(
                    """
                    INSERT INTO equipment_command (
                        equipment_id,
                        command_type,
                        status
                    )
                    VALUES (%s, %s, 'PENDING')
                    RETURNING
                        id,
                        command_type,
                        status,
                        requested_at,
                        completed_at;
                    """,
                    (equipment["id"], action),
                )
                command = cursor.fetchone()
                conn.commit()

        try:
            publish_equipment_control_command(
                equipment_code=equipment["code"],
                command_id=command["id"],
                action=action,
            )

        except Exception as mqtt_error:
            _mark_publish_failed(command["id"], mqtt_error)
            raise HTTPException(
                status_code=503,
                detail=(
                    "Equipment command was saved, "
                    "but MQTT publish failed: "
                    f"{mqtt_error}"
                ),
            )

        return {
            "equipment": {
                "code": equipment["code"],
                "name": equipment["name"],
                "type": equipment["type"],
            },
            "command": command,
            "state": {
                "current_status": equipment["status"],
                "target_status": target_status,
            },
        }

    except HTTPException:
        raise

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"Equipment command failed: {error}",
        )


@router.post("/{equipment_code}/start")
def start_equipment(equipment_code: str):
    return change_equipment_state(
        equipment_code=equipment_code,
        command_type="START",
    )


@router.post("/{equipment_code}/stop")
def stop_equipment(equipment_code: str):
    return change_equipment_state(
        equipment_code=equipment_code,
        command_type="STOP",
    )
