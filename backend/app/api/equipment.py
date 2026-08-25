from fastapi import APIRouter, HTTPException
from psycopg.rows import dict_row

from app.database import get_db_connection


router = APIRouter(
    prefix="/api/equipment",
    tags=["equipment"],
)


@router.get("")
def get_equipment():
    """
    전체 장비와 각 장비의 현재 상태를 조회한다.
    """

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


def change_equipment_state(equipment_code: str, command_type: str):
    """
    START / STOP 공통 처리 함수.

    현재 단계에서는 DB 상태만 변경한다.
    실제 ROS2 명령은 이후 ROS2 Adapter 단계에서 연결한다.
    """

    if command_type == "START":
        target_status = "RUNNING"

    elif command_type == "STOP":
        target_status = "STOPPED"

    else:
        raise HTTPException(
            status_code=400,
            detail="Unsupported command",
        )

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:

                # ------------------------------------------------
                # 1. Equipment 조회
                # ------------------------------------------------
                cursor.execute(
                    """
                    SELECT
                        id,
                        code,
                        name,
                        type,
                        enabled
                    FROM equipment
                    WHERE code = %s;
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

                # ------------------------------------------------
                # 2. Command 기록
                # ------------------------------------------------
                cursor.execute(
                    """
                    INSERT INTO equipment_command (
                        equipment_id,
                        command_type,
                        status,
                        completed_at
                    )
                    VALUES (
                        %s,
                        %s,
                        'SUCCESS',
                        NOW()
                    )
                    RETURNING
                        id,
                        command_type,
                        status,
                        requested_at,
                        completed_at;
                    """,
                    (
                        equipment["id"],
                        command_type,
                    ),
                )

                command = cursor.fetchone()

                # ------------------------------------------------
                # 3. Equipment State 변경
                # ------------------------------------------------
                cursor.execute(
                    """
                    UPDATE equipment_state
                    SET
                        status = %s,
                        updated_at = NOW()
                    WHERE equipment_id = %s
                    RETURNING
                        status,
                        mode,
                        position_x,
                        position_y,
                        yaw,
                        lift_state,
                        updated_at;
                    """,
                    (
                        target_status,
                        equipment["id"],
                    ),
                )

                state = cursor.fetchone()

                conn.commit()

        return {
            "equipment": {
                "code": equipment["code"],
                "name": equipment["name"],
                "type": equipment["type"],
            },
            "command": command,
            "state": state,
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
