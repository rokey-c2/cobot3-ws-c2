from fastapi import APIRouter, HTTPException
from psycopg.rows import dict_row

from app.api.equipment import (
    SUPPORTED_CONTROL_TYPES,
    change_equipment_state,
)
from app.database import get_db_connection


router = APIRouter(
    prefix="/api/system",
    tags=["system"],
)


def change_system_state(command_type: str):
    """현재 실제 Adapter가 연결된 모든 장비에 START/STOP을 전달한다.

    연결되지 않은 장비를 DB에서 거짓 SUCCESS로 바꾸지 않고 응답의
    unsupported_equipment에 명시한다. 이후 각 장비 Adapter가 추가되면
    SUPPORTED_CONTROL_TYPES에 종류를 추가하는 것만으로 전체 제어 대상에
    포함된다.
    """

    action = str(command_type).strip().upper()

    if action not in {"START", "STOP"}:
        raise HTTPException(
            status_code=400,
            detail="Unsupported system command",
        )

    target_status = "RUNNING" if action == "START" else "STOPPED"

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:
                cursor.execute(
                    """
                    SELECT
                        e.code,
                        e.type,
                        es.status
                    FROM equipment e
                    LEFT JOIN equipment_state es
                        ON es.equipment_id = e.id
                    WHERE e.enabled = TRUE
                    ORDER BY e.id;
                    """
                )
                equipment_list = cursor.fetchall()

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"System command failed: {error}",
        )

    controllable = [
        equipment
        for equipment in equipment_list
        if equipment["type"] in SUPPORTED_CONTROL_TYPES
    ]
    unsupported = [
        equipment["code"]
        for equipment in equipment_list
        if equipment["type"] not in SUPPORTED_CONTROL_TYPES
    ]

    commands = []
    unchanged = []
    failures = []

    for equipment in controllable:
        if equipment["status"] == target_status:
            unchanged.append(equipment["code"])
            continue

        try:
            result = change_equipment_state(
                equipment_code=equipment["code"],
                command_type=action,
            )
            commands.append(result["command"])

        except HTTPException as error:
            failures.append(
                {
                    "equipment_code": equipment["code"],
                    "detail": error.detail,
                }
            )

    if failures and not commands:
        response_status = "FAILED"
    elif failures:
        response_status = "PARTIAL"
    elif commands:
        response_status = "PENDING"
    else:
        response_status = "NO_CHANGE"

    return {
        "command": action,
        "status": response_status,
        "target_status": target_status,
        "equipment_count": len(controllable),
        "equipment": [item["code"] for item in controllable],
        "commands": commands,
        "unchanged_equipment": unchanged,
        "unsupported_equipment": unsupported,
        "failures": failures,
    }


@router.post("/start")
def start_system():
    return change_system_state("START")


@router.post("/stop")
def stop_system():
    return change_system_state("STOP")
