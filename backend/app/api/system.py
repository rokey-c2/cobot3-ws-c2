from fastapi import APIRouter, HTTPException
from psycopg.rows import dict_row

from app.database import get_db_connection


router = APIRouter(
    prefix="/api/system",
    tags=["system"],
)


def change_system_state(command_type: str):

    if command_type == "START":
        target_status = "RUNNING"

    elif command_type == "STOP":
        target_status = "STOPPED"

    else:
        raise HTTPException(
            status_code=400,
            detail="Unsupported system command",
        )

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:

                # 활성화된 장비 조회
                cursor.execute(
                    """
                    SELECT id, code
                    FROM equipment
                    WHERE enabled = TRUE
                    ORDER BY id;
                    """
                )

                equipment_list = cursor.fetchall()

                # 각 장비 Command 기록
                for equipment in equipment_list:
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
                        );
                        """,
                        (
                            equipment["id"],
                            command_type,
                        ),
                    )

                # 모든 활성 장비 상태 변경
                cursor.execute(
                    """
                    UPDATE equipment_state es
                    SET
                        status = %s,
                        updated_at = NOW()
                    FROM equipment e
                    WHERE
                        es.equipment_id = e.id
                        AND e.enabled = TRUE;
                    """,
                    (target_status,),
                )

                conn.commit()

        return {
            "command": command_type,
            "status": "SUCCESS",
            "equipment_count": len(equipment_list),
            "equipment": [
                equipment["code"]
                for equipment in equipment_list
            ],
        }

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"System command failed: {error}",
        )


@router.post("/start")
def start_system():
    return change_system_state("START")


@router.post("/stop")
def stop_system():
    return change_system_state("STOP")
