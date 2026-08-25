from typing import Literal

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from psycopg.rows import dict_row
from psycopg.types.json import Jsonb

from app.database import get_db_connection
from app.mqtt_client import publish_navigation_command


router = APIRouter(
    prefix="/api/amr",
    tags=["amr"],
)


class NavigateRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class LiftRequest(BaseModel):
    action: Literal["UP", "DOWN"]


# =========================================================
# NAVIGATE
# =========================================================

@router.post("/{equipment_code}/navigate")
def navigate_amr(
    equipment_code: str,
    request: NavigateRequest,
):
    """
    AMR Navigation 명령을 DB에 등록하고
    MQTT를 통해 ROS2 MQTT Adapter에 전달한다.
    """

    command = None
    equipment = None

    try:
        # -------------------------------------------------
        # 1. DB에 NAVIGATE 명령 등록
        # -------------------------------------------------

        with get_db_connection() as conn:
            with conn.cursor(
                row_factory=dict_row
            ) as cursor:

                # -----------------------------------------
                # AMR 조회
                # -----------------------------------------

                cursor.execute(
                    """
                    SELECT
                        e.id,
                        e.code,
                        e.name,
                        e.type,
                        e.enabled,
                        es.status
                    FROM equipment e

                    LEFT JOIN equipment_state es
                        ON es.equipment_id = e.id

                    WHERE e.code = %s;
                    """,
                    (
                        equipment_code,
                    ),
                )

                equipment = cursor.fetchone()

                if equipment is None:
                    raise HTTPException(
                        status_code=404,
                        detail="Equipment not found",
                    )

                # -----------------------------------------
                # AMR 여부 확인
                # -----------------------------------------

                if equipment["type"] != "AMR":
                    raise HTTPException(
                        status_code=400,
                        detail="Equipment is not an AMR",
                    )

                # -----------------------------------------
                # Enabled 확인
                # -----------------------------------------

                if not equipment["enabled"]:
                    raise HTTPException(
                        status_code=409,
                        detail="AMR is disabled",
                    )

                # -----------------------------------------
                # RUNNING 상태 확인
                # -----------------------------------------

                if equipment["status"] != "RUNNING":
                    raise HTTPException(
                        status_code=409,
                        detail=(
                            "AMR must be RUNNING "
                            "before navigation"
                        ),
                    )

                # -----------------------------------------
                # Navigation Payload
                # -----------------------------------------

                payload = {
                    "x": request.x,
                    "y": request.y,
                    "yaw": request.yaw,
                }

                # -----------------------------------------
                # equipment_command 저장
                # -----------------------------------------

                cursor.execute(
                    """
                    INSERT INTO equipment_command (
                        equipment_id,
                        command_type,
                        command_payload,
                        status
                    )
                    VALUES (
                        %s,
                        'NAVIGATE',
                        %s,
                        'PENDING'
                    )
                    RETURNING
                        id,
                        command_type,
                        command_payload,
                        status,
                        requested_at;
                    """,
                    (
                        equipment["id"],
                        Jsonb(payload),
                    ),
                )

                command = cursor.fetchone()

                conn.commit()

        # -------------------------------------------------
        # 2. MQTT NAVIGATE 명령 Publish
        # -------------------------------------------------

        try:
            publish_navigation_command(
                equipment_code=equipment["code"],
                command_id=command["id"],
                x=request.x,
                y=request.y,
                yaw=request.yaw,
            )

        except Exception as mqtt_error:
            # MQTT 발행에 실패하면
            # 이미 생성된 DB command를 FAILED 처리한다.
            try:
                with get_db_connection() as conn:
                    with conn.cursor() as cursor:
                        cursor.execute(
                            """
                            UPDATE equipment_command
                            SET status = 'FAILED'
                            WHERE id = %s;
                            """,
                            (
                                command["id"],
                            ),
                        )

                        conn.commit()

            except Exception as db_error:
                print(
                    f"[NAVIGATE] Failed to mark command "
                    f"as FAILED: {db_error}",
                    flush=True,
                )

            raise HTTPException(
                status_code=503,
                detail=(
                    "Navigation command was saved, "
                    "but MQTT publish failed: "
                    f"{mqtt_error}"
                ),
            )

        # -------------------------------------------------
        # 3. API Response
        # -------------------------------------------------

        return {
            "equipment": {
                "code": equipment["code"],
                "name": equipment["name"],
            },
            "command": command,
        }

    except HTTPException:
        raise

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=(
                f"Navigation command failed: "
                f"{error}"
            ),
        )


# =========================================================
# LIFT
# =========================================================

@router.post("/{equipment_code}/lift")
def control_lift(
    equipment_code: str,
    request: LiftRequest,
):
    """
    AMR Lift Up / Down 명령을 등록한다.

    현재 단계에서는 ROS2에 직접 명령하지 않고
    equipment_command에 PENDING 상태로 저장한다.
    """

    try:
        with get_db_connection() as conn:
            with conn.cursor(
                row_factory=dict_row
            ) as cursor:

                # -----------------------------------------
                # 1. AMR 조회
                # -----------------------------------------

                cursor.execute(
                    """
                    SELECT
                        e.id,
                        e.code,
                        e.name,
                        e.type,
                        e.enabled,
                        es.status,
                        es.lift_state

                    FROM equipment e

                    LEFT JOIN equipment_state es
                        ON es.equipment_id = e.id

                    WHERE e.code = %s;
                    """,
                    (
                        equipment_code,
                    ),
                )

                equipment = cursor.fetchone()

                if equipment is None:
                    raise HTTPException(
                        status_code=404,
                        detail="Equipment not found",
                    )

                if equipment["type"] != "AMR":
                    raise HTTPException(
                        status_code=400,
                        detail="Equipment is not an AMR",
                    )

                if not equipment["enabled"]:
                    raise HTTPException(
                        status_code=409,
                        detail="AMR is disabled",
                    )

                if equipment["status"] != "RUNNING":
                    raise HTTPException(
                        status_code=409,
                        detail=(
                            "AMR must be RUNNING "
                            "before lift control"
                        ),
                    )

                # -----------------------------------------
                # 이미 같은 상태라면 명령 방지
                # -----------------------------------------

                if request.action == equipment["lift_state"]:
                    raise HTTPException(
                        status_code=409,
                        detail=(
                            f"Lift is already "
                            f"{request.action}"
                        ),
                    )

                # -----------------------------------------
                # Command Type
                # -----------------------------------------

                if request.action == "UP":
                    command_type = "LIFT_UP"

                else:
                    command_type = "LIFT_DOWN"

                # -----------------------------------------
                # 2. Command 기록
                # -----------------------------------------

                cursor.execute(
                    """
                    INSERT INTO equipment_command (
                        equipment_id,
                        command_type,
                        command_payload,
                        status
                    )
                    VALUES (
                        %s,
                        %s,
                        %s,
                        'PENDING'
                    )
                    RETURNING
                        id,
                        command_type,
                        command_payload,
                        status,
                        requested_at;
                    """,
                    (
                        equipment["id"],
                        command_type,
                        Jsonb(
                            {
                                "action": request.action,
                            }
                        ),
                    ),
                )

                command = cursor.fetchone()

                conn.commit()

        return {
            "equipment": {
                "code": equipment["code"],
                "name": equipment["name"],
                "current_lift_state": (
                    equipment["lift_state"]
                ),
            },
            "command": command,
        }

    except HTTPException:
        raise

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=(
                f"Lift command failed: "
                f"{error}"
            ),
        )