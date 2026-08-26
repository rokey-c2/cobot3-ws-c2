import json
from typing import Literal

import paho.mqtt.client as mqtt
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from psycopg.rows import dict_row

from app.database import get_db_connection
from app.mqtt_client import mqtt_client


router = APIRouter(
    prefix="/api/amr",
    tags=["amr-manual"],
)


class ManualDriveRequest(BaseModel):
    direction: Literal["FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"]


def publish_manual_command(equipment_code: str, direction: str):
    if not mqtt_client.is_connected():
        raise RuntimeError("MQTT broker is not connected")

    topic = f"controltower/command/amr/{equipment_code}/manual"
    payload = {
        "direction": direction,
    }

    result = mqtt_client.publish(
        topic,
        json.dumps(payload),
        qos=1,
        retain=False,
    )

    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        raise RuntimeError(
            f"MQTT MANUAL publish failed rc={result.rc}"
        )

    print(
        f"[MQTT] Published MANUAL topic={topic} payload={payload}",
        flush=True,
    )


@router.post("/{equipment_code}/manual")
def manual_drive(
    equipment_code: str,
    request: ManualDriveRequest,
):
    direction = request.direction

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
                        es.sync_status
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

                if equipment["type"] != "AMR":
                    raise HTTPException(
                        status_code=400,
                        detail="Equipment is not an AMR",
                    )

                if not equipment["enabled"] and direction != "STOP":
                    raise HTTPException(
                        status_code=409,
                        detail="AMR is disabled",
                    )

                # STOP은 안전 명령이므로 현재 상태와 관계없이 허용한다.
                if direction != "STOP":
                    if equipment["status"] != "RUNNING":
                        raise HTTPException(
                            status_code=409,
                            detail=(
                                "AMR must be RUNNING "
                                "before manual control"
                            ),
                        )

                    if equipment["sync_status"] != "SYNCED":
                        raise HTTPException(
                            status_code=409,
                            detail="AMR pose is not synchronized",
                        )

                    # Nav2가 주행 중일 때 수동 /cmd_vel과 경쟁하지 않도록 막는다.
                    cursor.execute(
                        """
                        SELECT id
                        FROM equipment_command
                        WHERE
                            equipment_id = %s
                            AND command_type = 'NAVIGATE'
                            AND status = 'RUNNING'
                        ORDER BY id DESC
                        LIMIT 1;
                        """,
                        (equipment["id"],),
                    )
                    running_navigation = cursor.fetchone()

                    if running_navigation is not None:
                        raise HTTPException(
                            status_code=409,
                            detail=(
                                "Navigation is in progress. "
                                "Wait for Nav2 to finish or stop it first."
                            ),
                        )

        publish_manual_command(
            equipment_code=equipment_code,
            direction=direction,
        )

        return {
            "equipment": {
                "code": equipment["code"],
                "name": equipment["name"],
            },
            "manual": {
                "direction": direction,
                "deadman_timeout_ms": 450,
            },
        }

    except HTTPException:
        raise

    except Exception as error:
        raise HTTPException(
            status_code=500,
            detail=f"Manual AMR command failed: {error}",
        )
