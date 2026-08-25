import json
import os

import paho.mqtt.client as mqtt

from app.database import get_db_connection


MQTT_HOST = os.getenv("MQTT_HOST", "mosquitto")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

SUBSCRIBE_TOPIC = "controltower/#"


# =========================================================
# AMR STATUS
# =========================================================

def update_amr_status(
    equipment_code: str,
    payload: dict,
):
    """
    AMR 상태 MQTT 메시지를 equipment_state에 반영한다.
    """

    status = payload.get("status")
    mode = payload.get("mode")
    lift_state = payload.get("lift_state")

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cursor:
                cursor.execute(
                    """
                    UPDATE equipment_state es
                    SET
                        status = COALESCE(%s, es.status),
                        mode = COALESCE(%s, es.mode),
                        lift_state = COALESCE(%s, es.lift_state),
                        last_seen_at = NOW(),
                        updated_at = NOW()
                    FROM equipment e
                    WHERE
                        es.equipment_id = e.id
                        AND e.code = %s
                        AND e.type = 'AMR';
                    """,
                    (
                        status,
                        mode,
                        lift_state,
                        equipment_code,
                    ),
                )

                if cursor.rowcount == 0:
                    print(
                        f"[MQTT][DB] AMR not found: "
                        f"{equipment_code}",
                        flush=True,
                    )
                    return

                conn.commit()

        print(
            f"[MQTT][DB] Updated AMR status: "
            f"{equipment_code} "
            f"status={status} "
            f"mode={mode} "
            f"lift_state={lift_state}",
            flush=True,
        )

    except Exception as error:
        print(
            f"[MQTT][DB] Failed to update AMR status: "
            f"{error}",
            flush=True,
        )


# =========================================================
# AMR ODOM
# =========================================================

def update_amr_odom(
    equipment_code: str,
    payload: dict,
):
    """
    AMR odom MQTT 메시지를 equipment_state에 반영한다.
    """

    x = payload.get("x")
    y = payload.get("y")
    yaw = payload.get("yaw")

    if x is None or y is None or yaw is None:
        print(
            f"[MQTT][DB] Invalid odom payload: {payload}",
            flush=True,
        )
        return

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cursor:
                cursor.execute(
                    """
                    UPDATE equipment_state es
                    SET
                        position_x = %s,
                        position_y = %s,
                        yaw = %s,
                        last_seen_at = NOW(),
                        updated_at = NOW()
                    FROM equipment e
                    WHERE
                        es.equipment_id = e.id
                        AND e.code = %s
                        AND e.type = 'AMR';
                    """,
                    (
                        x,
                        y,
                        yaw,
                        equipment_code,
                    ),
                )

                if cursor.rowcount == 0:
                    print(
                        f"[MQTT][DB] AMR not found: "
                        f"{equipment_code}",
                        flush=True,
                    )
                    return

                conn.commit()

        print(
            f"[MQTT][DB] Updated AMR odom: "
            f"{equipment_code} "
            f"x={x:.3f} "
            f"y={y:.3f} "
            f"yaw={yaw:.3f}",
            flush=True,
        )

    except Exception as error:
        print(
            f"[MQTT][DB] Failed to update AMR odom: "
            f"{error}",
            flush=True,
        )


# =========================================================
# NAVIGATION COMMAND RESULT
# =========================================================

def update_command_result(payload: dict):
    """
    ROS2 MQTT Adapter가 반환한 명령 결과를
    equipment_command에 반영한다.

    예상 상태:
    RUNNING
    SUCCESS
    FAILED
    BUSY
    """

    command_id = payload.get("command_id")
    status = payload.get("status")

    if command_id is None or status is None:
        print(
            f"[MQTT][DB] Invalid command result: "
            f"{payload}",
            flush=True,
        )
        return

    # Adapter의 BUSY는 실제로 명령을 수행하지 못한 상태이므로
    # DB에서는 FAILED로 기록한다.
    if status == "BUSY":
        db_status = "FAILED"
    else:
        db_status = status

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cursor:
                cursor.execute(
                    """
                    UPDATE equipment_command
                    SET status = %s
                    WHERE id = %s;
                    """,
                    (
                        db_status,
                        command_id,
                    ),
                )

                if cursor.rowcount == 0:
                    print(
                        f"[MQTT][DB] Command not found: "
                        f"id={command_id}",
                        flush=True,
                    )
                    return

                conn.commit()

        print(
            f"[MQTT][DB] Command updated: "
            f"id={command_id} "
            f"status={db_status}",
            flush=True,
        )

    except Exception as error:
        print(
            f"[MQTT][DB] Failed to update command: "
            f"{error}",
            flush=True,
        )


# =========================================================
# NAVIGATION COMMAND PUBLISH
# =========================================================

def publish_navigation_command(
    equipment_code: str,
    command_id: int,
    x: float,
    y: float,
    yaw: float,
):
    """
    FastAPI에서 생성한 NAVIGATE 명령을
    MQTT를 통해 ROS2 MQTT Adapter에 전달한다.
    """

    topic = (
        f"controltower/command/amr/"
        f"{equipment_code}/navigate"
    )

    payload = {
        "command_id": int(command_id),
        "x": float(x),
        "y": float(y),
        "yaw": float(yaw),
    }

    if not mqtt_client.is_connected():
        raise RuntimeError(
            "MQTT broker is not connected"
        )

    result = mqtt_client.publish(
        topic,
        json.dumps(payload),
    )

    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        raise RuntimeError(
            f"MQTT NAVIGATE publish failed "
            f"rc={result.rc}"
        )

    print(
        f"[MQTT] Published NAVIGATE "
        f"topic={topic} "
        f"payload={payload}",
        flush=True,
    )


# =========================================================
# MQTT MESSAGE ROUTER
# =========================================================

def handle_message(
    topic: str,
    payload: dict,
):
    """
    MQTT Topic을 분석해 처리한다.

    지원 Topic:

    controltower/amr/{equipment_code}/status
    controltower/amr/{equipment_code}/odom

    controltower/result/command
    """

    # -----------------------------------------------------
    # Command Result
    # -----------------------------------------------------

    if topic == "controltower/result/command":
        update_command_result(payload)
        return

    # -----------------------------------------------------
    # AMR Status / Odom
    # -----------------------------------------------------

    parts = topic.split("/")

    if len(parts) != 4:
        return

    if parts[0] != "controltower":
        return

    resource_type = parts[1]
    equipment_code = parts[2]
    message_type = parts[3]

    if resource_type != "amr":
        return

    if message_type == "status":
        update_amr_status(
            equipment_code=equipment_code,
            payload=payload,
        )

    elif message_type == "odom":
        update_amr_odom(
            equipment_code=equipment_code,
            payload=payload,
        )


# =========================================================
# MQTT CALLBACK
# =========================================================

def on_connect(
    client,
    userdata,
    flags,
    reason_code,
    properties,
):
    print(
        f"[MQTT] Connected to broker "
        f"{MQTT_HOST}:{MQTT_PORT} "
        f"reason_code={reason_code}",
        flush=True,
    )

    client.subscribe(
        SUBSCRIBE_TOPIC
    )

    print(
        f"[MQTT] Subscribed: "
        f"{SUBSCRIBE_TOPIC}",
        flush=True,
    )


def on_message(
    client,
    userdata,
    message,
):
    raw_payload = message.payload.decode(
        "utf-8"
    )

    print(
        f"[MQTT] topic={message.topic} "
        f"payload={raw_payload}",
        flush=True,
    )

    try:
        payload = json.loads(
            raw_payload
        )

    except json.JSONDecodeError:
        print(
            "[MQTT] Invalid JSON payload",
            flush=True,
        )
        return

    handle_message(
        topic=message.topic,
        payload=payload,
    )


# =========================================================
# MQTT CLIENT
# =========================================================

mqtt_client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2,
    client_id="control-tower-backend",
)

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message


# =========================================================
# MQTT START / STOP
# =========================================================

def start_mqtt():
    print(
        f"[MQTT] Connecting to "
        f"{MQTT_HOST}:{MQTT_PORT}",
        flush=True,
    )

    mqtt_client.connect_async(
        MQTT_HOST,
        MQTT_PORT,
        keepalive=60,
    )

    mqtt_client.loop_start()


def stop_mqtt():
    mqtt_client.loop_stop()
    mqtt_client.disconnect()

    print(
        "[MQTT] Disconnected",
        flush=True,
    )