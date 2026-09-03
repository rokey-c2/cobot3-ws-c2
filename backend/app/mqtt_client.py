import json
import os

import paho.mqtt.client as mqtt
from psycopg.rows import dict_row

from app.database import get_db_connection


MQTT_HOST = os.getenv("MQTT_HOST", "mosquitto")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

SUBSCRIBE_TOPIC = "controltower/#"


def update_equipment_status(equipment_code: str, payload: dict):
    """Update a non-AMR equipment snapshot from its ROS2 bridge."""

    status = str(payload.get("status", "")).strip().upper()
    mode = payload.get("mode")
    if not status:
        print(f"[MQTT][DB] Invalid equipment status: {payload}", flush=True)
        return

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cursor:
                cursor.execute(
                    """
                    UPDATE equipment_state es
                    SET status = %s,
                        mode = COALESCE(%s, es.mode),
                        last_seen_at = NOW(),
                        updated_at = NOW()
                    FROM equipment e
                    WHERE es.equipment_id = e.id AND e.code = %s;
                    """,
                    (status, mode, equipment_code),
                )
                if cursor.rowcount == 0:
                    print(f"[MQTT][DB] Equipment not found: {equipment_code}", flush=True)
                    return
                conn.commit()
        print(f"[MQTT][DB] Equipment status: {equipment_code} -> {status}", flush=True)
    except Exception as error:
        print(f"[MQTT][DB] Failed to update equipment status: {error}", flush=True)


def update_amr_status(equipment_code: str, payload: dict):
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
                    (status, mode, lift_state, equipment_code),
                )

                if cursor.rowcount == 0:
                    print(
                        f"[MQTT][DB] AMR not found: {equipment_code}",
                        flush=True,
                    )
                    return

                conn.commit()

        print(
            f"[MQTT][DB] Updated AMR status: "
            f"{equipment_code} "
            f"status={status} mode={mode} lift_state={lift_state}",
            flush=True,
        )

    except Exception as error:
        print(
            f"[MQTT][DB] Failed to update AMR status: {error}",
            flush=True,
        )


def update_amr_odom(equipment_code: str, payload: dict):
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
                    (x, y, yaw, equipment_code),
                )

                if cursor.rowcount == 0:
                    print(
                        f"[MQTT][DB] AMR not found: {equipment_code}",
                        flush=True,
                    )
                    return

                conn.commit()

        print(
            f"[MQTT][DB] Updated AMR odom: "
            f"{equipment_code} x={x:.3f} y={y:.3f} yaw={yaw:.3f}",
            flush=True,
        )

    except Exception as error:
        print(
            f"[MQTT][DB] Failed to update AMR odom: {error}",
            flush=True,
        )


def update_command_result(payload: dict):
    """Adapter 결과를 명령 기록과 실제 장비 상태에 함께 반영한다."""

    command_id = payload.get("command_id")
    equipment_code = payload.get("equipment_code")
    status = str(payload.get("status", "")).strip().upper()
    error_message = payload.get("error_message")

    if command_id is None or not equipment_code or not status:
        print(
            f"[MQTT][DB] Invalid command result: {payload}",
            flush=True,
        )
        return

    db_status = "FAILED" if status == "BUSY" else status

    try:
        with get_db_connection() as conn:
            with conn.cursor(row_factory=dict_row) as cursor:
                cursor.execute(
                    """
                    UPDATE equipment_command AS ec
                    SET
                        status = %s,
                        completed_at = CASE
                            WHEN %s IN ('SUCCESS', 'FAILED', 'CANCELED')
                                THEN NOW()
                            ELSE ec.completed_at
                        END,
                        error_message = %s
                    FROM equipment AS e
                    WHERE
                        ec.id = %s
                        AND ec.equipment_id = e.id
                        AND e.code = %s
                    RETURNING
                        ec.equipment_id,
                        ec.command_type;
                    """,
                    (
                        db_status,
                        db_status,
                        error_message,
                        command_id,
                        equipment_code,
                    ),
                )
                command = cursor.fetchone()

                if command is None:
                    print(
                        f"[MQTT][DB] Command not found or equipment mismatch: "
                        f"id={command_id} equipment={equipment_code}",
                        flush=True,
                    )
                    return

                if (
                    db_status == "SUCCESS"
                    and command["command_type"] in {"START", "STOP"}
                ):
                    target_status = (
                        "RUNNING"
                        if command["command_type"] == "START"
                        else "STOPPED"
                    )

                    cursor.execute(
                        """
                        UPDATE equipment_state
                        SET
                            status = %s,
                            last_seen_at = NOW(),
                            updated_at = NOW()
                        WHERE equipment_id = %s;
                        """,
                        (target_status, command["equipment_id"]),
                    )

                conn.commit()

        print(
            f"[MQTT][DB] Command updated: "
            f"id={command_id} status={db_status}",
            flush=True,
        )

    except Exception as error:
        print(
            f"[MQTT][DB] Failed to update command: {error}",
            flush=True,
        )


def _publish_command(topic: str, payload: dict, command_name: str, retain=False):
    if not mqtt_client.is_connected():
        raise RuntimeError("MQTT broker is not connected")

    result = mqtt_client.publish(
        topic,
        json.dumps(payload),
        qos=1,
        retain=retain,
    )

    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        raise RuntimeError(
            f"MQTT {command_name} publish failed rc={result.rc}"
        )

    print(
        f"[MQTT] Published {command_name} "
        f"topic={topic} payload={payload}",
        flush=True,
    )


def publish_navigation_command(
    equipment_code: str,
    command_id: int,
    x: float,
    y: float,
    yaw: float,
):
    topic = f"controltower/command/amr/{equipment_code}/navigate"
    payload = {
        "command_id": int(command_id),
        "x": float(x),
        "y": float(y),
        "yaw": float(yaw),
    }
    _publish_command(topic, payload, "NAVIGATE")


def publish_lift_command(
    equipment_code: str,
    command_id: int,
    action: str,
):
    normalized_action = str(action).strip().upper()

    if normalized_action not in {"UP", "DOWN"}:
        raise ValueError(f"Invalid lift action: {action}")

    topic = f"controltower/command/amr/{equipment_code}/lift"
    payload = {
        "command_id": int(command_id),
        "action": normalized_action,
    }
    _publish_command(topic, payload, "LIFT")


def publish_p3020_arrival_command(
    command_id: int,
    mission_id: int,
    mission_code: str,
    equipment_code: str = "AMR_IN",
):
    """Ask the ROS2 mission to continue from a manually confirmed arrival."""
    topic = "controltower/command/mission/p3020-arrival"
    payload = {
        "command_id": int(command_id),
        "mission_id": int(mission_id),
        "mission_code": str(mission_code),
        "equipment_code": str(equipment_code),
    }
    _publish_command(topic, payload, "CONFIRM_P3020_ARRIVAL")


def publish_equipment_control_command(
    equipment_code: str,
    command_id: int,
    action: str,
):
    """실제 장비 Adapter에 START 또는 STOP 제어 명령을 보낸다.

    retain=True로 마지막 제어 상태를 보존한다. Adapter가 재시작되면 마지막
    STOP 명령을 다시 받아 정지 상태를 임의로 풀지 않는다.
    """

    normalized_action = str(action).strip().upper()

    if normalized_action not in {"START", "STOP"}:
        raise ValueError(f"Invalid equipment control action: {action}")

    topic = (
        f"controltower/command/equipment/"
        f"{equipment_code}/control"
    )
    payload = {
        "command_id": int(command_id),
        "action": normalized_action,
    }
    _publish_command(
        topic,
        payload,
        f"EQUIPMENT_{normalized_action}",
        retain=True,
    )


def handle_message(topic: str, payload: dict):
    if topic == "controltower/result/command":
        update_command_result(payload)
        return

    if topic == "controltower/process/event":
        try:
            from app.process_events import handle_process_event

            result = handle_process_event(payload)
            print(f"[MQTT][PROCESS] applied: {result}", flush=True)
        except Exception as error:
            print(f"[MQTT][PROCESS] failed: {error}", flush=True)
        return

    parts = topic.split("/")

    if len(parts) != 4 or parts[0] != "controltower":
        return

    resource_type = parts[1]
    equipment_code = parts[2]
    message_type = parts[3]

    if resource_type == "equipment" and message_type == "status":
        update_equipment_status(equipment_code, payload)
        return

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

    elif message_type == "lift":
        update_amr_status(
            equipment_code=equipment_code,
            payload=payload,
        )


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

    client.subscribe(SUBSCRIBE_TOPIC)

    print(
        f"[MQTT] Subscribed: {SUBSCRIBE_TOPIC}",
        flush=True,
    )


def on_message(
    client,
    userdata,
    message,
):
    raw_payload = message.payload.decode("utf-8")

    print(
        f"[MQTT] topic={message.topic} "
        f"payload={raw_payload}",
        flush=True,
    )

    try:
        payload = json.loads(raw_payload)

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


mqtt_client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2,
    client_id="control-tower-backend",
)

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message


def start_mqtt():
    print(
        f"[MQTT] Connecting to {MQTT_HOST}:{MQTT_PORT}",
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
