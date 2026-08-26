import json
import os

import paho.mqtt.client as mqtt

from app.database import get_db_connection


MQTT_HOST = os.getenv("MQTT_HOST", "mosquitto")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

POSE_TOPIC = "controltower/amr/+/pose"
POSE_SYNC_TOPIC = "controltower/amr/+/pose_sync"

VALID_SYNC_STATUS = {
    "OFFLINE",
    "SYNCING",
    "SYNCED",
    "DESYNC",
}


def _extract_equipment_code(topic: str, expected_message_type: str):
    parts = topic.split("/")

    if (
        len(parts) != 4
        or parts[0] != "controltower"
        or parts[1] != "amr"
        or parts[3] != expected_message_type
    ):
        return None

    return parts[2]


def update_canonical_pose(equipment_code: str, payload: dict):
    frame = str(payload.get("frame", "")).strip()
    source = str(payload.get("source", "")).strip()
    session_id = str(payload.get("session_id", "")).strip()

    try:
        x = float(payload["x"])
        y = float(payload["y"])
        yaw = float(payload["yaw"])
        seq = int(payload["seq"])
        session_epoch_ms = int(payload["session_epoch_ms"])
    except (KeyError, TypeError, ValueError) as error:
        print(
            f"[POSE][DB] Invalid canonical pose payload: {payload} ({error})",
            flush=True,
        )
        return

    if frame != "map":
        print(
            f"[POSE][DB] Rejected non-map pose: frame={frame!r}",
            flush=True,
        )
        return

    if not source or not session_id or seq < 1 or session_epoch_ms < 1:
        print(
            f"[POSE][DB] Invalid pose metadata: {payload}",
            flush=True,
        )
        return

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cursor:
                cursor.execute(
                    """
                    UPDATE equipment_state AS es
                    SET
                        position_x = %s,
                        position_y = %s,
                        yaw = %s,
                        pose_frame = 'map',
                        pose_source = %s,
                        pose_seq = %s,
                        pose_session_id = %s,
                        pose_session_epoch_ms = %s,
                        pose_updated_at = NOW(),
                        sync_status = CASE
                            WHEN es.sync_status = 'OFFLINE' THEN 'SYNCING'
                            ELSE es.sync_status
                        END,
                        last_seen_at = NOW(),
                        updated_at = NOW()
                    FROM equipment AS e
                    WHERE
                        es.equipment_id = e.id
                        AND e.code = %s
                        AND e.type = 'AMR'
                        AND (
                            %s > es.pose_session_epoch_ms
                            OR (
                                %s = es.pose_session_epoch_ms
                                AND %s > es.pose_seq
                            )
                        );
                    """,
                    (
                        x,
                        y,
                        yaw,
                        source,
                        seq,
                        session_id,
                        session_epoch_ms,
                        equipment_code,
                        session_epoch_ms,
                        session_epoch_ms,
                        seq,
                    ),
                )

                updated = cursor.rowcount
                conn.commit()

        if updated == 0:
            print(
                f"[POSE][DB] Ignored stale/missing pose "
                f"equipment={equipment_code} "
                f"session={session_id} seq={seq}",
                flush=True,
            )
            return

        print(
            f"[POSE][DB] Canonical map pose: {equipment_code} "
            f"x={x:.3f} y={y:.3f} yaw={yaw:.3f} "
            f"session={session_id} seq={seq}",
            flush=True,
        )

    except Exception as error:
        print(
            f"[POSE][DB] Failed to update canonical pose: {error}",
            flush=True,
        )


def update_pose_sync_status(equipment_code: str, payload: dict):
    status = str(payload.get("status", "")).strip().upper()

    try:
        session_epoch_ms = int(payload["session_epoch_ms"])
    except (KeyError, TypeError, ValueError):
        print(
            f"[POSE][DB] Invalid sync payload: {payload}",
            flush=True,
        )
        return

    if status not in VALID_SYNC_STATUS:
        print(
            f"[POSE][DB] Invalid sync status: {status!r}",
            flush=True,
        )
        return

    try:
        with get_db_connection() as conn:
            with conn.cursor() as cursor:
                cursor.execute(
                    """
                    UPDATE equipment_state AS es
                    SET
                        sync_status = %s,
                        updated_at = NOW()
                    FROM equipment AS e
                    WHERE
                        es.equipment_id = e.id
                        AND e.code = %s
                        AND e.type = 'AMR'
                        AND %s >= es.pose_session_epoch_ms;
                    """,
                    (
                        status,
                        equipment_code,
                        session_epoch_ms,
                    ),
                )
                conn.commit()

        print(
            f"[POSE][DB] Sync status: "
            f"{equipment_code} -> {status}",
            flush=True,
        )

    except Exception as error:
        print(
            f"[POSE][DB] Failed to update sync status: {error}",
            flush=True,
        )


def on_connect(
    client,
    userdata,
    flags,
    reason_code,
    properties,
):
    print(
        f"[POSE][MQTT] Connected to {MQTT_HOST}:{MQTT_PORT} "
        f"reason_code={reason_code}",
        flush=True,
    )

    client.subscribe(POSE_TOPIC, qos=1)
    client.subscribe(POSE_SYNC_TOPIC, qos=1)

    print(f"[POSE][MQTT] Subscribed: {POSE_TOPIC}", flush=True)
    print(f"[POSE][MQTT] Subscribed: {POSE_SYNC_TOPIC}", flush=True)


def on_message(
    client,
    userdata,
    message,
):
    raw_payload = message.payload.decode("utf-8")

    try:
        payload = json.loads(raw_payload)
    except json.JSONDecodeError:
        print(
            f"[POSE][MQTT] Invalid JSON topic={message.topic}",
            flush=True,
        )
        return

    equipment_code = _extract_equipment_code(
        message.topic,
        "pose",
    )
    if equipment_code is not None:
        update_canonical_pose(equipment_code, payload)
        return

    equipment_code = _extract_equipment_code(
        message.topic,
        "pose_sync",
    )
    if equipment_code is not None:
        update_pose_sync_status(equipment_code, payload)


pose_mqtt_client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2,
    client_id="control-tower-pose-backend",
)

pose_mqtt_client.on_connect = on_connect
pose_mqtt_client.on_message = on_message


def start_pose_mqtt():
    print(
        f"[POSE][MQTT] Connecting to {MQTT_HOST}:{MQTT_PORT}",
        flush=True,
    )

    pose_mqtt_client.connect_async(
        MQTT_HOST,
        MQTT_PORT,
        keepalive=60,
    )
    pose_mqtt_client.loop_start()


def stop_pose_mqtt():
    pose_mqtt_client.loop_stop()
    pose_mqtt_client.disconnect()

    print("[POSE][MQTT] Disconnected", flush=True)
