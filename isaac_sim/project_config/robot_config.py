"""Robot registry.

spawn_xyz / spawn_yaw 값은 실제 Scene 배치 후 프로젝트 좌표에 맞게 수정하세요.
"""

ROBOT_REGISTRY = [
    # Inbound AMR
    {
        "type": "forklift_b",
        "name": "amr_in_01",
        "namespace": "/amr_in_01",
        "role": "inbound",
        "spawn_xyz": (0.0, 0.0, 0.0),
        "spawn_yaw": 0.0,
    },
    {
        "type": "forklift_b",
        "name": "amr_in_02",
        "namespace": "/amr_in_02",
        "role": "inbound",
        "spawn_xyz": (0.0, 2.0, 0.0),
        "spawn_yaw": 0.0,
    },

    # Outbound A
    {
        "type": "forklift_b",
        "name": "amr_out_a_01",
        "namespace": "/amr_out_a_01",
        "role": "outbound_a",
        "spawn_xyz": (10.0, 5.0, 0.0),
        "spawn_yaw": 0.0,
    },
    {
        "type": "forklift_b",
        "name": "amr_out_a_02",
        "namespace": "/amr_out_a_02",
        "role": "outbound_a",
        "spawn_xyz": (10.0, 7.0, 0.0),
        "spawn_yaw": 0.0,
    },

    # Outbound B
    {
        "type": "forklift_b",
        "name": "amr_out_b_01",
        "namespace": "/amr_out_b_01",
        "role": "outbound_b",
        "spawn_xyz": (10.0, -5.0, 0.0),
        "spawn_yaw": 0.0,
    },
    {
        "type": "forklift_b",
        "name": "amr_out_b_02",
        "namespace": "/amr_out_b_02",
        "role": "outbound_b",
        "spawn_xyz": (10.0, -7.0, 0.0),
        "spawn_yaw": 0.0,
    },

    # P3020
    {
        "type": "p3020",
        "name": "arm_in_01",
        "namespace": "/arm_in_01",
        "role": "inbound_arm",
        "spawn_xyz": (3.0, 0.0, 0.0),
        "spawn_yaw": 0.0,
    },
    {
        "type": "p3020",
        "name": "arm_a_01",
        "namespace": "/arm_a_01",
        "role": "loading_a",
        "spawn_xyz": (8.0, 4.0, 0.0),
        "spawn_yaw": 0.0,
    },
    {
        "type": "p3020",
        "name": "arm_b_01",
        "namespace": "/arm_b_01",
        "role": "loading_b",
        "spawn_xyz": (8.0, -4.0, 0.0),
        "spawn_yaw": 0.0,
    },
]

SORTER_CONFIG = {
    "name": "sorter_01",
    "namespace": "/sorter_01",
    "routes": ["A", "B"],
}
