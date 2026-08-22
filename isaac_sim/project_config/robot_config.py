"""Runtime configuration for the single IW Hub navigation test."""

from pathlib import Path


ISAAC_SIM_ROOT = Path(__file__).resolve().parents[1]
IW_HUB_USD = (
    ISAAC_SIM_ROOT
    / "robots"
    / "iw_hub"
    / "iw_hub_v2.usda"
)


START_XY = (
    8.155903816223145,
    -5.628969192504883,
)

GOAL_XY = (
    -13.813100814819336,
    0.7454315423965454,
)


ROBOT_REGISTRY = [
    {
        "type": "iw_hub",
        "name": "amr_a",
        "namespace": "/amr_a",
        "role": "inbound_amr",
        "spawn_xyz": (START_XY[0], START_XY[1], 0.0),
        "spawn_yaw": 0.0,
    }
]


# This test uses the saved warehouse only. Do not create extra test obstacles.
TEST_OBSTACLES = []


# This test verifies navigation only. Do not attach a cargo payload.
CARGO_REGISTRY = []


SORTER_CONFIG = {
    "name": "sorter_01",
    "namespace": "/sorter_01",
    "routes": ["A", "B"],
}
