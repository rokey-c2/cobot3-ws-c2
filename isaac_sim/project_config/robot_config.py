"""Runtime configuration for the first IW Hub autonomous-navigation MVP."""

from pathlib import Path


ISAAC_SIM_ROOT = Path(__file__).resolve().parents[1]
WORLD_USD = (
    ISAAC_SIM_ROOT
    / "usd"
    / "env_temp_three_p3020"
    / "World0.usd"
)
IW_HUB_USD = (
    ISAAC_SIM_ROOT
    / "robots"
    / "iw_hub"
    / "iw_hub_v2.usda"
)


# Start with one AMR. Additional IW Hubs can be added after the single-robot
# Nav2 frame/topic chain has been verified in Isaac Sim.
ROBOT_REGISTRY = [
    {
        "type": "iw_hub",
        "name": "amr_a",
        "namespace": "/amr_a",
        "role": "inbound_amr",
        "spawn_xyz": (0.0, 0.0, 0.0),
        "spawn_yaw": 0.0,
    }
]


SORTER_CONFIG = {
    "name": "sorter_01",
    "namespace": "/sorter_01",
    "routes": ["A", "B"],
}

