"""Runtime configuration for the first IW Hub autonomous-navigation MVP."""

from pathlib import Path


ISAAC_SIM_ROOT = Path(__file__).resolve().parents[1]
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
        "spawn_xyz": (1.5, 0.0, 0.0),
        "spawn_yaw": 0.0,
    }
]


# The first obstacle blocks the straight route from (1.5, 0.0) to
# the experiment goal (6.0, 0.0), forcing Nav2 to plan around it.
TEST_OBSTACLES = [
    {
        "name": "route_obstacle_01",
        "position": (3.5, 0.0, 0.5),
        "scale": (0.8, 1.4, 1.0),
        "color": (0.85, 0.10, 0.10),
    }
]


# One loaded carrier starts on amr_a.  Its floor is above the 2D LiDAR scan
# plane so the robot does not classify its own payload as an obstacle.
CARGO_REGISTRY = [
    {
        "name": "container_01",
        "robot_name": "amr_a",
        "goal_xy": (6.0, 0.0),
        "lift_offset": (0.0, 0.0, 0.85),
    }
]


SORTER_CONFIG = {
    "name": "sorter_01",
    "namespace": "/sorter_01",
    "routes": ["A", "B"],
}
