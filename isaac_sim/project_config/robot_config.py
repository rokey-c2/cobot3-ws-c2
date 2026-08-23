"""Runtime configuration for the single IW Hub navigation test."""


IW_HUB_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Samples/ROS2/Scenario/"
    "iw_hub_warehouse_navigation.usd"
)


# User-verified initial IW Hub transform.
# Spawn: x=10.5, y=1.80122, yaw=0 deg.
START_XY = (
    10.5,
    1.80122,
)

GOAL_XY = (
    1.0015223026275635,
    0.018860459327697754,
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


TEST_OBSTACLES = []


CARGO_REGISTRY = [
    {
        "name": "cargo_pod",
        "usd": "usd/cargo/cargo_box.usd",
        "spawn_xyz": (10.5, -1.5, 0.5),
        "spawn_yaw": 0.0,
    }
]


# One 15 kg parcel centered inside the 1 m cargo pod.
# Cargo floor top is approximately world Z=0.30 m, so the parcel starts
# 5 mm above it and settles naturally under gravity.
PARCEL_REGISTRY = [
    {
        "name": "parcel_box_01",
        "spawn_xyz": (10.5, -1.5, 0.455),
        "size_xyz": (0.50, 0.40, 0.30),
        "mass_kg": 15.0,
    }
]


SORTER_CONFIG = {
    "name": "sorter_01",
    "namespace": "/sorter_01",
    "routes": ["A", "B"],
}
