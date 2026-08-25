"""Runtime configuration for the single IW Hub navigation test."""


IW_HUB_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Samples/ROS2/Scenario/"
    "iw_hub_warehouse_navigation.usd"
)

CARD_BOX_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props/"
    "SM_CardBoxB_01_359.usd"
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
        # Keep NVIDIA's original dual-LiDAR placement, orientation,
        # scan rate, and horizontal resolution. Only max range is reduced.
        "lidar_max_range_m": 5.0,
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


# NVIDIA Simple Warehouse cardboard-box prop placed inside the cargo pod.
# The source asset is uniformly scaled at runtime to fit inside this envelope,
# preserving its original proportions/materials. Physics is enforced locally:
# dynamic rigid body + gravity + collision + 15 kg mass.
PARCEL_REGISTRY = [
    {
        "name": "parcel_box_01",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (10.5, -1.5, 0.455),
        "max_size_xyz": (0.50, 0.40, 0.30),
        "mass_kg": 15.0,
    }
]


SORTER_CONFIG = {
    "name": "sorter_01",
    "namespace": "/sorter_01",
    "routes": ["A", "B"],
}
