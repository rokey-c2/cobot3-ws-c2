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
        # Preserve NVIDIA's IW Hub LiDAR mounting/orientation, scan rate,
        # firing rate, tick rate, and horizontal resolution.
        # rangeOffsetM starts rays outside the carried guard so its nearby
        # walls/legs do not occlude the real navigation scene.
        "lidar_range_offset_m": 0.75,
        "lidar_min_range_m": 0.80,
        "lidar_max_range_m": 5.0,
    }
]


TEST_OBSTACLES = []


# Replace the old generated cargo_pod with an exact runtime clone of the
# existing warehouse prim named cargo_box_gaurd_size_201.  The requested cargo
# home transform is kept EXACTLY at the previous cargo_pod transform.
CARGO_REGISTRY = [
    {
        "name": "cargo_box_gaurd_size_201_cargo",
        "source_prim_name": "cargo_box_gaurd_size_201",
        "spawn_xyz": (10.5, -1.5, 0.5),
        "spawn_yaw": 0.0,
        "mass_kg": 20.0,
    }
]


# Four identical NVIDIA parcels in one 2 x 2 layer.  The group is centered
# exactly on the cargo guard center (10.5, -1.5); every parcel remains 0.30 m
# high, matching the previously verified single-parcel height.
PARCEL_REGISTRY = [
    {
        "name": "parcel_box_01",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (10.32, -1.68, 0.455),
        "max_size_xyz": (0.30, 0.30, 0.30),
        "mass_kg": 15.0,
    },
    {
        "name": "parcel_box_02",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (10.32, -1.32, 0.455),
        "max_size_xyz": (0.30, 0.30, 0.30),
        "mass_kg": 15.0,
    },
    {
        "name": "parcel_box_03",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (10.68, -1.68, 0.455),
        "max_size_xyz": (0.30, 0.30, 0.30),
        "mass_kg": 15.0,
    },
    {
        "name": "parcel_box_04",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (10.68, -1.32, 0.455),
        "max_size_xyz": (0.30, 0.30, 0.30),
        "mass_kg": 15.0,
    },
]


SORTER_CONFIG = {
    "name": "sorter_01",
    "namespace": "/sorter_01",
    "routes": ["A", "B"],
}
