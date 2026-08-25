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


# The old cargo_pod is gone. Create ONE additional instance of the existing
# warehouse object cargo_box_gaurd_size_201 at the EXACT old cargo_pod pose.
# Old cargo_pod pose from the verified baseline: (10.5, -1.5, 0.5), yaw=0 deg.
CARGO_REGISTRY = [
    {
        "name": "cargo_box_gaurd_size_201",
        "source_prim_name": "cargo_box_gaurd_size_201",
        "spawn_xyz": (10.5, -1.5, 0.5),
        "spawn_yaw": 0.0,
        "mass_kg": 20.0,
    }
]


# Four parcels are arranged as ONE 2 x 2 floor layer INSIDE the cloned guard.
# X/Y are offsets from the exact cargo center (10.5, -1.5). Z is resolved at
# runtime from the cloned guard's actual world bounds so the boxes are placed
# above the guard bottom instead of reusing the old cargo_pod's Z blindly.
PARCEL_REGISTRY = [
    {
        "name": "parcel_box_01",
        "usd": CARD_BOX_USD,
        "offset_xy": (-0.18, -0.18),
        "max_size_xyz": (0.30, 0.30, 0.30),
        "mass_kg": 15.0,
    },
    {
        "name": "parcel_box_02",
        "usd": CARD_BOX_USD,
        "offset_xy": (-0.18, 0.18),
        "max_size_xyz": (0.30, 0.30, 0.30),
        "mass_kg": 15.0,
    },
    {
        "name": "parcel_box_03",
        "usd": CARD_BOX_USD,
        "offset_xy": (0.18, -0.18),
        "max_size_xyz": (0.30, 0.30, 0.30),
        "mass_kg": 15.0,
    },
    {
        "name": "parcel_box_04",
        "usd": CARD_BOX_USD,
        "offset_xy": (0.18, 0.18),
        "max_size_xyz": (0.30, 0.30, 0.30),
        "mass_kg": 15.0,
    },
]


SORTER_CONFIG = {
    "name": "sorter_01",
    "namespace": "/sorter_01",
    "routes": ["A", "B"],
}
