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


# TEMP TEST VALUE -- Parcel_Sorting_Map final AMR/cargo layout is not baked
# in yet (map still WIP). Placeholder spawn in open floor away from the
# conveyor (x=-6..0, y~0) and from p3020_in (0.2,-1.5)/p3020_out (-14.2,-2.6).
# Kept ~3 m from the cargo dock (TARGET_ROOT_Y in iw_hub_mission_agent.py)
# so the AMR's body (~1.5 m long) doesn't spawn overlapping the cargo pod --
# too close made ROTATE_TO_DOCK physically jam against the pod in testing.
# Replace once the user gives the real rviz2-measured spawn pose.
START_XY = (
    3.0,
    1.5,
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


# TEMP TEST VALUE -- source_prim_name updated to match the guard prim that
# actually exists in Parcel_Sorting_Map ("cargo_box_gaurd_size_200_fix"; the
# old "..._201" name from the previous map no longer exists). z=0.5 keeps
# cargo_guard_clone.py's fixed BASELINE_* collision geometry's leg bottom at
# world Z=0 (floor) -- this is independent of the cloned visual mesh's own
# size, so it stays 0.5 regardless of which guard asset is cloned. spawn_xyz
# x/y is a placeholder just past the AMR's test spawn (START_XY), not a real
# measured dock position.
CARGO_REGISTRY = [
    {
        "name": "cargo_box_gaurd_size_200_fix",
        "source_prim_name": "cargo_box_gaurd_size_200_fix",
        "spawn_xyz": (3.0, -1.8, 0.5),
        "spawn_yaw": 0.0,
        "mass_kg": 20.0,
    }
]


# Cargo pod is now baked into the map (not code-spawned) at
# /World/Cargo/cargo_box_gaurd_size_200_fix, translate (9, -3, 0.38).
# CARGO_FLOOR_TOP_Z is measured directly off that saved prim's
# PhysicsColliders/floor collider (world Z of its top face) via headless USD
# inspection -- NOT derived from any formula, since this instance's actual
# geometry does not match cargo_guard_clone.py's code-spawn baseline anymore.
CARGO_FLOOR_TOP_Z = 0.055

# User-specified: 2x2 grid at x=9 +-0.25, y=-3 +-0.2, non-uniform scale
# (0.75, 0.75, 0.5) applied directly to the CardBoxB_01 source (a 0.5 m
# cube), so half of the scaled height is 0.5 * 0.5 * 0.5 = 0.125 m.
# User asked for z=0.27, but that assumed a much higher floor than this
# pod's actual measured 0.055 m -- at z=0.27 the box would float ~0.09 m
# above the floor and drop when Play starts. Using the real floor height
# instead: box center z = floor_top + half_height + small clearance.
PARCEL_SCALE_XYZ = (0.75, 0.75, 0.5)
_PARCEL_HALF_HEIGHT_Z = 0.125
_PARCEL_FLOOR_CLEARANCE_Z = 0.005
PARCEL_Z = CARGO_FLOOR_TOP_Z + _PARCEL_HALF_HEIGHT_Z + _PARCEL_FLOOR_CLEARANCE_Z

PARCEL_REGISTRY = [
    {
        "name": "parcel_box_01",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (9.0 - 0.25, -3.0 - 0.2, PARCEL_Z),
        "scale_xyz": PARCEL_SCALE_XYZ,
        "mass_kg": 15.0,
    },
    {
        "name": "parcel_box_02",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (9.0 - 0.25, -3.0 + 0.2, PARCEL_Z),
        "scale_xyz": PARCEL_SCALE_XYZ,
        "mass_kg": 15.0,
    },
    {
        "name": "parcel_box_03",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (9.0 + 0.25, -3.0 - 0.2, PARCEL_Z),
        "scale_xyz": PARCEL_SCALE_XYZ,
        "mass_kg": 15.0,
    },
    {
        "name": "parcel_box_04",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (9.0 + 0.25, -3.0 + 0.2, PARCEL_Z),
        "scale_xyz": PARCEL_SCALE_XYZ,
        "mass_kg": 15.0,
    },
]


SORTER_CONFIG = {
    "name": "sorter_01",
    "namespace": "/sorter_01",
    "routes": ["A", "B"],
}
