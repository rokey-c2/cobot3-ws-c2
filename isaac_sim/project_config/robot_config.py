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
        "lidar_max_range_m": 30.0,
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


# Cargo pod is baked into the map (not code-spawned). In
# Parcel_Sorting_Map_real_real_final_final it's at
# /World/cargo_box_gaurd_size_200_fix_02, translate (9, -3, 0.5) -- both the
# path and Z changed from the previous map (was /World/Cargo/... at z=0.38),
# and the pod was rebuilt with a different internal structure (no more
# separate Visual/PhysicsColliders split). PARCEL_Z below is carried over
# from the OLD pod's empirical drop-test value (rest z=0.4) and has NOT been
# re-verified against this rebuilt pod -- box placement should be re-checked
# (e.g. another drop test) before trusting it here.
#
# CARGO_FLOOR_TOP_Z was previously "measured" as 0.055 via a static USD
# BBoxCache read of PhysicsColliders/floor -- that method turned out to be
# unreliable for this pod (the same method also mismeasured the wall
# height earlier). A real physics drop test (spawn a parcel well above the
# pod, let gravity settle it, read where it actually stops) showed the box
# comes to rest with its center at world Z 0.4, not ~0.185. Using the
# empirically-verified value directly instead of a derived floor-top +
# half-height formula, since the formula's own inputs were unreliable.
PARCEL_SCALE_XYZ = (0.75, 0.75, 0.5)
_PARCEL_SETTLE_CLEARANCE_Z = 0.005
PARCEL_Z = 0.4 + _PARCEL_SETTLE_CLEARANCE_Z

# Real measured box footprint is 0.375 x 0.375 m (half=0.1875 m). The old
# offsets (+-0.25 X, +-0.2 Y) left only a 2.5 cm gap on the Y axis, tight
# enough that the P3020 camera was seeing two adjacent boxes as one blob.
# +-0.28 on both axes gives a 0.56 m center-to-center spacing -> ~18.5 cm
# clear gap between box edges on both axes.
_PARCEL_GRID_OFFSET = 0.28

PARCEL_REGISTRY = [
    {
        "name": "parcel_box_01",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (9.0 - _PARCEL_GRID_OFFSET, -3.0 - _PARCEL_GRID_OFFSET, PARCEL_Z),
        "scale_xyz": PARCEL_SCALE_XYZ,
        "mass_kg": 15.0,
        "destination": "A",
    },
    {
        "name": "parcel_box_02",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (9.0 - _PARCEL_GRID_OFFSET, -3.0 + _PARCEL_GRID_OFFSET, PARCEL_Z),
        "scale_xyz": PARCEL_SCALE_XYZ,
        "mass_kg": 15.0,
        "destination": "B",
    },
    {
        "name": "parcel_box_03",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (9.0 + _PARCEL_GRID_OFFSET, -3.0 - _PARCEL_GRID_OFFSET, PARCEL_Z),
        "scale_xyz": PARCEL_SCALE_XYZ,
        "mass_kg": 15.0,
        "destination": "C",
    },
    {
        "name": "parcel_box_04",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (9.0 + _PARCEL_GRID_OFFSET, -3.0 + _PARCEL_GRID_OFFSET, PARCEL_Z),
        "scale_xyz": PARCEL_SCALE_XYZ,
        "mass_kg": 15.0,
        "destination": "UNKNOWN",
    },
]


SORTER_CONFIG = {
    "name": "sorter_01",
    "namespace": "/sorter_01",
    "routes": ["A", "B"],
}
