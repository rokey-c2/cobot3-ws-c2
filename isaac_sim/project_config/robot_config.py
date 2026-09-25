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
# Parcel_Sorting_Map it's at
# /World/cargo_box_gaurd_size_200_fix_02, translate (9, -3, 0.5).
#
# User decision: 4 boxes made the pod rock (uneven weight distribution as
# each one got picked) -- spawn just 1 box instead, centered on the pod, at
# scale (0.7,0.7,0.7). PARCEL_Z is a fresh drop test with this exact scale
# on this exact pod (spawn well above, let gravity settle, read rest
# position): box center comes to rest at world Z 0.4518.
PARCEL_SCALE_XYZ = (0.7, 0.7, 0.7)
_PARCEL_SETTLE_CLEARANCE_Z = 0.005
PARCEL_Z = 0.4518 + _PARCEL_SETTLE_CLEARANCE_Z

# "destination"을 생략하면 main_mission.py._spawn_parcels()가 A/B/C/D 중
# 하나를 랜덤으로 배정한다 (D는 sorter 어느 구역과도 안 맞아 p3020_out
# 쪽 "배송지 오류" 구간으로 흘러간다). 특정 박스를 항상 같은 목적지로
# 고정하고 싶을 때만 여기에 "destination"을 명시하면 된다.
PARCEL_REGISTRY = [
    {
        "name": "parcel_box_01",
        "box_id": 4,
        "destination": "D",
        "usd": CARD_BOX_USD,
        "spawn_xyz": (9.0, -3.0, PARCEL_Z),
        "scale_xyz": PARCEL_SCALE_XYZ,
        "mass_kg": 15.0,
    },
]


SORTER_CONFIG = {
    "name": "sorter_01",
    "namespace": "/sorter_01",
    "routes": ["A", "B"],
}
