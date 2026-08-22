"""Runtime configuration for the single IW Hub navigation test."""


# NVIDIA's official Isaac Sim 5.1 IW Hub Navigation sample.
# The navigation scene contains the IW Hub setup used by NVIDIA Nav2,
# including the front/back 2D LiDAR configuration and ROS 2 publishers.
# We reference the robot setup from this scene instead of creating LiDARs
# ourselves or changing their sensor parameters.
IW_HUB_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Samples/ROS2/Scenario/"
    "iw_hub_warehouse_navigation.usd"
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
