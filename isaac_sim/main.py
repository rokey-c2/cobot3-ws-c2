"""Run the custom warehouse world with one Nav2-controlled IW Hub AMR."""

from pathlib import Path

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS


# ---------------------------------------------------------------------
# 1. Isaac Sim 시작
# ---------------------------------------------------------------------

simulation_app = SimulationApp({"headless": HEADLESS})


# ---------------------------------------------------------------------
# 2. Isaac Sim imports
#    SimulationApp 생성 이후 import
# ---------------------------------------------------------------------

import omni.graph.core as og

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage

from project_config.robot_config import (
    CARGO_REGISTRY,
    IW_HUB_USD,
    ROBOT_REGISTRY,
)


# ---------------------------------------------------------------------
# 3. Warehouse USD
# ---------------------------------------------------------------------

ISAAC_SIM_DIR = Path(__file__).resolve().parent
WORLD_USD = (
    ISAAC_SIM_DIR
    / "usd"
    / "enva_small_warehouse_p3020_marker"
    / "World0.usd"
)


# ---------------------------------------------------------------------
# 4. ROS2 / RTX Sensor Extension 활성화
# ---------------------------------------------------------------------

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")

simulation_app.update()


# Extension 활성화 이후 project module import
from robots.iw_hub.iw_hub_agent import IwHubAgent
from sensors.lidar_sensor import IwHubLidarRos2Publisher
from cargo.container_payload import CargoContainerPayload


# ---------------------------------------------------------------------
# ROS2 /clock publisher
# ---------------------------------------------------------------------

def _create_clock_graph():
    """Publish Isaac Sim simulation time on /clock."""

    og.Controller.edit(
        {
            "graph_path": "/World/ROS2ClockGraph",
            "evaluator_name": "execution",
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "clock.inputs:execIn"),
                ("context.outputs:context", "clock.inputs:context"),
                ("sim_time.outputs:simulationTime", "clock.inputs:timeStamp"),
            ],
        },
    )

    print("[ROS2] /clock publisher created")


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

def main():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(f"Warehouse USD not found: {WORLD_USD}")

    if not IW_HUB_USD.is_file():
        raise FileNotFoundError(f"IW Hub USD not found: {IW_HUB_USD}")

    print()
    print("============================================")
    print("[WORLD] loading custom warehouse")
    print(f"[WORLD] USD: {WORLD_USD}")
    print("============================================")
    print()

    result = open_stage(str(WORLD_USD))

    if result is False:
        raise RuntimeError(f"Failed to open warehouse USD: {WORLD_USD}")

    for _ in range(5):
        simulation_app.update()

    print("[WORLD] custom warehouse loaded")

    world = World(stage_units_in_meters=1.0)
    print("[WORLD] Isaac World created from loaded Stage")

    _create_clock_graph()

    agents = []

    for config in ROBOT_REGISTRY:
        if config["type"] != "iw_hub":
            continue

        print()
        print(
            f"[IW HUB] spawning {config['name']} "
            f"at {config['spawn_xyz']}"
        )

        agent = IwHubAgent(config, world, IW_HUB_USD)
        agent.setup()
        agents.append(agent)

    if not agents:
        raise RuntimeError("No IW Hub robot found in ROBOT_REGISTRY")

    print("[WORLD] resetting simulation")
    world.reset()

    for agent in agents:
        agent.post_reset()

    print("[WORLD] reset complete")

    agents_by_name = {agent.name: agent for agent in agents}
    cargo_payloads = []

    for config in CARGO_REGISTRY:
        robot_name = config["robot_name"]

        if robot_name not in agents_by_name:
            print(
                f"[CARGO] skip {config['name']}: "
                f"robot '{robot_name}' not found"
            )
            continue

        agent = agents_by_name[robot_name]
        payload = CargoContainerPayload(
            lift_prim_path=agent.lift_prim_path,
            goal_xy=config["goal_xy"],
            prim_path=f"/World/Cargo/{config['name']}",
            lift_offset=config["lift_offset"],
        )
        cargo_payloads.append(payload)
        print(f"[CARGO] created: {config['name']}")

    lidar_publishers = []

    for agent in agents:
        print(f"[LIDAR] creating LiDAR for {agent.name}")
        lidar = IwHubLidarRos2Publisher(
            parent_prim_path=agent.sensor_prim_path,
            namespace=agent.name,
        )
        lidar_publishers.append(lidar)

    for _ in range(3):
        simulation_app.update()

    world.play()
    print("[WORLD] simulation playing")
    print("[LIDAR] warming up...")

    for _ in range(20):
        world.step(render=True)

    print()
    print("============================================")
    print(" IW HUB NAVIGATION SIMULATION STARTED")
    print("============================================")
    print()
    print(f"[WORLD] {WORLD_USD}")
    print()
    print("[ROS2]")
    print("  clock:")
    print("    /clock")
    print()
    print("  IW Hub:")
    print("    input : /amr_a/drive_cmd_vel")
    print("    output: /amr_a/odom")
    print("    output: /amr_a/scan")
    print()
    print("[RVIZ / NAV2]")
    print("  map:")
    print("    warehouse_navigation.yaml")
    print()
    print("[INFO] Camera publisher is NOT configured in this main.py.")
    print("[INFO] RViz Image displaying 'No Image' is therefore expected.")
    print()
    print("============================================")
    print()

    try:
        while simulation_app.is_running():
            world.step(render=True)
            for payload in cargo_payloads:
                payload.update()

    except KeyboardInterrupt:
        print()
        print("[SYSTEM] Ctrl+C received")

    finally:
        _ = (agents, lidar_publishers, cargo_payloads)
        print("[WORLD] stopping simulation")
        world.stop()
        simulation_app.close()
        print("[SYSTEM] Isaac Sim closed")


if __name__ == "__main__":
    main()
