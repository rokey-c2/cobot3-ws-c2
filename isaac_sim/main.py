"""Run the custom warehouse with one default IW Hub Sensor AMR."""

from pathlib import Path

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS


simulation_app = SimulationApp({"headless": HEADLESS})


import omni.graph.core as og

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage

from project_config.robot_config import (
    GOAL_XY,
    IW_HUB_USD,
    ROBOT_REGISTRY,
    START_XY,
)


ISAAC_SIM_DIR = Path(__file__).resolve().parent
WORLD_USD = (
    ISAAC_SIM_DIR
    / "usd"
    / "enva_small_warehouse_p3020_marker"
    / "World0.usd"
)


# Required by the IW Hub asset's built-in ROS 2 and RTX sensors.
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
simulation_app.update()


from robots.iw_hub.iw_hub_agent import IwHubAgent


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
                (
                    "sim_time.outputs:simulationTime",
                    "clock.inputs:timeStamp",
                ),
            ],
        },
    )

    print("[ROS2] /clock publisher created")


def main():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(
            f"Warehouse USD not found: {WORLD_USD}"
        )

    if not IW_HUB_USD.is_file():
        raise FileNotFoundError(
            f"IW Hub USD not found: {IW_HUB_USD}"
        )

    print()
    print("============================================")
    print("[WORLD] loading custom warehouse")
    print(f"[WORLD] USD: {WORLD_USD}")
    print("============================================")
    print()

    result = open_stage(str(WORLD_USD))

    if result is False:
        raise RuntimeError(
            f"Failed to open warehouse USD: {WORLD_USD}"
        )

    for _ in range(5):
        simulation_app.update()

    world = World(stage_units_in_meters=1.0)
    print("[WORLD] custom warehouse loaded")

    _create_clock_graph()

    agents = []

    for config in ROBOT_REGISTRY:
        if config["type"] != "iw_hub":
            continue

        print(
            f"[IW HUB] spawning {config['name']} "
            f"at {config['spawn_xyz']}"
        )

        agent = IwHubAgent(
            config,
            world,
            IW_HUB_USD,
        )
        agent.setup()
        agents.append(agent)

    if not agents:
        raise RuntimeError(
            "No IW Hub robot found in ROBOT_REGISTRY"
        )

    print("[WORLD] resetting simulation")
    world.reset()

    for agent in agents:
        agent.post_reset()

    world.play()

    # Give the default IW Hub RTX sensors time to start publishing.
    for _ in range(30):
        world.step(render=True)

    print()
    print("============================================")
    print(" DEFAULT IW HUB SENSOR NAVIGATION READY")
    print("============================================")
    print(f"[START] x={START_XY[0]:.6f}, y={START_XY[1]:.6f}")
    print(f"[GOAL ] x={GOAL_XY[0]:.6f}, y={GOAL_XY[1]:.6f}")
    print()
    print("[IW HUB DEFAULT ROS TOPICS]")
    print("  command : /cmd_vel")
    print("  odom    : /chassis/odom")
    print("  lidar   : /front_2d_lidar/scan")
    print("  lidar   : /back_2d_lidar/scan")
    print()
    print("[INFO] No custom LiDAR is created by this project.")
    print("[INFO] The IW Hub Sensor asset's built-in LiDAR is used unchanged.")
    print("============================================")
    print()

    try:
        while simulation_app.is_running():
            world.step(render=True)

    except KeyboardInterrupt:
        print()
        print("[SYSTEM] Ctrl+C received")

    finally:
        _ = agents
        print("[WORLD] stopping simulation")
        world.stop()
        simulation_app.close()
        print("[SYSTEM] Isaac Sim closed")


if __name__ == "__main__":
    main()
