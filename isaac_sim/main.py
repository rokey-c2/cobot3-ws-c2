"""Run the custom warehouse with NVIDIA's official IW Hub Nav2 robot setup."""

from pathlib import Path

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS


simulation_app = SimulationApp({"headless": HEADLESS})


import omni.graph.core as og
import omni.usd

from pxr import Gf, UsdGeom

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage

from project_config.robot_config import (
    CARGO_REGISTRY,
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


# Required by NVIDIA's IW Hub Navigation sample ROS 2 / RTX setup.
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


def _spawn_cargo_pods():
    """Reference configured cargo USD assets into the current warehouse."""

    if not CARGO_REGISTRY:
        return

    stage = omni.usd.get_context().get_stage()
    cargo_root = UsdGeom.Xform.Define(stage, "/World/Cargo")
    _ = cargo_root

    for config in CARGO_REGISTRY:
        name = config["name"]
        usd_path = ISAAC_SIM_DIR / config["usd"]

        if not usd_path.is_file():
            raise FileNotFoundError(f"Cargo USD not found: {usd_path}")

        prim_path = f"/World/Cargo/{name}"
        prim = stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddReference(str(usd_path))

        xform = UsdGeom.Xformable(prim)
        translate_op = xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*config["spawn_xyz"]))

        yaw = float(config.get("spawn_yaw", 0.0))
        if yaw != 0.0:
            rotate_op = xform.AddRotateZOp()
            rotate_op.Set(yaw)

        print(
            f"[CARGO] spawned {name} at {config['spawn_xyz']} "
            f"from {usd_path}"
        )


def main():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(
            f"Warehouse USD not found: {WORLD_USD}"
        )

    print()
    print("============================================")
    print("[WORLD] loading custom warehouse")
    print(f"[WORLD] USD: {WORLD_USD}")
    print("[IW HUB] NVIDIA Navigation robot source")
    print(f"[IW HUB] USD: {IW_HUB_USD}")
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
    _spawn_cargo_pods()

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

    # Give NVIDIA's built-in front/back RTX LiDAR publishers time to start.
    for _ in range(30):
        world.step(render=True)

    print()
    print("============================================")
    print(" NVIDIA IW HUB NAVIGATION READY")
    print("============================================")
    print(f"[START] x={START_XY[0]:.6f}, y={START_XY[1]:.6f}")
    print(f"[GOAL ] x={GOAL_XY[0]:.6f}, y={GOAL_XY[1]:.6f}")
    print()
    print("[NVIDIA DEFAULT NAVIGATION TOPICS]")
    print("  command : /cmd_vel")
    print("  odom    : /chassis/odom")
    print("  lidar   : /front_2d_lidar/scan")
    print("  lidar   : /back_2d_lidar/scan")
    print()
    print("[INFO] LiDAR positions/ranges/orientations are not recreated here.")
    print("[INFO] They come from NVIDIA's IW Hub Navigation sample unchanged.")
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
