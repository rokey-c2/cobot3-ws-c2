"""Run the custom warehouse with NVIDIA's official IW Hub Nav2 robot setup."""

from pathlib import Path

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS


simulation_app = SimulationApp({"headless": HEADLESS})


import numpy as np
import omni.graph.core as og
import omni.usd

from pxr import Gf, UsdGeom

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
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

# Conveyor runtime support. The UI extension is not required for normal runs.
enable_extension("isaacsim.asset.gen.conveyor")

# Enable PhysX authoring UI when Isaac Sim is started through run_isaac.sh.
# This makes Physics properties and the Physics entries under the Add button
# available without manually enabling the extensions from the Extensions window.
enable_extension("omni.kit.property.physx")
enable_extension("omni.physx.ui")
enable_extension("omni.physx.supportui")
enable_extension("omni.usdphysics.ui")

simulation_app.update()


from cargo.cargo_pod_physics import add_cargo_pod_physics
from equipment.conveyor.conveyor_controller import ConveyorController
from equipment.wheel_sorter.wheel_sorter_controller import WheelSorterController
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

        mass_kg = float(config.get("mass_kg", 20.0))
        add_cargo_pod_physics(stage, prim_path, mass_kg=mass_kg)

        print(
            f"[CARGO] spawned {name} at {config['spawn_xyz']} "
            f"from {usd_path}"
        )


def _spawn_conveyor_test_cube(world):
    """Spawn a rigid-body cube with collision for conveyor testing."""

    cube = DynamicCuboid(
        prim_path="/World/ConveyorTestCube",
        name="conveyor_test_cube",
        position=np.array([-0.5, 0.0, 1.2]),
        scale=np.array([0.4, 0.4, 0.4]),
    )

    world.scene.add(cube)

    print(
        "[TEST CUBE] spawned at x=-0.5, y=0.0, z=1.2 "
        "scale=0.4 with rigid body + collision"
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
    _spawn_conveyor_test_cube(world)

    conveyor_speed = 1.0
    sorter_speed = -1.0

    conveyor = ConveyorController(speed=conveyor_speed)
    sorter = WheelSorterController(
        toggle_steps=120,
        speed=sorter_speed,
    )

    conveyor.setup()
    sorter.setup()

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

    # Re-apply equipment values after reset so the test starts deterministically.
    conveyor.setup()
    sorter.setup()

    world.play()

    # Apply equipment values again after OmniGraph playback becomes active.
    conveyor.start()
    sorter.set_speed(sorter_speed)

    # Give NVIDIA's built-in front/back RTX LiDAR publishers time to start.
    for _ in range(30):
        world.step(render=True)

    print()
    print("============================================")
    print(" NVIDIA IW HUB NAVIGATION READY")
    print("============================================")
    print(f"[START] x={START_XY[0]:.6f}, y={START_XY[1]:.6f}")
    print(f"[GOAL ] x={GOAL_XY[0]:.6f}, y={GOAL_XY[1]:.6f}")
    print(f"[CONVEYOR] speed={conveyor_speed:.1f}")
    print(
        f"[SORTER] speed={sorter_speed:.1f}, "
        "binary switch toggles every 120 steps"
    )
    print("[TEST CUBE] position=(-0.5, 0.0, 1.2), scale=0.4")
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
            sorter.update()

    except KeyboardInterrupt:
        print()
        print("[SYSTEM] Ctrl+C received")

    finally:
        _ = agents
        conveyor.stop()
        print("[WORLD] stopping simulation")
        world.stop()
        simulation_app.close()
        print("[SYSTEM] Isaac Sim closed")


if __name__ == "__main__":
    main()
