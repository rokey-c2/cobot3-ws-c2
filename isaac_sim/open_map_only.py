"""Open the new parcel_sorting_map with nothing else spawned.

Purpose: let the user look around the bare map in the Isaac Sim viewport
and (once rviz2/map_server are also up) in rviz2, to read off real-world
coordinates for where AMR/cargo/parcels should go. main.py and
main_mission.py both assume the OLD map's prim names (conveyor/wheel
sorter/cargo-guard paths, ROBOT_REGISTRY spawn positions) which do not
match this new map yet -- this script is deliberately separate from both
so it can't fail on any of that. Delete once the registries/controllers
are updated for the new map and main_mission.py can be used directly.
"""

from pathlib import Path

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS


simulation_app = SimulationApp({"headless": HEADLESS})

import omni.graph.core as og

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage


ISAAC_SIM_DIR = Path(__file__).resolve().parent
WORLD_USD = ISAAC_SIM_DIR / "usd" / "parcel_sorting_map" / "parcel_sorting_map.usd"

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")

# Without these, the Property panel's "Add" button has no Physics/Collider
# entries at all (not related to whether an asset is a custom USD file --
# this is purely a UI extension gate). Matches main.py's setup.
enable_extension("omni.kit.property.physx")
enable_extension("omni.physx.ui")
enable_extension("omni.physx.supportui")
enable_extension("omni.usdphysics.ui")
simulation_app.update()


def _create_clock_graph():
    """Publish Isaac Sim simulation time on /clock (nothing else needs ROS2 yet)."""

    og.Controller.edit(
        {"graph_path": "/World/ROS2ClockGraph", "evaluator_name": "execution"},
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


def main():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(f"Map USD not found: {WORLD_USD}")

    print(f"[WORLD] opening {WORLD_USD}")
    if open_stage(str(WORLD_USD)) is False:
        raise RuntimeError(f"Failed to open map USD: {WORLD_USD}")

    for _ in range(5):
        simulation_app.update()

    world = World(stage_units_in_meters=1.0)
    _create_clock_graph()

    world.reset()
    world.play()

    print()
    print("============================================")
    print(" MAP-ONLY VIEW READY (no robots/cargo spawned)")
    print(" /clock is publishing so rviz2's use_sim_time works")
    print("============================================")
    print()

    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        print("\n[SYSTEM] Ctrl+C received")
    finally:
        world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
