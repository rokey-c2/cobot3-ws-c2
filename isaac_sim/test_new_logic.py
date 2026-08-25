"""Standalone test harness for the AMR 3-phase movement, multi-box P3020
pick-place setup, and wheel-sorter logic, using TEMP TEST placeholder
coordinates against the current WIP Parcel_Sorting_Map (final AMR/cargo/
conveyor layout not baked in yet -- see the TEMP TEST VALUE comments in
project_config/robot_config.py and robots/iw_hub/iw_hub_mission_agent.py).

Mirrors main_mission.py's setup but replaces its ROS2-command while-loop
with a scripted test sequence, and manually teleports the AMR to skip the
real Nav2/AGV leg (no Nav2 stack running in this test). Does NOT exercise
the vision-based pick-and-place loop itself (run_until_cargo_empty) --
that needs the external YOLO box detector node publishing /box_pixel,
which isn't started here.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS

simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import numpy as np

from pxr import Sdf, UsdGeom

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage

from project_config.robot_config import (
    CARGO_REGISTRY,
    IW_HUB_USD,
    PARCEL_REGISTRY,
    ROBOT_REGISTRY,
)

ISAAC_SIM_DIR = Path(__file__).resolve().parent
WORLD_USD = ISAAC_SIM_DIR / "usd" / "Parcel_Sorting_Map" / "Parcel_Sorting_Map.usd"

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
enable_extension("isaacsim.robot.wheeled_robots")
simulation_app.update()

from cargo.cargo_guard_clone import resolve_parcel_layer, spawn_cargo_guard_clone
from cargo.cargo_pod_physics import add_parcel_asset
import robots.iw_hub.iw_hub_mission_agent as iw_hub_mission_module
from robots.iw_hub.iw_hub_mission_agent import MissionIwHubAgent
from robots.p3020.p3020_mission_agent import P3020PickPlaceAgent
from equipment.conveyor.conveyor_controller import ConveyorController
from equipment.wheel_sorter.wheel_sorter_controller import WheelSorterController


PASS = []
FAIL = []


def check(name, ok, detail=""):
    tag = "PASS" if ok else "FAIL"
    (PASS if ok else FAIL).append(name)
    print(f"[TEST][{tag}] {name} {detail}")


def spawn_cargo_and_parcels(stage):
    UsdGeom.Xform.Define(stage, "/World/Cargo")
    config = CARGO_REGISTRY[0]
    prim_path = f"/World/Cargo/{config['name']}"
    spawn_cargo_guard_clone(
        stage,
        prim_path,
        spawn_xyz=config["spawn_xyz"],
        spawn_yaw=float(config.get("spawn_yaw", 0.0)),
        source_name=config["source_prim_name"],
        mass_kg=float(config.get("mass_kg", 20.0)),
    )
    iw_hub_mission_module.CARGO_PRIM_PATH = prim_path

    cargo_xyz = config["spawn_xyz"]
    resolved = resolve_parcel_layer(
        stage,
        prim_path,
        PARCEL_REGISTRY,
        cargo_center_xy=(float(cargo_xyz[0]), float(cargo_xyz[1])),
    )

    UsdGeom.Xform.Define(stage, "/World/Cargo/Parcels")
    # TEST-ONLY: real destinations will come from a "destination" attribute
    # the user sets ahead of time on each parcel prim (see
    # p3020_mission_agent.py's PARCEL_DESTINATION_ATTR) -- here we just
    # cycle through A/B/C/D so every sorter path gets exercised.
    destinations = ["A", "B", "C", "D"]
    for i, parcel_cfg in enumerate(resolved):
        parcel_path = f"/World/Cargo/Parcels/{parcel_cfg['name']}"
        add_parcel_asset(
            stage,
            parcel_path,
            asset_url=parcel_cfg["usd"],
            center=parcel_cfg["spawn_xyz"],
            max_size=parcel_cfg["max_size_xyz"],
            mass_kg=float(parcel_cfg.get("mass_kg", 15.0)),
        )
        dest = destinations[i % len(destinations)]
        stage.GetPrimAtPath(parcel_path).CreateAttribute(
            "destination", Sdf.ValueTypeNames.String
        ).Set(dest)
        print(f"[TEST] parcel {parcel_cfg['name']} -> destination={dest}")

    return prim_path


def main():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(f"Map USD not found: {WORLD_USD}")
    if open_stage(str(WORLD_USD)) is False:
        raise RuntimeError(f"Failed to open map USD: {WORLD_USD}")
    for _ in range(5):
        simulation_app.update()

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    conveyor = ConveyorController(speed=1.0)
    sorter = WheelSorterController(regions=("A", "B", "C"), sorter_speed=1.0)
    conveyor.setup()
    sorter.setup()
    check(
        "conveyor.setup() found segments",
        len(conveyor._graph_paths) > 0,
        f"({len(conveyor._graph_paths)} found)",
    )
    check(
        "sorter.setup() found 3 region units",
        len(sorter.units_by_region) == 3,
        f"({list(sorter.units_by_region.keys())})",
    )

    spawn_cargo_and_parcels(stage)

    agent = MissionIwHubAgent(ROBOT_REGISTRY[0], world, IW_HUB_USD)
    agent.setup()

    p3020_agent = P3020PickPlaceAgent(world)
    try:
        p3020_agent.setup()
        p3020_setup_ok = True
    except Exception as exc:
        p3020_setup_ok = False
        print(f"[TEST] p3020_agent.setup() raised: {exc!r}")
    check("p3020_agent.setup() (IK/gripper/camera init)", p3020_setup_ok)

    world.reset()
    agent.post_reset()
    if p3020_setup_ok:
        try:
            p3020_agent.post_reset()
            p3020_post_reset_ok = True
        except Exception as exc:
            p3020_post_reset_ok = False
            print(f"[TEST] p3020_agent.post_reset() raised: {exc!r}")
        check("p3020_agent.post_reset() (ready-pose IK solve)", p3020_post_reset_ok)

    conveyor.setup()
    sorter.setup()
    world.play()
    conveyor.start()

    for _ in range(30):
        world.step(render=True)

    dt = float(world.get_physics_dt())

    # ---- Phase 1: AMR local precision -- dock at cargo, lift ----
    agent.request_pickup()
    max_steps = int(30.0 / dt)
    for _ in range(max_steps):
        agent.on_physics_step(dt)
        world.step(render=True)
        if agent.mission_state in {"PICKUP_DONE", "ERROR"}:
            break
    check(
        "Phase 1 (AMR local): reach PICKUP_DONE",
        agent.mission_state == "PICKUP_DONE",
        f"(state={agent.mission_state}, last_error={agent.get_last_error()})",
    )

    # ---- Phase 2 stand-in: no real Nav2 running here, teleport near the
    # arm's reach to simulate "Nav2 got us there" ----
    if agent.mission_state == "PICKUP_DONE":
        pos, orient = agent.robot.get_world_pose()
        teleport_pos = np.array([1.1, -1.5, pos[2]])
        agent.robot.set_world_pose(position=teleport_pos, orientation=orient)
        world.step(render=True)
        print(f"[TEST] teleported AMR to simulate Nav2 arrival: {teleport_pos}")

        # ---- Phase 3: arrival confirmation only, no local maneuver ----
        agent.request_conveyor_dock()
        agent.on_physics_step(dt)
        world.step(render=True)
        check(
            "Phase 3: reach CONVEYOR_DOCK_DONE",
            agent.mission_state == "CONVEYOR_DOCK_DONE",
            f"(state={agent.mission_state}, last_error={agent.get_last_error()})",
        )
    else:
        check("Phase 3: reach CONVEYOR_DOCK_DONE", False, "(skipped, phase 1 failed)")

    # ---- Wheel sorter routing (independent of vision/arm) ----
    for dest in ["A", "B", "C", "D", "Z"]:
        try:
            sorter.route_box(dest)
            routing_ok = True
        except Exception as exc:
            routing_ok = False
            print(f"[TEST] sorter.route_box({dest!r}) raised: {exc!r}")
        check(f"sorter.route_box('{dest}') runs without error", routing_ok)

    print("\n" + "=" * 60)
    print(f"RESULT: {len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED:", FAIL)
    print("=" * 60)

    if HEADLESS:
        world.stop()
        simulation_app.close()
    else:
        print(
            "\n[TEST] GUI mode: leaving the final state on screen. "
            "Close the Isaac Sim window (or Ctrl+C here) when done looking."
        )
        try:
            while simulation_app.is_running():
                world.step(render=True)
        except KeyboardInterrupt:
            pass
        world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
