"""Wheel-sorter-only demonstration on the current Parcel Sorting Map."""

import random
from pathlib import Path

from isaacsim import SimulationApp
from project_config.simulation_config import HEADLESS

simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage
from project_config.robot_config import CARD_BOX_USD

ISAAC_SIM_DIR = Path(__file__).resolve().parent
WORLD_USD = ISAAC_SIM_DIR / "usd" / "Parcel_Sorting_Map" / "Parcel_Sorting_Map.usd"

BOX_SPAWN_POSITION = (-0.5, 0.0, 1.2)
BOX_SCALE_XYZ = (0.75, 0.75, 0.5)
BOX_MASS_KG = 15.0
BOX_SPAWN_INTERVAL_SECONDS = 2.0

BOX_ID_TO_TRACK = {
    1: "01",
    2: "02",
    3: "03",
    4: None,
}

SORTER_APPROACH_THRESHOLD_M = 0.45
SORTER_RESET_THRESHOLD_M = 0.70

enable_extension("isaacsim.asset.gen.conveyor")
simulation_app.update()

from cargo.box_spawner import spawn_box_with_id
from equipment.conveyor.conveyor_controller import ConveyorController
from equipment.wheel_sorter.wheel_sorter_controller import WheelSorterController


def _print_demo_config():
    print()
    print("============================================")
    print(" WHEEL SORTER RANDOM BOX DEMO READY")
    print("============================================")
    print(f"[WORLD] {WORLD_USD}")
    print(f"[BOX] asset: {CARD_BOX_USD}")
    print(f"[BOX] spawn: {BOX_SPAWN_POSITION}")
    print(f"[BOX] scale: {BOX_SCALE_XYZ}")
    print(f"[BOX] interval: {BOX_SPAWN_INTERVAL_SECONDS:.1f} s")
    print("[ROUTE] box_id=1 -> ConveyorTrack_01")
    print("[ROUTE] box_id=2 -> ConveyorTrack_02")
    print("[ROUTE] box_id=3 -> ConveyorTrack_03")
    print("[ROUTE] box_id=4 -> pass all sorters")
    print("[SORTER] SorterSpeed=-1.0")
    print("[SORTER] STRAIGHT=(1, 0, 0)")
    print("[SORTER] DIVERT=(1, -2, 0)")
    print(f"[SORTER] approach threshold={SORTER_APPROACH_THRESHOLD_M:.2f} m")
    print(f"[SORTER] reset threshold={SORTER_RESET_THRESHOLD_M:.2f} m")
    print("[INFO] AMR/P3020 controllers are NOT started in this demo")
    print("============================================")
    print()


def main():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(f"Parcel Sorting Map not found: {WORLD_USD}")

    if open_stage(str(WORLD_USD)) is False:
        raise RuntimeError(f"Failed to open stage: {WORLD_USD}")

    for _ in range(5):
        simulation_app.update()

    world = World(stage_units_in_meters=1.0)

    conveyor = ConveyorController()
    sorter = WheelSorterController(
        approach_threshold=SORTER_APPROACH_THRESHOLD_M,
        reset_threshold=SORTER_RESET_THRESHOLD_M,
    )

    world.reset()
    world.play()

    for _ in range(5):
        world.step(render=True)

    conveyor.setup()
    sorter.setup()
    conveyor.start()
    sorter.start()

    conveyor.verify()
    sorter.verify()
    _print_demo_config()

    stage = omni.usd.get_context().get_stage()
    box_paths = []
    box_counter = 0
    spawn_elapsed = 0.0

    try:
        while simulation_app.is_running():
            dt = float(world.get_physics_dt())
            spawn_elapsed += dt

            if spawn_elapsed >= BOX_SPAWN_INTERVAL_SECONDS:
                spawn_elapsed -= BOX_SPAWN_INTERVAL_SECONDS
                box_counter += 1

                box_id = random.choice((1, 2, 3, 4))
                box_path = f"/World/SorterDemoBoxes/box_{box_counter:04d}"

                spawn_box_with_id(
                    stage,
                    box_path,
                    BOX_SPAWN_POSITION,
                    box_id=box_id,
                    asset_url=CARD_BOX_USD,
                    scale_xyz=BOX_SCALE_XYZ,
                    mass_kg=BOX_MASS_KG,
                )
                box_paths.append(box_path)

                target = BOX_ID_TO_TRACK[box_id]
                target_text = f"ConveyorTrack_{target}" if target is not None else "PASS_ALL"
                print(
                    f"[DEMO] spawned box #{box_counter} "
                    f"box_id={box_id} target={target_text}"
                )

            sorter.update_boxes(box_paths, BOX_ID_TO_TRACK)
            world.step(render=True)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Ctrl+C received")

    finally:
        sorter.reset_all()
        conveyor.stop()
        world.stop()
        simulation_app.close()
        print("[SYSTEM] sorter demo closed")


if __name__ == "__main__":
    main()
