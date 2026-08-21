"""Open a saved USD environment scene (usd/env_temp_*) directly in Isaac Sim."""

import argparse
from pathlib import Path

from isaacsim import SimulationApp

ISAAC_SIM_DIR = Path(__file__).resolve().parent.parent
DEFAULT_USD = ISAAC_SIM_DIR / "usd" / "env_temp_p3020_not_gripper_1" / "World0.usd"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usd",
        type=str,
        default=str(DEFAULT_USD),
        help="Path to the USD scene to open.",
    )
    parser.add_argument("--headless", action="store_true", help="Run without the GUI.")
    return parser.parse_args()


def main():
    args = parse_args()
    usd_path = str(Path(args.usd).resolve())

    simulation_app = SimulationApp({"headless": args.headless})

    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import open_stage

    print(f"[INFO] Opening stage: {usd_path}")
    open_stage(usd_path)

    world = World(stage_units_in_meters=1.0)
    world.reset()
    print("[INFO] Simulation START")

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
