from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS

# SimulationApp must be created before most Isaac Sim imports.
simulation_app = SimulationApp({"headless": HEADLESS})

from isaacsim.core.api import World

from project_config.robot_config import ROBOT_REGISTRY
from world_setup import setup_world


def main():
    world = World(stage_units_in_meters=1.0)

    setup_world(world)

    print(f"[INFO] configured robots: {len(ROBOT_REGISTRY)}")
    for robot in ROBOT_REGISTRY:
        print(
            f" - {robot['name']} | "
            f"{robot['type']} | "
            f"{robot['role']} | "
            f"{robot['namespace']}"
        )

    world.reset()
    print("[INFO] Simulation START")

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
