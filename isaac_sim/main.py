"""Run the project world with one Nav2-controlled IW Hub AMR."""

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS


simulation_app = SimulationApp({"headless": HEADLESS})


from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import is_stage_loading, open_stage
import omni.graph.core as og
import omni.usd

from project_config.robot_config import (
    IW_HUB_USD,
    ROBOT_REGISTRY,
    WORLD_USD,
)


enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
simulation_app.update()


from robots.iw_hub.iw_hub_agent import IwHubAgent
from sensors.lidar_sensor import IwHubLidarRos2Publisher


def _load_world_stage():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(f"World USD not found: {WORLD_USD}")
    if not IW_HUB_USD.is_file():
        raise FileNotFoundError(f"IW Hub USD not found: {IW_HUB_USD}")

    print(f"[WORLD] loading: {WORLD_USD}")
    open_stage(str(WORLD_USD))
    while is_stage_loading():
        simulation_app.update()

    # The collected P3020 world still contains an older ForkliftB prim.
    # Remove every ForkliftB prim before the IW Hub is spawned.
    stage = omni.usd.get_context().get_stage()
    forklift_paths = [
        prim.GetPath()
        for prim in stage.Traverse()
        if "forklift" in prim.GetName().lower()
    ]
    for prim_path in sorted(forklift_paths, key=str, reverse=True):
        stage.RemovePrim(prim_path)
        print(f"[WORLD] removed legacy ForkliftB prim: {prim_path}")


def _create_clock_graph():
    """Publish Isaac simulation time on /clock for Nav2."""

    og.Controller.edit(
        {"graph_path": "/World/ROS2ClockGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                (
                    "sim_time",
                    "isaacsim.core.nodes.IsaacReadSimulationTime",
                ),
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


def main():
    _load_world_stage()
    world = World(stage_units_in_meters=1.0)
    _create_clock_graph()

    agents = []
    for config in ROBOT_REGISTRY:
        if config["type"] != "iw_hub":
            continue
        agent = IwHubAgent(config, world, IW_HUB_USD)
        agent.setup()
        agents.append(agent)

    world.reset()
    for agent in agents:
        agent.post_reset()

    lidar_publishers = [
        IwHubLidarRos2Publisher(
            parent_prim_path=agent.prim_path,
            namespace=agent.name,
        )
        for agent in agents
    ]
    simulation_app.update()
    world.play()

    print("[IW HUB] autonomous-navigation simulation started")
    print("[IW HUB] input : /amr_a/drive_cmd_vel")
    print("[IW HUB] output: /amr_a/odom, /amr_a/scan, /clock")
    try:
        while simulation_app.is_running():
            # Keep RTX render products and the embedded drive graph updating.
            simulation_app.update()
    finally:
        _ = lidar_publishers
        world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
