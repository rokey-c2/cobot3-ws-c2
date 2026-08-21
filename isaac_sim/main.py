"""Run the project world with one Nav2-controlled IW Hub AMR."""

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS


simulation_app = SimulationApp({"headless": HEADLESS})


from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
import omni.graph.core as og

from project_config.robot_config import (
    IW_HUB_USD,
    ROBOT_REGISTRY,
)


enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
simulation_app.update()


from robots.iw_hub.iw_hub_agent import IwHubAgent
from sensors.lidar_sensor import IwHubLidarRos2Publisher


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
    if not IW_HUB_USD.is_file():
        raise FileNotFoundError(f"IW Hub USD not found: {IW_HUB_USD}")

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    print("[WORLD] empty test world + ground plane")
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
