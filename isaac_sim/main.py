"""Run the project world with one Nav2-controlled IW Hub AMR."""

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS


simulation_app = SimulationApp({"headless": HEADLESS})


from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.utils.extensions import enable_extension
import omni.graph.core as og
import numpy as np

from project_config.robot_config import (
    CARGO_REGISTRY,
    IW_HUB_USD,
    ROBOT_REGISTRY,
    TEST_OBSTACLES,
)


enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
simulation_app.update()


from robots.iw_hub.iw_hub_agent import IwHubAgent
from sensors.lidar_sensor import IwHubLidarRos2Publisher
from cargo.container_payload import CargoContainerPayload


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

    for obstacle in TEST_OBSTACLES:
        world.scene.add(
            FixedCuboid(
                prim_path=f"/World/Obstacles/{obstacle['name']}",
                name=obstacle["name"],
                position=np.array(obstacle["position"], dtype=float),
                scale=np.array(obstacle["scale"], dtype=float),
                color=np.array(obstacle["color"], dtype=float),
            )
        )
        print(
            f"[WORLD] obstacle: {obstacle['name']} "
            f"position={obstacle['position']} scale={obstacle['scale']}"
        )

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

    agents_by_name = {agent.name: agent for agent in agents}
    cargo_payloads = []
    for config in CARGO_REGISTRY:
        agent = agents_by_name[config["robot_name"]]
        cargo_payloads.append(
            CargoContainerPayload(
                lift_prim_path=agent.lift_prim_path,
                goal_xy=config["goal_xy"],
                prim_path=f"/World/Cargo/{config['name']}",
                lift_offset=config["lift_offset"],
            )
        )

    lidar_publishers = [
        IwHubLidarRos2Publisher(
            # The referenced IW Hub articulation moves at iw_hub_sensors.
            # Mount the LiDAR there so its world pose follows the chassis.
            parent_prim_path=agent.sensor_prim_path,
            namespace=agent.name,
        )
        for agent in agents
    ]
    simulation_app.update()
    world.play()

    print("[IW HUB] autonomous-navigation simulation started")
    print("[IW HUB] input : /amr_a/drive_cmd_vel")
    print("[IW HUB] output: /amr_a/odom, /amr_a/scan, /clock")
    print("[CARGO] container_01 loaded with boxes")
    print("[CARGO] mission: lift -> navigate -> lower/place at (6.0, 0.0)")
    try:
        while simulation_app.is_running():
            # Keep RTX render products and the embedded drive graph updating.
            simulation_app.update()
            for payload in cargo_payloads:
                payload.update()
    finally:
        _ = (lidar_publishers, cargo_payloads)
        world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
