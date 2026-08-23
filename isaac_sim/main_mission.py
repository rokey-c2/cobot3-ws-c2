"""Custom warehouse + NVIDIA IW Hub + mission bridge.

Final order:
local start -> cargo -> lift -> Nav2 delivery -> P3020 action
-> Nav2 return -> local precision return -> lift down.
"""

from pathlib import Path

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS

simulation_app = SimulationApp({"headless": HEADLESS})

import omni.graph.core as og
import omni.usd
import rclpy

from pxr import Gf, UsdGeom
from rclpy.node import Node
from std_msgs.msg import String

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage

from project_config.robot_config import (
    CARGO_REGISTRY,
    IW_HUB_USD,
    ROBOT_REGISTRY,
)


ISAAC_SIM_DIR = Path(__file__).resolve().parent
WORLD_USD = (
    ISAAC_SIM_DIR
    / "usd"
    / "enva_small_warehouse_p3020_marker"
    / "World0.usd"
)

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
enable_extension("isaacsim.robot.wheeled_robots")
simulation_app.update()

from cargo.cargo_pod_physics import add_cargo_pod_physics
from robots.iw_hub.iw_hub_mission_agent import MissionIwHubAgent


def _create_clock_graph():
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


def _spawn_cargo_pods():
    if not CARGO_REGISTRY:
        return

    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World/Cargo")

    for config in CARGO_REGISTRY:
        name = config["name"]
        usd_path = ISAAC_SIM_DIR / config["usd"]

        if not usd_path.is_file():
            raise FileNotFoundError(f"Cargo USD not found: {usd_path}")

        prim_path = f"/World/Cargo/{name}"
        prim = stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddReference(str(usd_path))

        xform = UsdGeom.Xformable(prim)
        xform.AddTranslateOp().Set(Gf.Vec3d(*config["spawn_xyz"]))

        yaw = float(config.get("spawn_yaw", 0.0))
        if yaw != 0.0:
            xform.AddRotateZOp().Set(yaw)

        add_cargo_pod_physics(
            stage,
            prim_path,
            mass_kg=float(config.get("mass_kg", 20.0)),
        )


class AmrMissionBridge(Node):
    def __init__(self, agent):
        super().__init__("isaac_amr_mission_bridge")
        self.agent = agent
        self.last_state = None

        self.create_subscription(
            String,
            "/amr_a/pickup_command",
            self._command_callback,
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            "/amr_a/pickup_state",
            10,
        )
        self.create_timer(0.2, self._publish_state)

    def _command_callback(self, message):
        command = message.data.strip().upper()

        if command == "PICKUP":
            self.agent.request_pickup()
        elif command == "RETURN_DOCK":
            self.agent.request_return_dock()
        elif command == "LOWER":
            self.agent.request_lower()
        elif command == "RESET":
            self.agent.reset_mission()
        else:
            self.get_logger().warning(
                f"unknown command: {command}"
            )

    def _publish_state(self):
        state = self.agent.get_mission_state()
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

        if state != self.last_state:
            self.get_logger().info(f"pickup state: {state}")
            self.last_state = state


def main():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(
            f"Warehouse USD not found: {WORLD_USD}"
        )

    if open_stage(str(WORLD_USD)) is False:
        raise RuntimeError(
            f"Failed to open warehouse USD: {WORLD_USD}"
        )

    for _ in range(5):
        simulation_app.update()

    world = World(stage_units_in_meters=1.0)

    _create_clock_graph()
    _spawn_cargo_pods()

    agents = []
    for config in ROBOT_REGISTRY:
        if config["type"] != "iw_hub":
            continue

        agent = MissionIwHubAgent(
            config,
            world,
            IW_HUB_USD,
        )
        agent.setup()
        agents.append(agent)

    if not agents:
        raise RuntimeError("No IW Hub robot found")

    world.reset()
    for agent in agents:
        agent.post_reset()

    world.play()

    for _ in range(30):
        world.step(render=True)

    rclpy.init(args=None)
    bridge = AmrMissionBridge(agents[0])

    print()
    print("============================================")
    print(" IW HUB CARGO + NAV2 + P3020 MISSION READY")
    print("============================================")
    print("[START] IW Hub: (1.5, 0.0)")
    print("[LOCAL] start -> cargo -> lift")
    print("[CARGO] original: (10.5, -1.5), yaw=0 deg")
    print("[NAV2] starts only after PICKUP_DONE")
    print("[DELIVERY] (1.30104, -0.06065)")
    print("[RETURN] Nav2 -> cargo area -> local precision dock")
    print("[LOCAL] verify original cargo pose -> lift down")
    print("[ROS2] /amr_a/pickup_command")
    print("[ROS2] /amr_a/pickup_state")
    print("============================================")

    try:
        while simulation_app.is_running():
            rclpy.spin_once(bridge, timeout_sec=0.0)

            dt = float(world.get_physics_dt())
            for agent in agents:
                agent.on_physics_step(dt)

            world.step(render=True)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Ctrl+C received")

    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
