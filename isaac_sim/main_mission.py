"""Custom warehouse + NVIDIA IW Hub + mission bridge.

Final order:
local start -> cargo -> lift -> Nav2 delivery -> P3020 action
-> Nav2 return -> local precision return -> lift down -> local spawn return.
"""

from pathlib import Path

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS

simulation_app = SimulationApp({"headless": HEADLESS})

import omni.graph.core as og
import omni.usd
import rclpy

from pxr import UsdGeom
from rclpy.node import Node
from std_msgs.msg import String

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
WORLD_USD = (
    ISAAC_SIM_DIR
    / "usd"
    / "warehouse_final_final"
    / "env_warehouse_only_arms.usd"
)

VISION_RGB_PUBLISH_INTERVAL_STEPS = 6

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
enable_extension("isaacsim.robot.wheeled_robots")
simulation_app.update()

from cargo.cargo_guard_clone import (
    resolve_parcel_layer,
    spawn_cargo_guard_clone,
)
from cargo.cargo_pod_physics import add_parcel_asset
import robots.iw_hub.iw_hub_mission_agent as iw_hub_mission_module
from robots.iw_hub.iw_hub_mission_agent import MissionIwHubAgent
from robots.p3020.p3020_mission_agent import (
    P3020PickPlaceAgent,
    P3020RosBridge,
    pixel_to_world_xy,
)


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


def _spawn_cargo_guards():
    if not CARGO_REGISTRY:
        return []

    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World/Cargo")
    spawned_paths = []

    for config in CARGO_REGISTRY:
        prim_path = f"/World/Cargo/{config['name']}"
        spawned_paths.append(
            spawn_cargo_guard_clone(
                stage,
                prim_path,
                spawn_xyz=config["spawn_xyz"],
                spawn_yaw=float(config.get("spawn_yaw", 0.0)),
                source_name=config["source_prim_name"],
                mass_kg=float(config.get("mass_kg", 20.0)),
            )
        )

    return spawned_paths


def _spawn_parcels(parcel_configs):
    if not parcel_configs:
        return

    stage = omni.usd.get_context().get_stage()
    # Keep parcel rigid bodies outside the cargo rigid body hierarchy, but group
    # them under /World/Cargo so Stage clearly shows they belong to this load.
    UsdGeom.Xform.Define(stage, "/World/Cargo/Parcels")

    for config in parcel_configs:
        prim_path = f"/World/Cargo/Parcels/{config['name']}"
        add_parcel_asset(
            stage,
            prim_path,
            asset_url=config["usd"],
            center=config["spawn_xyz"],
            max_size=config["max_size_xyz"],
            mass_kg=float(config.get("mass_kg", 15.0)),
        )


class AmrMissionBridge(Node):
    def __init__(self, agent):
        super().__init__("isaac_amr_mission_bridge")
        self.agent = agent
        self.last_state = None
        self.last_lift_state = None

        self.create_subscription(
            String,
            "/amr_a/pickup_command",
            self._command_callback,
            10,
        )
        self.create_subscription(
            String,
            "/amr_a/lift_command",
            self._lift_command_callback,
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            "/amr_a/pickup_state",
            10,
        )
        self.lift_state_pub = self.create_publisher(
            String,
            "/amr_a/lift_state",
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
        elif command == "RETURN_SPAWN":
            self.agent.request_return_spawn()
        elif command == "RESET":
            self.agent.reset_mission()
        else:
            self.get_logger().warning(
                f"unknown command: {command}"
            )

    def _lift_command_callback(self, message):
        action = message.data.strip().upper()

        accepted = self.agent.request_manual_lift(action)

        if accepted:
            self.get_logger().info(
                f"lift command accepted: {action}"
            )
        else:
            self.get_logger().warning(
                f"lift command rejected: {action}"
            )

    def _publish_state(self):
        state = self.agent.get_mission_state()
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

        if state != self.last_state:
            self.get_logger().info(f"pickup state: {state}")
            self.last_state = state

        lift_state = self.agent.get_manual_lift_state()
        lift_msg = String()
        lift_msg.data = lift_state
        self.lift_state_pub.publish(lift_msg)

        if lift_state != self.last_lift_state:
            self.get_logger().info(
                f"lift state: {lift_state}"
            )
            self.last_lift_state = lift_state


class OptimizedP3020PickPlaceAgent(P3020PickPlaceAgent):
    """Limit YOLO RGB publishing while keeping depth inside Isaac Sim."""

    def _wait_for_detection(self, ros_node, timeout_steps, tick_others, dt):
        not_before = ros_node.get_clock().now()
        depth_map = None
        last_frame = None

        for step in range(timeout_steps):
            if step % VISION_RGB_PUBLISH_INTERVAL_STEPS == 0:
                frame = self.camera.get_frame()
                if frame is not None:
                    last_frame = frame
                    ros_node.publish_image(frame)
                    depth_map = self.camera.get_depth()

            rclpy.spin_once(ros_node, timeout_sec=0.0)
            pixel = ros_node.take_pixel_after(not_before)

            if pixel is not None and depth_map is not None:
                world_xyz = pixel_to_world_xy(
                    pixel,
                    depth_map,
                    self.camera,
                    last_frame,
                )
                if world_xyz is not None:
                    return world_xyz

            if tick_others:
                tick_others(dt)
            self.world.step(render=True)

        return None


class OptimizedP3020RosBridge(P3020RosBridge):
    """Disable ROS2 depth transport; depth remains local to Isaac Sim."""

    def __init__(self):
        super().__init__()

        if self.depth_pub is not None:
            self.destroy_publisher(self.depth_pub)
            self.depth_pub = None

        self.get_logger().info(
            "vision optimization: /rgb ~=10 Hz, /depth ROS2 publisher disabled"
        )

    def publish_depth(self, depth_map):
        del depth_map


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
    cargo_paths = _spawn_cargo_guards()

    resolved_parcels = []
    if cargo_paths:
        iw_hub_mission_module.CARGO_PRIM_PATH = cargo_paths[0]

        cargo_xyz = CARGO_REGISTRY[0]["spawn_xyz"]
        resolved_parcels = resolve_parcel_layer(
            omni.usd.get_context().get_stage(),
            cargo_paths[0],
            PARCEL_REGISTRY,
            cargo_center_xy=(float(cargo_xyz[0]), float(cargo_xyz[1])),
        )
    _spawn_parcels(resolved_parcels)

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

    p3020_agent = OptimizedP3020PickPlaceAgent(world)
    p3020_agent.setup()

    world.reset()
    for agent in agents:
        agent.post_reset()
    p3020_agent.post_reset()

    world.play()

    for _ in range(30):
        world.step(render=True)

    rclpy.init(args=None)
    bridge = AmrMissionBridge(agents[0])
    p3020_bridge = OptimizedP3020RosBridge()

    print()
    print("============================================")
    print(" IW HUB CARGO + NAV2 + P3020 MISSION READY")
    print("============================================")
    print("[START] IW Hub: (10.5, 1.80122), yaw=0 deg")
    print("[LOCAL] rotate to +90 deg")
    print("[LOCAL] drive to: (10.5, -1.25), yaw=90 deg")
    print("[LOCAL] lift target: 0.04 m")
    print("[CARGO] cargo_box_gaurd_size_201: (10.5, -1.5, 0.5), yaw=0 deg")
    print("[CARGO] parcels: 4 boxes INSIDE guard, one 2x2 floor layer")
    print("[NAV2] starts only after PICKUP_DONE")
    print("[DELIVERY] (1.30104, -0.06065)")
    print("[RETURN] Nav2 -> cargo area -> local precision dock")
    print("[LOCAL] lift down at: (10.5, -1.25), yaw=90 deg")
    print("[LOCAL] return spawn: (10.5, 1.80122), yaw=0 deg")
    print("[ROS2] /amr_a/pickup_command")
    print("[ROS2] /amr_a/pickup_state")
    print("[ROS2] /amr_a/lift_command")
    print("[ROS2] /amr_a/lift_state")
    print("[PERF] YOLO RGB publish: every 6 simulation steps (~10 Hz)")
    print("[PERF] /depth ROS2 publishing: disabled (local depth kept)")
    print("============================================")

    def tick_iw_hub_agents(step_dt):
        for agent in agents:
            agent.on_physics_step(step_dt)

    try:
        while simulation_app.is_running():
            rclpy.spin_once(bridge, timeout_sec=0.0)
            rclpy.spin_once(p3020_bridge, timeout_sec=0.0)

            dt = float(world.get_physics_dt())
            tick_iw_hub_agents(dt)

            command = p3020_bridge.take_command()
            if command is not None:
                place_xy = (float(command["place_x"]), float(command["place_y"]))
                scan_hint = None
                if "scan_hint_x" in command and "scan_hint_y" in command:
                    scan_hint = (float(command["scan_hint_x"]), float(command["scan_hint_y"]))
                print(f"\n[P3020] pick_place command received: place={place_xy} scan_hint={scan_hint}")
                success, message = p3020_agent.run_pick_place(
                    p3020_bridge,
                    place_xy_world=place_xy,
                    scan_xy_world=scan_hint,
                    tick_others=tick_iw_hub_agents,
                    dt=dt,
                )
                print(f"[P3020] result: success={success} message={message}")

            world.step(render=True)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Ctrl+C received")

    finally:
        bridge.destroy_node()
        p3020_bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
