"""Custom warehouse + NVIDIA IW Hub + mission bridge.

Final order:
local start -> cargo -> lift -> Nav2 delivery -> P3020 action
-> Nav2 return -> local precision return -> lift down -> local spawn return.
"""

import json
import math
import random
import time
import uuid
from pathlib import Path

from isaacsim import SimulationApp

from project_config.simulation_config import (
    HEADLESS,
    LAUNCH_CONFIG,
    PHYSICS_DT,
    RENDERING_DT,
    apply_render_optimizations,
    disable_unused_cameras,
)

simulation_app = SimulationApp(LAUNCH_CONFIG)

import omni.graph.core as og
import omni.usd
import rclpy

from geometry_msgs.msg import PoseStamped
from pxr import Sdf, UsdGeom
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage

from project_config.robot_config import (
    PARCEL_REGISTRY,
    ROBOT_REGISTRY,
)


ISAAC_SIM_DIR = Path(__file__).resolve().parent
WORLD_USD = (
    ISAAC_SIM_DIR
    / "usd"
    / "Final_Real_Map"
    / "Parcel_Sorting_Map.usd"
)

VISION_RGB_PUBLISH_INTERVAL_STEPS = 6

# Physical sorter routing truth (see docs/sorter_merge/SORTER_MERGE_PLAN.md
# section 7) -- box_id, not the "destination" attribute, decides which
# ConveyorTrack a box gets diverted onto. box_id 4 passes every sorter.
BOX_ID_TO_TRACK = {1: "01", 2: "02", 3: "03", 4: None}

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
enable_extension("isaacsim.robot.wheeled_robots")
# Without this, the IsaacConveyor OmniGraph node type used by every
# ConveyorBeltGraph in the map is unrecognized ("Could not find node type
# interface for 'isaacsim.asset.gen.conveyor.IsaacConveyor'") -- Velocity
# still gets set on the graph, but the node type itself doesn't do
# anything, so the belt never actually moves.
enable_extension("isaacsim.asset.gen.conveyor")
simulation_app.update()

apply_render_optimizations()

from cargo.cargo_pod_physics import add_parcel_asset_scaled
from equipment.conveyor.conveyor_controller import ConveyorController
from equipment.wheel_sorter.wheel_sorter_controller import WheelSorterController
from robots.iw_hub.iw_hub_mission_agent import MissionIwHubAgent
from robots.p3020.p3020_mission_agent import (
    PARCEL_DESTINATION_ATTR,
    P3020PickPlaceAgent,
    P3020RosBridge,
    pixel_to_world_xy,
)
from robots.p3020.p3020_out_mission_agent import (
    P3020OutRosBridge,
    P3020UnloadToBinAgent,
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


def _spawn_parcels(parcel_configs):
    """Returns the spawned parcels' prim paths so the caller can keep
    tracking them (e.g. WheelSorterController.update_boxes()) -- the prim
    itself stays put even after P3020 picks it up, so the same path list is
    valid for the parcel's whole time on the map."""

    if not parcel_configs:
        return []

    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World/Cargo/Parcels")

    box_ids = random.sample([1, 2, 3, 4], len(parcel_configs))
    parcel_paths = []

    # A/B/C는 sorter 정상 구역, D는 어느 sorter 구역과도 안 맞아 컨베이어
    # 끝단(p3020_out 쪽 "배송지 오류" 구간)으로 흘러간다 -- wheel_sorter_
    # controller.py의 route_box() 참고. PARCEL_REGISTRY 쪽에서 "destination"을
    # 직접 지정하지 않은 박스는 이 네 값 중 하나로 랜덤 배정된다.
    DESTINATIONS = ("A", "B", "C", "D")

    for config, box_id in zip(parcel_configs, box_ids):
        prim_path = f"/World/Cargo/Parcels/{config['name']}"
        add_parcel_asset_scaled(
            stage,
            prim_path,
            asset_url=config["usd"],
            center=config["spawn_xyz"],
            scale_xyz=config["scale_xyz"],
            box_id=box_id,
            mass_kg=float(config.get("mass_kg", 15.0)),
        )
        parcel_prim = stage.GetPrimAtPath(prim_path)
        if "destination" in config:
            destination = str(config["destination"]).strip().upper()
        else:
            destination = random.choice(DESTINATIONS)
        parcel_prim.CreateAttribute(
            PARCEL_DESTINATION_ATTR, Sdf.ValueTypeNames.String
        ).Set(destination)
        print(f"[PARCEL] {config['name']} -> box_id={box_id}, destination={destination}")
        parcel_paths.append(prim_path)

    return parcel_paths


class AmrMissionBridge(Node):
    def __init__(self, agent):
        super().__init__("isaac_amr_mission_bridge")
        self.agent = agent
        self.last_state = None
        self.last_lift_state = None
        self.pose_source_session_id = (
            f"{time.time_ns() // 1_000_000}-"
            f"{uuid.uuid4().hex[:8]}"
        )

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
        self.create_subscription(
            PoseStamped,
            "/amr_a/restore_pose",
            self._restore_pose_callback,
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
        self.map_pose_pub = self.create_publisher(
            PoseStamped,
            "/amr_a/map_pose",
            10,
        )
        session_qos = QoSProfile(depth=1)
        session_qos.reliability = ReliabilityPolicy.RELIABLE
        session_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pose_source_session_pub = self.create_publisher(
            String,
            "/amr_a/pose_source_session",
            session_qos,
        )

        self.create_timer(0.2, self._publish_state)
        self.create_timer(0.2, self._publish_map_pose)
        self.create_timer(1.0, self._publish_pose_source_session)
        self._publish_pose_source_session()

    def _command_callback(self, message):
        command = message.data.strip().upper()

        if command == "PICKUP":
            self.agent.request_pickup()
        elif command == "CONVEYOR_DOCK":
            self.agent.request_conveyor_dock()
        elif command == "LOWER_AT_DELIVERY":
            self.agent.request_lower_at_delivery()
        elif command == "RAISE_AT_DELIVERY":
            self.agent.request_raise_at_delivery()
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

    def _restore_pose_callback(self, message):
        if message.header.frame_id != "map":
            self.get_logger().error(
                "restore pose rejected: "
                f"frame={message.header.frame_id!r}"
            )
            return

        if self.agent.get_mission_state() != "IDLE":
            self.get_logger().error(
                "restore pose rejected: mission is not IDLE"
            )
            return

        position = message.pose.position
        orientation = message.pose.orientation
        yaw = math.atan2(
            2.0
            * (
                float(orientation.w) * float(orientation.z)
                + float(orientation.x) * float(orientation.y)
            ),
            1.0
            - 2.0
            * (
                float(orientation.y) ** 2
                + float(orientation.z) ** 2
            ),
        )
        values = (float(position.x), float(position.y), float(yaw))

        if not all(math.isfinite(value) for value in values):
            self.get_logger().error(
                "restore pose rejected: non-finite value"
            )
            return

        try:
            self.agent.set_map_pose(*values)
        except Exception as error:
            self.get_logger().error(
                f"restore pose failed: {error}"
            )
            return

        self.get_logger().info(
            "restore pose applied: "
            f"x={values[0]:.3f} y={values[1]:.3f} "
            f"yaw={values[2]:.3f}"
        )

    def _publish_pose_source_session(self):
        message = String()
        message.data = self.pose_source_session_id
        self.pose_source_session_pub.publish(message)

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

    def _publish_map_pose(self):
        if self.agent.robot is None:
            return

        position, quaternion = self.agent.robot.get_world_pose()
        w, x, y, z = [float(value) for value in quaternion]

        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.map_pose_pub.publish(msg)


class OptimizedP3020PickPlaceAgent(P3020PickPlaceAgent):
    """Limit YOLO RGB publishing while keeping depth inside Isaac Sim."""

    def _wait_for_detection(self, ros_node, timeout_steps, tick_others, dt):
        not_before = ros_node.get_clock().now()
        ros_node._node.get_logger().info(
            f"[DIAG] _wait_for_detection start not_before={not_before.nanoseconds}"
        )
        depth_map = None
        last_frame = None

        for step in range(timeout_steps):
            if step % VISION_RGB_PUBLISH_INTERVAL_STEPS == 0:
                frame = self.camera.get_frame()
                if frame is not None:
                    last_frame = frame
                    ros_node.publish_image(frame)
                    depth_map = self.camera.get_depth()

            rclpy.spin_once(ros_node._node, timeout_sec=0.0)
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

    def __init__(self, node):
        super().__init__(node)

        if self.depth_pub is not None:
            self._node.destroy_publisher(self.depth_pub)
            self.depth_pub = None

        self._node.get_logger().info(
            "vision optimization: /rgb ~=10 Hz, /depth ROS2 publisher disabled"
        )

    def publish_depth(self, depth_map):
        del depth_map


class ProcessEquipmentBridge:
    """Expose P3020/conveyor/sorter control and process feedback to ROS2.

    Attaches to a caller-supplied node instead of being its own Node --
    Isaac Sim's bundled rclpy only bridges the first Node created after
    rclpy.init() to the outside world, so a second Node's subscriptions
    (like this one's /controltower/equipment/command) never receive
    anything even though discovery/matching looks fine."""

    def __init__(self, node, conveyor, sorter):
        self._node = node
        self.conveyor = conveyor
        self.sorter = sorter
        self.p3020_enabled = True
        self.p3020_out_enabled = True
        self.status_pub = node.create_publisher(
            String, "/controltower/equipment/status", 10
        )
        self.process_pub = node.create_publisher(
            String, "/controltower/process/event", 10
        )
        node.create_subscription(
            String,
            "/controltower/equipment/command",
            self._on_command,
            10,
        )
        node.create_timer(1.0, self._publish_all_status)
        node.create_timer(0.1, self._publish_sorter_events)

    def _on_command(self, message):
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as error:
            self._node.get_logger().error(f"bad equipment command: {error}")
            return

        code = str(payload.get("equipment_code", "")).strip().upper()
        action = str(payload.get("action", "")).strip().upper()
        enabled = action == "START"
        if action not in {"START", "STOP"}:
            return

        if code == "P3020_IN":
            self.p3020_enabled = enabled
        elif code == "P3020_OUT":
            self.p3020_out_enabled = enabled
        elif code == "MAIN_CONVEYOR":
            self.conveyor.start() if enabled else self.conveyor.stop()
        elif code.startswith("SORTER_"):
            self.sorter.set_region_enabled(code.removeprefix("SORTER_"), enabled)
        else:
            return
        self._publish_status(code)

    def _status_for(self, code):
        if code == "P3020_IN":
            return "RUNNING" if self.p3020_enabled else "STOPPED"
        if code == "P3020_OUT":
            return "RUNNING" if self.p3020_out_enabled else "STOPPED"
        if code == "MAIN_CONVEYOR":
            return self.conveyor.get_status()
        if code.startswith("SORTER_"):
            return self.sorter.get_region_status(code.removeprefix("SORTER_"))
        return "UNKNOWN"

    def _publish_status(self, code):
        message = String()
        message.data = json.dumps(
            {"equipment_code": code, "status": self._status_for(code)}
        )
        self.status_pub.publish(message)

    def _publish_all_status(self):
        for code in (
            "P3020_IN", "P3020_OUT", "MAIN_CONVEYOR",
            "SORTER_A", "SORTER_B", "SORTER_C",
        ):
            self._publish_status(code)

    def publish_conveyor_event(self, state):
        message = String()
        message.data = json.dumps(
            {
                "event_type": "CONVEYOR_STATE",
                "equipment_code": "MAIN_CONVEYOR",
                "state": state,
            }
        )
        self.process_pub.publish(message)

    def _publish_sorter_events(self):
        for event in self.sorter.take_process_events():
            state = event["state"]
            _, _, region = state.partition(":")
            equipment_region = region if region in {"A", "B", "C"} else "C"
            message = String()
            message.data = json.dumps(
                {
                    "event_type": "SORTER_STATE",
                    "equipment_code": f"SORTER_{equipment_region}",
                    "state": state,
                    "region": region,
                }
            )
            self.process_pub.publish(message)


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

    disable_unused_cameras(omni.usd.get_context().get_stage())

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=RENDERING_DT,
    )

    _create_clock_graph()

    conveyor = ConveyorController()
    sorter = WheelSorterController()
    conveyor.setup()
    sorter.setup()

    parcel_paths = _spawn_parcels(PARCEL_REGISTRY)

    # The reject bin used to be cloned here at REJECT_BIN_PRIM_PATH -- it's
    # now placed by hand directly in the map, so spawning a second one here
    # would just leave two overlapping bins in the scene.

    agents = []
    for config in ROBOT_REGISTRY:
        if config["type"] != "iw_hub":
            continue

        agent = MissionIwHubAgent(config, world)
        agent.setup()
        agents.append(agent)

    if not agents:
        raise RuntimeError("No IW Hub robot found")

    p3020_agent = OptimizedP3020PickPlaceAgent(world)
    p3020_agent.setup()

    p3020_out_agent = P3020UnloadToBinAgent(world)
    p3020_out_agent.setup()

    world.reset()
    for agent in agents:
        agent.post_reset()
    p3020_agent.post_reset()
    p3020_out_agent.post_reset()

    conveyor.setup()
    sorter.setup()

    world.play()
    conveyor.start()

    for _ in range(30):
        world.step(render=True)

    rclpy.init(args=None)
    # Every pub/sub below rides on this one node -- Isaac Sim's bundled
    # rclpy only bridges the first Node created after rclpy.init() to the
    # outside world, so separate Node objects for p3020/p3020_out/equipment
    # would silently stop receiving external messages (confirmed directly:
    # /arm_a/pick_place_command and /controltower/equipment/command never
    # arrived when p3020_bridge/equipment_bridge were their own Nodes).
    bridge = AmrMissionBridge(agents[0])
    p3020_bridge = OptimizedP3020RosBridge(bridge)
    p3020_out_bridge = P3020OutRosBridge(bridge)
    equipment_bridge = ProcessEquipmentBridge(bridge, conveyor, sorter)

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
    print("[ROS2] /amr_a/map_pose (frame=map, source=Isaac World)")
    print("[ROS2] /amr_a/pose_source_session")
    print("[ROS2] /amr_a/restore_pose")
    print("[P3020] one command = one Pick & Place cycle")
    print("[P3020_OUT] watches conveyor end-of-line, loads reject bin via OutboundLoadPlanner")
    print("[ROS2] /arm_b/rgb, /arm_b/box_pixel, /arm_b/pick_place_status")
    print("[CONTROL] /controltower/equipment/command + status/event feedback")
    print("[PERF] YOLO RGB publish: every 6 simulation steps (~10 Hz)")
    print("[PERF] /depth ROS2 publishing: disabled (local depth kept)")
    print("============================================")

    def tick_iw_hub_agents(step_dt):
        for agent in agents:
            agent.on_physics_step(step_dt)
        # P3020's pick/place loop blocks the outer while loop below by
        # stepping the world itself -- passing this same tick_others into
        # it (see the run_until_cargo_empty() call further down) is the
        # only way sorter routing keeps updating for boxes already on the
        # conveyor while P3020 is mid-cycle (see SORTER_MERGE_PLAN.md
        # section 9).
        sorter.update_boxes(parcel_paths, BOX_ID_TO_TRACK)

    try:
        while simulation_app.is_running():
            # bridge is now the single Node backing all four bridges above
            # (see the rclpy.init() comment). spin_once() only ever
            # executes one ready callback per call, and this one node now
            # carries every timer/subscription that used to be spread
            # across 4 separate Nodes -- looping a bounded number of times
            # drains the backlog each outer-loop pass instead of servicing
            # only one callback while the rest wait for the next pass.
            for _ in range(20):
                rclpy.spin_once(bridge, timeout_sec=0.0)

            dt = float(world.get_physics_dt())
            tick_iw_hub_agents(dt)

            command = p3020_bridge.take_command()
            if command is not None:
                if not equipment_bridge.p3020_enabled:
                    p3020_bridge.publish_status("DONE_FAIL:P3020_IN is STOPPED")
                    world.step(render=True)
                    continue
                place_xy = (
                    float(command["place_x"]),
                    float(command["place_y"]),
                )
                scan_hint = None
                if "scan_hint_x" in command and "scan_hint_y" in command:
                    scan_hint = (
                        float(command["scan_hint_x"]),
                        float(command["scan_hint_y"]),
                    )

                print(
                    "\n[P3020] pick_place command received: "
                    f"place={place_xy} scan_hint={scan_hint}"
                )

                # 액션 서버(pick_place_action_server.py)는 한 골(goal)을
                # "적재함이 빌 때까지"로 취급하고 CARGO_EMPTY가 와야만
                # 끝낸다 -- run_until_cargo_empty()가 박스를 하나씩 옮기다가
                # 기본 자세에서 NO_BOX_CONFIRM_TIMEOUT_S초 동안 더 이상
                # 안 보이면 CARGO_EMPTY를 찍고 AMR에 복귀 요청까지 보낸다.
                def _on_box_placed(message):
                    equipment_bridge.publish_conveyor_event("PACKAGE_ENTERED")
                    print(f"[P3020] result: success=True message={message}")

                p3020_agent.run_until_cargo_empty(
                    p3020_bridge,
                    place_xy_world=place_xy,
                    amr_agent=agents[0],
                    scan_xy_world=scan_hint,
                    tick_others=tick_iw_hub_agents,
                    dt=dt,
                    on_box_placed=_on_box_placed,
                )

            if equipment_bridge.p3020_out_enabled:
                out_success, out_message = p3020_out_agent.try_unload_cycle(
                    p3020_out_bridge,
                    tick_others=tick_iw_hub_agents,
                    dt=dt,
                    conveyor=conveyor,
                )
                if out_success:
                    print(f"[P3020_OUT] result: {out_message}")

            world.step(render=True)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Ctrl+C received")

    finally:
        # p3020_bridge/p3020_out_bridge/equipment_bridge are no longer
        # separate Nodes -- destroying bridge tears down their pub/sub too.
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
