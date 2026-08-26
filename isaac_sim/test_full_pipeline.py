"""Full-pipeline test: AMR delivers cargo -> P3020 arm #1 picks each box by
VISION (not scripted coordinates) and places it on the conveyor -> wheel
sorter routes it by its "destination" attribute.

No external YOLO detector is running, so this script stands one in: it knows
each spawned parcel's real 3D position (since it spawned them) and forward-
projects that position through the P3020 camera's real intrinsics/pose
(CameraInterface.world_to_pixel) to publish a believable /box_pixel message,
exactly the message shape a real detector would send. Everything downstream
of that (depth read, pixel->world back-projection, IK, gripper, placement,
sorter routing) is the real, unmodified pipeline.

TEMP TEST placeholder coordinates -- see test_new_logic.py and the
TEMP TEST VALUE comments in project_config/robot_config.py /
robots/iw_hub/iw_hub_mission_agent.py. Map final layout still WIP.
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from isaacsim import SimulationApp

from project_config.simulation_config import HEADLESS

simulation_app = SimulationApp({"headless": HEADLESS})

import omni.usd
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from pxr import Sdf, UsdGeom

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage

from project_config.robot_config import (
    PARCEL_REGISTRY,
    ROBOT_REGISTRY,
)

ISAAC_SIM_DIR = Path(__file__).resolve().parent
WORLD_USD = ISAAC_SIM_DIR / "usd" / "Parcel_Sorting_Map" / "Parcel_Sorting_Map.usd"

# TEMP TEST VALUE -- must stay within P3020 arm #1's 2.0 m reach of its real
# measured base (0.2, -1.5, 0.4) and land on the plain conveyor segment
# ("/World/ConveyorTrack", x=-2..0, y=-0.575..0.575) that feeds sorter A.
CONVEYOR_PLACE_XY = (-0.5, 0.0)

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
enable_extension("isaacsim.robot.wheeled_robots")
simulation_app.update()

from cargo.cargo_pod_physics import add_parcel_asset_scaled
import robots.iw_hub.iw_hub_mission_agent as iw_hub_mission_module
from robots.iw_hub.iw_hub_mission_agent import MissionIwHubAgent
from robots.p3020.p3020_mission_agent import P3020PickPlaceAgent, P3020RosBridge
from equipment.conveyor.conveyor_controller import ConveyorController
from equipment.wheel_sorter.wheel_sorter_controller import WheelSorterController


class FakeBoxDetectorNode(Node):
    """Stands in for the external YOLO detector: publishes /box_pixel for
    whichever real parcel prim is still under PARCEL_PARENT_PATH, using the
    P3020 camera's own forward projection so the pixel is exactly where that
    box really renders (not a guess)."""

    # A box already placed on the conveyor is still a child of parent_path
    # (nothing reparents/removes it) but has physically moved far from the
    # cargo pod by then -- only consider boxes still within this radius of
    # the pod so a just-placed, belt-moving box isn't mistaken for the next
    # pick.
    STILL_IN_POD_RADIUS = 0.6

    def __init__(self, camera, stage, cargo_prim_path, parent_path="/World/Cargo/Parcels"):
        super().__init__("fake_box_detector")
        self.camera = camera
        self.stage = stage
        self.cargo_prim_path = cargo_prim_path
        self.parent_path = parent_path
        self.pub = self.create_publisher(PointStamped, "/box_pixel", 10)

    def _cargo_pod_position(self):
        cargo_prim = self.stage.GetPrimAtPath(self.cargo_prim_path)
        if not cargo_prim.IsValid():
            return None
        xf = UsdGeom.Xformable(cargo_prim).ComputeLocalToWorldTransform(0)
        return xf.Transform((0, 0, 0))

    def publish_once(self):
        parent = self.stage.GetPrimAtPath(self.parent_path)
        if not parent.IsValid():
            return
        children = list(parent.GetChildren())
        if not children:
            return

        pod_pos = self._cargo_pod_position()
        if pod_pos is None:
            return

        nearest_prim = None
        nearest_dist = None
        for child in children:
            xf = UsdGeom.Xformable(child).ComputeLocalToWorldTransform(0)
            p = xf.Transform((0, 0, 0))
            dist = ((p[0] - pod_pos[0]) ** 2 + (p[1] - pod_pos[1]) ** 2) ** 0.5
            if dist <= self.STILL_IN_POD_RADIUS and (nearest_dist is None or dist < nearest_dist):
                nearest_dist = dist
                nearest_prim = child

        if nearest_prim is None:
            return  # every remaining child has already left the pod (on the belt)

        prim = nearest_prim
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        pos = xf.Transform((0, 0, 0))
        px, py = self.camera.world_to_pixel((pos[0], pos[1], pos[2]))

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "p3020_rsd455"
        msg.point.x = px
        msg.point.y = py
        msg.point.z = 1.0  # confidence
        self.pub.publish(msg)


def spawn_cargo_and_parcels(stage):
    """Cargo pod is baked into the map now (not code-spawned) -- this only
    spawns the 4 parcels onto it, mirroring main_mission.py's
    _spawn_parcels(). iw_hub_mission_module.CARGO_PRIM_PATH already
    defaults to the real baked-in pod's path, so it doesn't need overriding
    here the way it did when cargo was cloned fresh per run."""

    UsdGeom.Xform.Define(stage, "/World/Cargo/Parcels")
    destinations = ["A", "B", "C", "D"]
    box_ids = random.sample([1, 2, 3, 4], len(PARCEL_REGISTRY))

    for i, (parcel_cfg, box_id) in enumerate(zip(PARCEL_REGISTRY, box_ids)):
        parcel_path = f"/World/Cargo/Parcels/{parcel_cfg['name']}"
        add_parcel_asset_scaled(
            stage,
            parcel_path,
            asset_url=parcel_cfg["usd"],
            center=parcel_cfg["spawn_xyz"],
            scale_xyz=parcel_cfg["scale_xyz"],
            box_id=box_id,
            mass_kg=float(parcel_cfg.get("mass_kg", 15.0)),
        )
        dest = destinations[i % len(destinations)]
        stage.GetPrimAtPath(parcel_path).CreateAttribute(
            "destination", Sdf.ValueTypeNames.String
        ).Set(dest)
        print(
            f"[TEST] parcel {parcel_cfg['name']} -> "
            f"box_id={box_id}, destination={dest}"
        )

    return iw_hub_mission_module.CARGO_PRIM_PATH


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

    spawn_cargo_and_parcels(stage)

    agent = MissionIwHubAgent(ROBOT_REGISTRY[0], world)
    agent.setup()

    p3020_agent = P3020PickPlaceAgent(world)
    p3020_agent.setup()

    world.reset()
    agent.post_reset()
    p3020_agent.post_reset()

    conveyor.setup()
    sorter.setup()
    world.play()
    conveyor.start()

    for _ in range(30):
        world.step(render=True)

    dt = float(world.get_physics_dt())

    rclpy.init(args=None)
    p3020_bridge = P3020RosBridge()
    fake_detector = FakeBoxDetectorNode(
        p3020_agent.camera, stage, iw_hub_mission_module.CARGO_PRIM_PATH
    )

    def tick_all(step_dt):
        agent.on_physics_step(step_dt)
        fake_detector.publish_once()

    print("\n[TEST] Phase 1: AMR docks at cargo, lifts")
    agent.request_pickup()
    max_steps = int(30.0 / dt)
    for _ in range(max_steps):
        tick_all(dt)
        world.step(render=True)
        if agent.mission_state in {"PICKUP_DONE", "ERROR"}:
            break
    print(f"[TEST] AMR state: {agent.mission_state}")

    if agent.mission_state == "PICKUP_DONE":
        pos, orient = agent.robot.get_world_pose()
        teleport_pos = np.array([1.1, -1.5, pos[2]])
        delta = teleport_pos - np.asarray(pos)
        agent.robot.set_world_pose(position=teleport_pos, orientation=orient)

        # The cargo pod + parcels are a separate rigid body resting on the
        # lift, not a USD child of the robot -- teleporting the AMR alone
        # leaves them behind (real driving would carry them along through
        # physics). Move them by the same delta so the arm's camera finds
        # them where the AMR actually ended up.
        cargo_prim = stage.GetPrimAtPath(iw_hub_mission_module.CARGO_PRIM_PATH)
        for prim in [cargo_prim] + list(
            stage.GetPrimAtPath("/World/Cargo/Parcels").GetChildren()
        ):
            xformable = UsdGeom.Xformable(prim)
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    current = op.Get()
                    op.Set(type(current)(current[0] + delta[0], current[1] + delta[1], current[2] + delta[2]))
                    break

        world.step(render=True)
        print(f"[TEST] teleported AMR (and cargo+parcels) to simulate Nav2 arrival: {teleport_pos}")

        agent.request_conveyor_dock()
        tick_all(dt)
        world.step(render=True)
        print(f"[TEST] AMR state: {agent.mission_state}")

    print("\n[TEST] Phase 2: P3020 arm #1 picks every box by vision and "
          "sorts it (fake detector standing in for YOLO)")
    p3020_agent.run_until_cargo_empty(
        p3020_bridge,
        place_xy_world=CONVEYOR_PLACE_XY,
        amr_agent=agent,
        tick_others=tick_all,
        dt=dt,
        sorter=sorter,
    )
    print(f"[TEST] AMR state after unload: {agent.mission_state}")

    print("\n[TEST] done. Letting the conveyor run a bit longer so sorted "
          "boxes are visible reaching their region.")
    for _ in range(600):
        tick_all(dt)
        world.step(render=True)

    p3020_bridge.destroy_node()
    fake_detector.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

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
