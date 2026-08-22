"""Simple scripted IW Hub cargo pickup + transport test.

No ROS2, Nav2, or LiDAR logic is used here.
The purpose is only to validate the physical sequence:
START -> cargo_pod -> lift up -> delivery coordinate.

The docking pose reuses the exact yaw=0 lift alignment that already passed the
standalone lift test.
"""

from pathlib import Path
import math
import sys

import numpy as np

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})


import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.viewports import set_camera_view

# Only wheel control is needed. ROS2 bridge is intentionally not enabled.
enable_extension("isaacsim.robot.wheeled_robots")
simulation_app.update()

from isaacsim.robot.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot


ISAAC_SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ISAAC_SIM_DIR))

from cargo.cargo_pod_physics import add_cargo_pod_physics
from project_config.robot_config import IW_HUB_USD, START_XY


WORLD_USD = (
    ISAAC_SIM_DIR
    / "usd"
    / "enva_small_warehouse_p3020_marker"
    / "World0.usd"
)
CARGO_USD = ISAAC_SIM_DIR / "usd" / "cargo" / "cargo_box.usd"

ROBOT_PRIM_PATH = "/World/Robots/amr_a"
SOURCE_ROBOT_PRIM = Sdf.Path("/World/iw_hub_ROS")
CARGO_PRIM_PATH = "/World/Cargo/cargo_pod"
COLLIDER_ROOT_PATH = f"{CARGO_PRIM_PATH}/PhysicsColliders"
LIFT_JOINT_PATH = f"{ROBOT_PRIM_PATH}/lift_joint"
LIFT_COLLISION_PATH = f"{ROBOT_PRIM_PATH}/lift/Collision"

WHEEL_DOF_NAMES = ["left_wheel_joint", "right_wheel_joint"]
WHEEL_RADIUS = 0.08
WHEEL_BASE = 0.58

# Cargo position.
POD_X = 10.5
POD_Y = -1.5
POD_Z = 0.5

# Requested destination.
DELIVERY_X = 1.30104
DELIVERY_Y = -0.06065

# This is the lift local X center measured from the current NVIDIA IW Hub.
# At yaw=0, placing the robot root at POD_X - (-0.255899...) centers the lift
# under the cargo. This exact pose already passed the standalone lift test.
LIFT_LOCAL_CENTER_X = -0.255899694280196
DOCK_ROOT_X = POD_X - LIFT_LOCAL_CENTER_X
DOCK_ROOT_Y = POD_Y
DOCK_YAW = 0.0

# Keep enough east-side clearance to rotate before reversing under the pod.
EAST_CLEAR_X = POD_X + 0.90

LIFT_DOWN = 0.0
LIFT_UP = 0.04
CARGO_MASS_KG = 20.0

EMPTY_LINEAR_SPEED = 0.25
DOCK_LINEAR_SPEED = 0.06
ROTATE_SPEED = 0.45
LOADED_LINEAR_SPEED = 0.16
LOADED_ANGULAR_SPEED = 0.20

POSITION_TOLERANCE = 0.025
DOCK_TOLERANCE = 0.010
YAW_TOLERANCE = math.radians(2.0)

START_CAMERA_EYE = (12.6, -3.9, 2.5)
START_CAMERA_TARGET = (10.6, -0.7, 0.25)


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(quaternion):
    w, x, y, z = [float(value) for value in quaternion]
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def _spawn_cargo(stage):
    UsdGeom.Xform.Define(stage, "/World/Cargo")

    prim = stage.DefinePrim(CARGO_PRIM_PATH, "Xform")
    prim.GetReferences().AddReference(str(CARGO_USD))

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(POD_X, POD_Y, POD_Z))

    add_cargo_pod_physics(
        stage,
        CARGO_PRIM_PATH,
        mass_kg=CARGO_MASS_KG,
    )

    # Debug request: show the compound physics boxes from the start.
    collider_root = stage.GetPrimAtPath(COLLIDER_ROOT_PATH)
    if collider_root.IsValid():
        for collider_prim in Usd.PrimRange(collider_root):
            if collider_prim.IsA(UsdGeom.Imageable):
                UsdGeom.Imageable(collider_prim).MakeVisible()

    print(f"[CARGO] spawned at ({POD_X:.3f}, {POD_Y:.3f}, {POD_Z:.3f})")
    print("[CARGO] PhysicsColliders are visible for this debug test")


def _spawn_iw_hub(stage):
    UsdGeom.Xform.Define(stage, "/World/Robots")

    prim = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
    prim.GetReferences().AddReference(IW_HUB_USD, SOURCE_ROBOT_PRIM)

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(
        Gf.Vec3d(float(START_XY[0]), float(START_XY[1]), 0.0)
    )
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    stage.Load(ROBOT_PRIM_PATH)

    # The navigation reference contains LiDAR prims. They are not needed in
    # this local physics test, so deactivate only LiDAR-named prims here.
    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    lidar_paths = []
    for child in Usd.PrimRange(robot_prim):
        path_text = child.GetPath().pathString.lower()
        if "lidar" in path_text:
            lidar_paths.append(child.GetPath())

    # Deactivate deepest paths first so traversal remains stable.
    for path in sorted(lidar_paths, key=lambda item: item.pathString.count("/"), reverse=True):
        lidar_prim = stage.GetPrimAtPath(path)
        if lidar_prim.IsValid() and lidar_prim.IsActive():
            try:
                lidar_prim.SetActive(False)
            except Exception:
                pass

    print(
        f"[IW HUB] spawned at START "
        f"({START_XY[0]:.3f}, {START_XY[1]:.3f}, 0.000)"
    )


def _set_lift_target(stage, target_position):
    joint = stage.GetPrimAtPath(LIFT_JOINT_PATH)
    if not joint.IsValid():
        raise RuntimeError(f"lift_joint not found: {LIFT_JOINT_PATH}")

    target_attr = joint.GetAttribute("drive:linear:physics:targetPosition")
    if not target_attr.IsValid():
        raise RuntimeError("lift_joint targetPosition attribute is missing")

    target_attr.Set(float(target_position))
    print(f"[LIFT] targetPosition = {target_position:.3f} m")


def _world_position(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path}")

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    matrix = cache.GetLocalToWorldTransform(prim)
    position = matrix.ExtractTranslation()
    return np.array(
        [float(position[0]), float(position[1]), float(position[2])],
        dtype=float,
    )


def _step_seconds(world, seconds):
    dt = float(world.get_physics_dt())
    for _ in range(max(1, int(round(seconds / dt)))):
        world.step(render=True)


def _stop(robot, controller):
    robot.apply_wheel_actions(
        controller.forward(np.array([0.0, 0.0], dtype=float))
    )


def _rotate_to(world, robot, controller, target_yaw, timeout=20.0):
    dt = float(world.get_physics_dt())
    max_steps = int(timeout / dt)

    for _ in range(max_steps):
        _, orientation = robot.get_world_pose()
        yaw = _yaw_from_quaternion(orientation)
        error = _wrap_angle(target_yaw - yaw)

        if abs(error) <= YAW_TOLERANCE:
            _stop(robot, controller)
            world.step(render=True)
            return True

        angular = float(np.clip(1.8 * error, -ROTATE_SPEED, ROTATE_SPEED))
        robot.apply_wheel_actions(
            controller.forward(np.array([0.0, angular], dtype=float))
        )
        world.step(render=True)

    _stop(robot, controller)
    return False


def _drive_x(world, robot, controller, target_x, speed, timeout=30.0):
    """Drive along world X. Robot must already be at yaw=0."""

    dt = float(world.get_physics_dt())
    max_steps = int(timeout / dt)

    for _ in range(max_steps):
        position, _ = robot.get_world_pose()
        error = float(target_x) - float(position[0])

        if abs(error) <= POSITION_TOLERANCE:
            _stop(robot, controller)
            world.step(render=True)
            return True

        linear = math.copysign(min(speed, max(0.035, 0.7 * abs(error))), error)
        robot.apply_wheel_actions(
            controller.forward(np.array([linear, 0.0], dtype=float))
        )
        world.step(render=True)

    _stop(robot, controller)
    return False


def _drive_y_south(world, robot, controller, target_y, speed, timeout=30.0):
    """Drive along world -Y. Robot must already be at yaw=-90 degrees."""

    dt = float(world.get_physics_dt())
    max_steps = int(timeout / dt)

    for _ in range(max_steps):
        position, _ = robot.get_world_pose()
        error = float(target_y) - float(position[1])

        if abs(error) <= POSITION_TOLERANCE:
            _stop(robot, controller)
            world.step(render=True)
            return True

        # At yaw=-90, positive linear velocity decreases world Y.
        linear = -math.copysign(min(speed, max(0.035, 0.7 * abs(error))), error)
        robot.apply_wheel_actions(
            controller.forward(np.array([linear, 0.0], dtype=float))
        )
        world.step(render=True)

    _stop(robot, controller)
    return False


def _reverse_into_pod(world, robot, controller, target_x, timeout=25.0):
    """At yaw=0, reverse west until the proven lift-alignment root pose."""

    dt = float(world.get_physics_dt())
    max_steps = int(timeout / dt)

    for _ in range(max_steps):
        position, _ = robot.get_world_pose()
        error = float(target_x) - float(position[0])

        if abs(error) <= DOCK_TOLERANCE:
            _stop(robot, controller)
            world.step(render=True)
            return True

        linear = math.copysign(
            min(DOCK_LINEAR_SPEED, max(0.025, 0.5 * abs(error))),
            error,
        )
        robot.apply_wheel_actions(
            controller.forward(np.array([linear, 0.0], dtype=float))
        )
        world.step(render=True)

    _stop(robot, controller)
    return False


def _drive_to_goal(
    world,
    robot,
    controller,
    goal_x,
    goal_y,
    timeout=140.0,
):
    """Simple point-to-point loaded drive; no obstacle avoidance."""

    dt = float(world.get_physics_dt())
    max_steps = int(timeout / dt)

    for _ in range(max_steps):
        position, orientation = robot.get_world_pose()
        x = float(position[0])
        y = float(position[1])
        yaw = _yaw_from_quaternion(orientation)

        dx = float(goal_x) - x
        dy = float(goal_y) - y
        distance = math.hypot(dx, dy)

        if distance <= 0.06:
            _stop(robot, controller)
            world.step(render=True)
            return True

        desired_yaw = math.atan2(dy, dx)
        heading_error = _wrap_angle(desired_yaw - yaw)

        linear = 0.0
        if abs(heading_error) < math.radians(10.0):
            linear = min(LOADED_LINEAR_SPEED, max(0.04, 0.4 * distance))

        angular = float(
            np.clip(1.4 * heading_error, -LOADED_ANGULAR_SPEED, LOADED_ANGULAR_SPEED)
        )

        robot.apply_wheel_actions(
            controller.forward(np.array([linear, angular], dtype=float))
        )
        world.step(render=True)

    _stop(robot, controller)
    return False


def _print_pose(label, robot):
    position, orientation = robot.get_world_pose()
    yaw = math.degrees(_yaw_from_quaternion(orientation))
    print(
        f"[{label}] x={position[0]:.3f}, y={position[1]:.3f}, "
        f"z={position[2]:.3f}, yaw={yaw:.1f} deg"
    )


def main():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(f"Warehouse USD not found: {WORLD_USD}")
    if not CARGO_USD.is_file():
        raise FileNotFoundError(f"Cargo USD not found: {CARGO_USD}")

    print()
    print("============================================")
    print(" SIMPLE CARGO PICKUP + TRANSPORT")
    print("============================================")
    print("[INFO] ROS2/Nav2/LiDAR logic is NOT used")
    print(f"[START] ({START_XY[0]:.3f}, {START_XY[1]:.3f})")
    print(f"[CARGO] ({POD_X:.3f}, {POD_Y:.3f})")
    print(f"[GOAL ] ({DELIVERY_X:.5f}, {DELIVERY_Y:.5f})")
    print("============================================")

    if open_stage(str(WORLD_USD)) is False:
        raise RuntimeError(f"Failed to open warehouse USD: {WORLD_USD}")

    for _ in range(5):
        simulation_app.update()

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    _spawn_cargo(stage)
    _spawn_iw_hub(stage)

    robot = world.scene.add(
        WheeledRobot(
            prim_path=ROBOT_PRIM_PATH,
            name="simple_transport_iw_hub",
            wheel_dof_names=WHEEL_DOF_NAMES,
            create_robot=False,
        )
    )

    controller = DifferentialController(
        name="simple_transport_controller",
        wheel_radius=WHEEL_RADIUS,
        wheel_base=WHEEL_BASE,
    )

    world.reset()
    world.play()

    set_camera_view(
        eye=START_CAMERA_EYE,
        target=START_CAMERA_TARGET,
        camera_prim_path="/OmniverseKit_Persp",
    )

    _set_lift_target(stage, LIFT_DOWN)
    print("[VIEW] cargo, visible colliders, and start area shown for 3 seconds")
    _step_seconds(world, 3.0)
    _print_pose("START", robot)

    # Keep the route simple and deterministic. We do not use a generic
    # pre-dock planner. The robot clears the pod on the east side, comes down
    # to the pod Y line, then reverses straight into the already-proven lift
    # pose. This avoids turning while under the pod.
    print()
    print("============================================")
    print(" 1. MOVE DIRECTLY TO CARGO DOCKING LINE")
    print("============================================")

    if not _drive_x(world, robot, controller, EAST_CLEAR_X, EMPTY_LINEAR_SPEED):
        raise RuntimeError("Failed to move east for cargo approach")

    if not _rotate_to(world, robot, controller, -math.pi / 2.0):
        raise RuntimeError("Failed to face cargo approach direction")

    if not _drive_y_south(world, robot, controller, POD_Y, EMPTY_LINEAR_SPEED):
        raise RuntimeError("Failed to reach cargo Y line")

    if not _rotate_to(world, robot, controller, DOCK_YAW):
        raise RuntimeError("Failed to align for reverse docking")

    print()
    print("============================================")
    print(" 2. REVERSE STRAIGHT UNDER CARGO")
    print("============================================")

    if not _reverse_into_pod(world, robot, controller, DOCK_ROOT_X):
        raise RuntimeError("Failed to reverse under cargo")

    _stop(robot, controller)
    _step_seconds(world, 1.0)
    _print_pose("DOCKED", robot)

    cargo_center = _world_position(stage, CARGO_PRIM_PATH)
    lift_center = _world_position(stage, LIFT_COLLISION_PATH)
    center_error = math.hypot(
        cargo_center[0] - lift_center[0],
        cargo_center[1] - lift_center[1],
    )

    print(
        f"[DOCK CHECK] cargo center x={cargo_center[0]:.4f}, y={cargo_center[1]:.4f}"
    )
    print(
        f"[DOCK CHECK] lift center  x={lift_center[0]:.4f}, y={lift_center[1]:.4f}"
    )
    print(f"[DOCK CHECK] center error = {center_error:.4f} m")

    print()
    print("============================================")
    print(" 3. LIFT UP")
    print("============================================")

    cargo_before = _world_position(stage, CARGO_PRIM_PATH)
    _set_lift_target(stage, LIFT_UP)
    _step_seconds(world, 4.0)
    cargo_after = _world_position(stage, CARGO_PRIM_PATH)
    lift_delta = cargo_after[2] - cargo_before[2]

    print(
        f"[CARGO] z: {cargo_before[2]:.4f} -> {cargo_after[2]:.4f}, "
        f"delta={lift_delta:.4f} m"
    )

    if lift_delta < 0.005:
        raise RuntimeError("Cargo was not lifted; transport cancelled")

    print("[PASS] Cargo lift confirmed")

    print()
    print("============================================")
    print(" 4. TRANSPORT TO DESTINATION")
    print("============================================")

    if not _drive_to_goal(
        world,
        robot,
        controller,
        DELIVERY_X,
        DELIVERY_Y,
    ):
        raise RuntimeError("Failed to reach delivery coordinate")

    _stop(robot, controller)
    _step_seconds(world, 2.0)
    _print_pose("GOAL", robot)

    cargo_goal = _world_position(stage, CARGO_PRIM_PATH)
    print(
        f"[CARGO] goal x={cargo_goal[0]:.3f}, "
        f"y={cargo_goal[1]:.3f}, z={cargo_goal[2]:.3f}"
    )

    print()
    print("============================================")
    print(" TEST COMPLETE")
    print("============================================")
    print("[PASS] START -> cargo -> lift -> destination completed")
    print("[INFO] Lift stays UP for inspection")
    print("[INFO] Ctrl+C closes Isaac Sim")
    print("============================================")

    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        print("\n[SYSTEM] Ctrl+C received")
    finally:
        _stop(robot, controller)
        world.stop()
        simulation_app.close()
        print("[SYSTEM] Simple cargo transport test closed")


if __name__ == "__main__":
    main()
