"""Temporary IW Hub pickup + transport test without ROS2/Nav2/LiDAR.

Sequence:
1) Spawn IW Hub at the current project START position.
2) Rotate to +90 degrees.
3) Move only along Y until the lift center is under cargo_pod.
4) Raise lift_joint to its authored maximum upper limit.
5) Carry cargo_pod to the requested destination coordinate.

Later, only the driving portion will be replaced by Nav2/LiDAR.
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

# Local wheel control only. ROS2 bridge is intentionally not enabled.
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
LIFT_JOINT_PATH = f"{ROBOT_PRIM_PATH}/lift_joint"
LIFT_COLLISION_PATH = f"{ROBOT_PRIM_PATH}/lift/Collision"

WHEEL_DOF_NAMES = ["left_wheel_joint", "right_wheel_joint"]
WHEEL_RADIUS = 0.08
WHEEL_BASE = 0.58

# cargo_pod position.
POD_X = 10.5
POD_Y = -1.5
POD_Z = 0.5
CARGO_MASS_KG = 20.0

# Measured from NVIDIA IW Hub.
# lift center is robot-local X = -0.255899694... m.
# At yaw=+90 deg this becomes world Y = -0.255899694... m.
LIFT_LOCAL_CENTER_X = -0.255899694280196
TARGET_YAW = math.radians(90.0)
DOCK_ROOT_Y = POD_Y - LIFT_LOCAL_CENTER_X

# User-requested destination coordinate.
DELIVERY_X = 1.30104
DELIVERY_Y = -0.06065

LIFT_DOWN = 0.0

ROTATE_SPEED = 0.35
DOCK_SPEED = 0.10
LOADED_LINEAR_SPEED = 0.10
LOADED_ANGULAR_SPEED = 0.12

YAW_TOLERANCE = math.radians(1.5)
DOCK_Y_TOLERANCE = 0.012
DELIVERY_TOLERANCE = 0.08
MAX_DOCK_CENTER_ERROR = 0.10

CAMERA_EYE = (12.4, -3.8, 2.6)
CAMERA_TARGET = (10.5, -1.2, 0.25)


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

    print(f"[CARGO] spawned at ({POD_X:.3f}, {POD_Y:.3f}, {POD_Z:.3f})")
    print("[CARGO] PhysicsColliders are visible")


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

    # This temporary test does not use LiDAR.
    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    lidar_paths = []
    for child in Usd.PrimRange(robot_prim):
        if "lidar" in child.GetPath().pathString.lower():
            lidar_paths.append(child.GetPath())

    for path in sorted(
        lidar_paths,
        key=lambda item: item.pathString.count("/"),
        reverse=True,
    ):
        lidar_prim = stage.GetPrimAtPath(path)
        if lidar_prim.IsValid() and lidar_prim.IsActive():
            try:
                lidar_prim.SetActive(False)
            except Exception:
                pass

    print(
        f"[IW HUB] spawned at START "
        f"({START_XY[0]:.5f}, {START_XY[1]:.5f}, 0.00000)"
    )


def _lift_upper_limit(stage):
    joint = stage.GetPrimAtPath(LIFT_JOINT_PATH)
    if not joint.IsValid():
        raise RuntimeError(f"lift_joint not found: {LIFT_JOINT_PATH}")

    upper_attr = joint.GetAttribute("physics:upperLimit")
    if not upper_attr.IsValid():
        raise RuntimeError("lift_joint upperLimit attribute is missing")

    return float(upper_attr.Get())


def _set_lift_target(stage, target_position):
    joint = stage.GetPrimAtPath(LIFT_JOINT_PATH)
    if not joint.IsValid():
        raise RuntimeError(f"lift_joint not found: {LIFT_JOINT_PATH}")

    target_attr = joint.GetAttribute("drive:linear:physics:targetPosition")
    if not target_attr.IsValid():
        raise RuntimeError("lift_joint targetPosition attribute is missing")

    target_attr.Set(float(target_position))
    print(f"[LIFT] targetPosition = {target_position:.5f} m")


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
    steps = max(1, int(round(float(seconds) / dt)))
    for _ in range(steps):
        world.step(render=True)


def _stop(robot, controller):
    robot.apply_wheel_actions(
        controller.forward(np.array([0.0, 0.0], dtype=float))
    )


def _rotate_to(world, robot, controller, target_yaw, timeout=25.0):
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


def _drive_y_to_dock(world, robot, controller, timeout=45.0):
    """At yaw=+90 deg, move only along Y to the lift-aligned root Y."""

    dt = float(world.get_physics_dt())
    max_steps = int(timeout / dt)

    for _ in range(max_steps):
        position, _ = robot.get_world_pose()
        error_y = DOCK_ROOT_Y - float(position[1])

        if abs(error_y) <= DOCK_Y_TOLERANCE:
            _stop(robot, controller)
            world.step(render=True)
            return True

        linear = math.copysign(
            min(DOCK_SPEED, max(0.025, 0.45 * abs(error_y))),
            error_y,
        )

        robot.apply_wheel_actions(
            controller.forward(np.array([linear, 0.0], dtype=float))
        )
        world.step(render=True)

    _stop(robot, controller)
    return False


def _drive_to_delivery(world, robot, controller, timeout=180.0):
    """Slow direct point drive for this temporary non-Nav2 test."""

    dt = float(world.get_physics_dt())
    max_steps = int(timeout / dt)

    for _ in range(max_steps):
        position, orientation = robot.get_world_pose()
        x = float(position[0])
        y = float(position[1])
        yaw = _yaw_from_quaternion(orientation)

        dx = DELIVERY_X - x
        dy = DELIVERY_Y - y
        distance = math.hypot(dx, dy)

        if distance <= DELIVERY_TOLERANCE:
            _stop(robot, controller)
            world.step(render=True)
            return True

        desired_yaw = math.atan2(dy, dx)
        heading_error = _wrap_angle(desired_yaw - yaw)

        # Rotate gently first. Move forward only when mostly aligned.
        linear = 0.0
        if abs(heading_error) <= math.radians(8.0):
            linear = min(
                LOADED_LINEAR_SPEED,
                max(0.035, 0.30 * distance),
            )

        angular = float(
            np.clip(
                1.2 * heading_error,
                -LOADED_ANGULAR_SPEED,
                LOADED_ANGULAR_SPEED,
            )
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
        f"[{label}] x={position[0]:.5f}, y={position[1]:.5f}, "
        f"z={position[2]:.5f}, yaw={yaw:.2f} deg"
    )


def main():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(f"Warehouse USD not found: {WORLD_USD}")
    if not CARGO_USD.is_file():
        raise FileNotFoundError(f"Cargo USD not found: {CARGO_USD}")

    print()
    print("============================================")
    print(" IW HUB PICKUP + TRANSPORT TEST")
    print("============================================")
    print("[INFO] ROS2/Nav2/LiDAR logic is NOT used")
    print(f"[START] x={START_XY[0]:.5f}, y={START_XY[1]:.5f}")
    print(f"[CARGO] x={POD_X:.5f}, y={POD_Y:.5f}")
    print(f"[DOCK ROOT Y] {DOCK_ROOT_Y:.5f}")
    print(f"[DELIVERY] x={DELIVERY_X:.5f}, y={DELIVERY_Y:.5f}")
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
            name="cargo_transport_iw_hub",
            wheel_dof_names=WHEEL_DOF_NAMES,
            create_robot=False,
        )
    )

    controller = DifferentialController(
        name="cargo_transport_controller",
        wheel_radius=WHEEL_RADIUS,
        wheel_base=WHEEL_BASE,
    )

    world.reset()
    world.play()

    set_camera_view(
        eye=CAMERA_EYE,
        target=CAMERA_TARGET,
        camera_prim_path="/OmniverseKit_Persp",
    )

    _set_lift_target(stage, LIFT_DOWN)
    _step_seconds(world, 2.0)
    _print_pose("START", robot)

    print()
    print("============================================")
    print(" 1. ROTATE TO +90 DEG")
    print("============================================")
    if not _rotate_to(world, robot, controller, TARGET_YAW):
        raise RuntimeError("Failed to rotate IW Hub to +90 degrees")
    _print_pose("ROTATED", robot)

    print()
    print("============================================")
    print(" 2. MOVE ONLY ALONG Y UNDER CARGO")
    print("============================================")
    if not _drive_y_to_dock(world, robot, controller):
        raise RuntimeError(f"Failed to reach dock root Y={DOCK_ROOT_Y:.5f}")

    _stop(robot, controller)
    _step_seconds(world, 1.0)
    _print_pose("DOCKED ROOT", robot)

    cargo_center = _world_position(stage, CARGO_PRIM_PATH)
    lift_center = _world_position(stage, LIFT_COLLISION_PATH)
    center_error = math.hypot(
        float(cargo_center[0] - lift_center[0]),
        float(cargo_center[1] - lift_center[1]),
    )

    print(
        f"[CARGO CENTER] x={cargo_center[0]:.5f}, y={cargo_center[1]:.5f}"
    )
    print(
        f"[LIFT CENTER ] x={lift_center[0]:.5f}, y={lift_center[1]:.5f}"
    )
    print(f"[CENTER ERROR] {center_error:.5f} m")

    if center_error > MAX_DOCK_CENTER_ERROR:
        raise RuntimeError(
            "IW Hub lift is not sufficiently under cargo_pod; lift cancelled"
        )

    print()
    print("============================================")
    print(" 3. LIFT UP TO PHYSICAL MAXIMUM")
    print("============================================")

    lift_max = _lift_upper_limit(stage)
    print(f"[LIFT] authored upperLimit = {lift_max:.8f} m")

    cargo_before = _world_position(stage, CARGO_PRIM_PATH)
    _set_lift_target(stage, lift_max)
    _step_seconds(world, 5.0)
    cargo_after = _world_position(stage, CARGO_PRIM_PATH)

    cargo_delta_z = float(cargo_after[2] - cargo_before[2])
    print(
        f"[CARGO] z={cargo_before[2]:.4f} -> {cargo_after[2]:.4f}, "
        f"delta={cargo_delta_z:.4f} m"
    )

    if cargo_delta_z < 0.005:
        raise RuntimeError("cargo_pod did not lift; delivery cancelled")

    print("[PASS] cargo_pod lifted successfully")

    print()
    print("============================================")
    print(" 4. MOVE TO REQUESTED DESTINATION")
    print("============================================")
    print(f"[TARGET] x={DELIVERY_X:.5f}, y={DELIVERY_Y:.5f}")

    if not _drive_to_delivery(world, robot, controller):
        raise RuntimeError("Failed to reach requested destination")

    _stop(robot, controller)
    _step_seconds(world, 2.0)
    _print_pose("DELIVERY", robot)

    cargo_goal = _world_position(stage, CARGO_PRIM_PATH)
    print(
        f"[CARGO FINAL] x={cargo_goal[0]:.5f}, "
        f"y={cargo_goal[1]:.5f}, z={cargo_goal[2]:.5f}"
    )

    print()
    print("============================================")
    print(" TEST COMPLETE")
    print("============================================")
    print("[PASS] START -> cargo -> MAX LIFT -> destination")
    print("[INFO] Lift remains at the maximum upper limit")
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
        print("[SYSTEM] Cargo transport test closed")


if __name__ == "__main__":
    main()
