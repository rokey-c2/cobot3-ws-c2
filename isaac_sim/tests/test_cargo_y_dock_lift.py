"""Temporary IW Hub cargo pickup + delivery test.

No ROS2, Nav2, or LiDAR logic is used.
Sequence:
1) Start on the current inbound lane.
2) Rotate to +90 degrees.
3) Move only along Y until the lift center is under cargo_pod.
4) Raise lift_joint to its authored physical maximum.
5) Carry cargo_pod to the requested destination coordinate.

Later the driving logic can be replaced by Nav2 while keeping the same
physical cargo and lift behavior.
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

# cargo_pod world pose
POD_X = 10.5
POD_Y = -1.5
POD_Z = 0.5
CARGO_MASS_KG = 20.0

# Keep the straight docking lane seen in the user's screenshot.
TEST_START_X = 10.53654
TEST_START_Y = float(START_XY[1])
TARGET_YAW = math.radians(90.0)

# NVIDIA IW Hub lift center measured from the robot root.
# local lift center X = -0.2558996943 m.
# At yaw +90 deg this becomes world Y = -0.2558996943 m.
# Therefore robot root Y must be POD_Y + 0.2558996943.
LIFT_LOCAL_CENTER_X = -0.255899694280196
TARGET_ROOT_Y = POD_Y - LIFT_LOCAL_CENTER_X

# User-requested destination.
DELIVERY_X = 1.30104
DELIVERY_Y = -0.06065

LIFT_DOWN = 0.0

ROTATE_SPEED = 0.35
Y_DRIVE_SPEED = 0.10
LOADED_LINEAR_SPEED = 0.10
LOADED_ANGULAR_SPEED = 0.12

YAW_TOLERANCE = math.radians(1.0)
Y_TOLERANCE = 0.008
MAX_DOCK_CENTER_ERROR = 0.08
DELIVERY_TOLERANCE = 0.08

CAMERA_EYE = (12.3, -3.5, 2.5)
CAMERA_TARGET = (10.5, -1.3, 0.25)


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

    add_cargo_pod_physics(stage, CARGO_PRIM_PATH, mass_kg=CARGO_MASS_KG)
    print(f"[CARGO] spawned at x={POD_X:.5f}, y={POD_Y:.5f}, z={POD_Z:.3f}")
    print("[CARGO] PhysicsColliders are visible")


def _spawn_iw_hub(stage):
    UsdGeom.Xform.Define(stage, "/World/Robots")
    prim = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
    prim.GetReferences().AddReference(IW_HUB_USD, SOURCE_ROBOT_PRIM)

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(TEST_START_X, TEST_START_Y, 0.0))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    stage.Load(ROBOT_PRIM_PATH)

    # This local test does not use LiDAR.
    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    lidar_paths = []
    for child in Usd.PrimRange(robot_prim):
        if "lidar" in child.GetPath().pathString.lower():
            lidar_paths.append(child.GetPath())

    for path in sorted(lidar_paths, key=lambda p: p.pathString.count("/"), reverse=True):
        lidar_prim = stage.GetPrimAtPath(path)
        if lidar_prim.IsValid() and lidar_prim.IsActive():
            try:
                lidar_prim.SetActive(False)
            except Exception:
                pass

    print(f"[IW HUB] test start x={TEST_START_X:.5f}, y={TEST_START_Y:.5f}")


def _lift_upper_limit(stage):
    joint = stage.GetPrimAtPath(LIFT_JOINT_PATH)
    if not joint.IsValid():
        raise RuntimeError(f"lift_joint not found: {LIFT_JOINT_PATH}")

    attr = joint.GetAttribute("physics:upperLimit")
    if not attr.IsValid():
        raise RuntimeError("lift_joint upperLimit attribute is missing")

    return float(attr.Get())


def _set_lift_target(stage, target_position):
    joint = stage.GetPrimAtPath(LIFT_JOINT_PATH)
    if not joint.IsValid():
        raise RuntimeError(f"lift_joint not found: {LIFT_JOINT_PATH}")

    attr = joint.GetAttribute("drive:linear:physics:targetPosition")
    if not attr.IsValid():
        raise RuntimeError("lift_joint targetPosition attribute is missing")

    attr.Set(float(target_position))
    print(f"[LIFT] targetPosition = {target_position:.8f} m")


def _world_position(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path}")

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    pos = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array([float(pos[0]), float(pos[1]), float(pos[2])], dtype=float)


def _step_seconds(world, seconds):
    dt = float(world.get_physics_dt())
    for _ in range(max(1, int(round(seconds / dt)))):
        world.step(render=True)


def _stop(robot, controller):
    robot.apply_wheel_actions(controller.forward(np.array([0.0, 0.0], dtype=float)))


def _rotate_to(world, robot, controller, target_yaw, timeout=25.0):
    dt = float(world.get_physics_dt())

    for _ in range(int(timeout / dt)):
        _, orientation = robot.get_world_pose()
        yaw = _yaw_from_quaternion(orientation)
        error = _wrap_angle(target_yaw - yaw)

        if abs(error) <= YAW_TOLERANCE:
            _stop(robot, controller)
            world.step(render=True)
            return True

        angular = float(np.clip(1.8 * error, -ROTATE_SPEED, ROTATE_SPEED))
        robot.apply_wheel_actions(controller.forward(np.array([0.0, angular], dtype=float)))
        world.step(render=True)

    _stop(robot, controller)
    return False


def _drive_y_to_dock(world, robot, controller, timeout=35.0):
    """Move only along the Y docking lane while holding yaw near +90 deg."""

    dt = float(world.get_physics_dt())

    for _ in range(int(timeout / dt)):
        position, orientation = robot.get_world_pose()
        error_y = TARGET_ROOT_Y - float(position[1])
        yaw = _yaw_from_quaternion(orientation)
        yaw_error = _wrap_angle(TARGET_YAW - yaw)

        if abs(error_y) <= Y_TOLERANCE:
            _stop(robot, controller)
            world.step(render=True)
            return True

        linear = math.copysign(
            min(Y_DRIVE_SPEED, max(0.025, 0.45 * abs(error_y))),
            error_y,
        )
        angular = float(np.clip(1.2 * yaw_error, -0.10, 0.10))

        robot.apply_wheel_actions(
            controller.forward(np.array([linear, angular], dtype=float))
        )
        world.step(render=True)

    _stop(robot, controller)
    return False


def _drive_to_delivery(world, robot, controller, timeout=180.0):
    """Slow direct drive to the requested point. No obstacle avoidance."""

    dt = float(world.get_physics_dt())

    for _ in range(int(timeout / dt)):
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

        # With cargo lifted, rotate and drive slowly to reduce slipping.
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
            name="cargo_y_dock_iw_hub",
            wheel_dof_names=WHEEL_DOF_NAMES,
            create_robot=False,
        )
    )

    controller = DifferentialController(
        name="cargo_y_dock_controller",
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

    print()
    print("============================================")
    print(" IW HUB PICKUP + DELIVERY TEST")
    print("============================================")
    print(f"[POD] x={POD_X:.5f}, y={POD_Y:.5f}")
    print(f"[ROBOT X LANE] x={TEST_START_X:.5f}")
    print(f"[DOCK ROOT Y] y={TARGET_ROOT_Y:.5f}")
    print(f"[DELIVERY] x={DELIVERY_X:.5f}, y={DELIVERY_Y:.5f}")
    print("[INFO] ROS2/Nav2/LiDAR are NOT used")
    print("============================================")

    _set_lift_target(stage, LIFT_DOWN)
    _step_seconds(world, 2.0)
    _print_pose("START", robot)

    print("\n[STEP 1] rotate to +90 deg")
    if not _rotate_to(world, robot, controller, TARGET_YAW):
        raise RuntimeError("Failed to rotate to +90 degrees")
    _print_pose("ROTATED", robot)

    print("\n[STEP 2] move only along Y under cargo_pod")
    if not _drive_y_to_dock(world, robot, controller):
        raise RuntimeError(f"Failed to reach dock root y={TARGET_ROOT_Y:.5f}")

    _stop(robot, controller)
    _step_seconds(world, 1.0)
    _print_pose("DOCK ROOT", robot)

    cargo_center = _world_position(stage, CARGO_PRIM_PATH)
    lift_center = _world_position(stage, LIFT_COLLISION_PATH)
    dx = float(lift_center[0] - cargo_center[0])
    dy = float(lift_center[1] - cargo_center[1])
    center_error = math.hypot(dx, dy)

    print("\n============================================")
    print(" DOCK CHECK")
    print("============================================")
    print(f"[CARGO CENTER] x={cargo_center[0]:.5f}, y={cargo_center[1]:.5f}")
    print(f"[LIFT CENTER ] x={lift_center[0]:.5f}, y={lift_center[1]:.5f}")
    print(f"[CENTER ERROR] dx={dx:.5f}, dy={dy:.5f}, total={center_error:.5f} m")

    if center_error > MAX_DOCK_CENTER_ERROR:
        raise RuntimeError("Lift center is not sufficiently under cargo; lift cancelled")

    print("\n[STEP 3] lift to physical maximum")
    lift_max = _lift_upper_limit(stage)
    print(f"[LIFT] authored upperLimit = {lift_max:.8f} m")

    cargo_before = _world_position(stage, CARGO_PRIM_PATH)
    _set_lift_target(stage, lift_max)
    _step_seconds(world, 5.0)
    cargo_after = _world_position(stage, CARGO_PRIM_PATH)
    cargo_delta_z = float(cargo_after[2] - cargo_before[2])

    print(f"[CARGO] z={cargo_before[2]:.4f} -> {cargo_after[2]:.4f}")
    print(f"[CARGO] delta Z={cargo_delta_z:.4f} m")

    if cargo_delta_z < 0.005:
        raise RuntimeError("cargo_pod did not lift; delivery cancelled")

    print("[PASS] cargo_pod lifted successfully")

    print("\n[STEP 4] carry cargo to requested destination")
    print(f"[TARGET] x={DELIVERY_X:.5f}, y={DELIVERY_Y:.5f}")

    if not _drive_to_delivery(world, robot, controller):
        raise RuntimeError("Failed to reach requested destination")

    _stop(robot, controller)
    _step_seconds(world, 2.0)
    _print_pose("DELIVERY", robot)

    cargo_final = _world_position(stage, CARGO_PRIM_PATH)
    print(
        f"[CARGO FINAL] x={cargo_final[0]:.5f}, "
        f"y={cargo_final[1]:.5f}, z={cargo_final[2]:.5f}"
    )

    print("\n============================================")
    print(" TEST COMPLETE")
    print("============================================")
    print("[PASS] dock -> maximum lift -> destination completed")
    print("[INFO] Lift stays at the physical maximum for inspection")
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


if __name__ == "__main__":
    main()
