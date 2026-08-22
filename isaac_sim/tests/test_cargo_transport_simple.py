"""Simple IW Hub Y-axis approach + lift test.

No ROS2, Nav2, or LiDAR logic is used here.
Sequence:
1) Spawn at the current project START position.
2) Rotate to +90 degrees.
3) Move only along the Y direction until y=-2.0846.
4) Stop and command lift_joint from 0.00 m to 0.04 m.

This is a temporary scripted test. Later the approach will be replaced by Nav2.
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

# Only local wheel control is needed. ROS2 bridge is intentionally not enabled.
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

WHEEL_DOF_NAMES = ["left_wheel_joint", "right_wheel_joint"]
WHEEL_RADIUS = 0.08
WHEEL_BASE = 0.58

# Existing cargo position.
POD_X = 10.5
POD_Y = -1.5
POD_Z = 0.5
CARGO_MASS_KG = 20.0

# Requested temporary target from the Isaac Sim screenshot.
# We intentionally control only Y after the 90-degree rotation.
TARGET_Y = -2.0846
TARGET_YAW = math.radians(90.0)
PHOTO_X_REFERENCE = 10.53654

LIFT_DOWN = 0.0
LIFT_UP = 0.04

ROTATE_SPEED = 0.35
Y_DRIVE_SPEED = 0.12
YAW_TOLERANCE = math.radians(1.5)
Y_TOLERANCE = 0.015

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

    # This temporary test does not use LiDAR. Deactivate LiDAR-named prims only.
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
    steps = max(1, int(round(float(seconds) / dt)))
    for _ in range(steps):
        world.step(render=True)


def _stop(robot, controller):
    robot.apply_wheel_actions(
        controller.forward(np.array([0.0, 0.0], dtype=float))
    )


def _rotate_to_90(world, robot, controller, timeout=20.0):
    dt = float(world.get_physics_dt())
    max_steps = int(timeout / dt)

    for _ in range(max_steps):
        _, orientation = robot.get_world_pose()
        yaw = _yaw_from_quaternion(orientation)
        error = _wrap_angle(TARGET_YAW - yaw)

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


def _drive_y_only(world, robot, controller, timeout=40.0):
    """After yaw=+90 deg, move only by forward/backward wheel motion on Y."""

    dt = float(world.get_physics_dt())
    max_steps = int(timeout / dt)

    for _ in range(max_steps):
        position, _ = robot.get_world_pose()
        error_y = TARGET_Y - float(position[1])

        if abs(error_y) <= Y_TOLERANCE:
            _stop(robot, controller)
            world.step(render=True)
            return True

        # At +90 deg, positive linear velocity increases world Y.
        # Our target is lower Y, so the command will naturally become negative.
        linear = math.copysign(
            min(Y_DRIVE_SPEED, max(0.03, 0.45 * abs(error_y))),
            error_y,
        )

        robot.apply_wheel_actions(
            controller.forward(np.array([linear, 0.0], dtype=float))
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
    print(" IW HUB Y-AXIS APPROACH + LIFT TEST")
    print("============================================")
    print("[INFO] ROS2/Nav2/LiDAR logic is NOT used")
    print(f"[START] x={START_XY[0]:.5f}, y={START_XY[1]:.5f}")
    print("[STEP 1] rotate to +90 deg")
    print(f"[STEP 2] move only on Y to y={TARGET_Y:.4f}")
    print(f"[PHOTO] reference x={PHOTO_X_REFERENCE:.5f}")
    print("[STEP 3] lift up to 0.04 m")
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
            name="y_axis_lift_iw_hub",
            wheel_dof_names=WHEEL_DOF_NAMES,
            create_robot=False,
        )
    )

    controller = DifferentialController(
        name="y_axis_lift_controller",
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
    if not _rotate_to_90(world, robot, controller):
        raise RuntimeError("Failed to rotate IW Hub to +90 degrees")
    _print_pose("ROTATED", robot)

    print()
    print("============================================")
    print(" 2. MOVE ONLY ALONG Y")
    print("============================================")
    if not _drive_y_only(world, robot, controller):
        raise RuntimeError(f"Failed to reach target Y={TARGET_Y:.4f}")

    _stop(robot, controller)
    _step_seconds(world, 1.0)
    _print_pose("Y TARGET", robot)

    print()
    print("============================================")
    print(" 3. LIFT UP")
    print("============================================")

    cargo_before = _world_position(stage, CARGO_PRIM_PATH)
    _set_lift_target(stage, LIFT_UP)
    _step_seconds(world, 4.0)
    cargo_after = _world_position(stage, CARGO_PRIM_PATH)

    cargo_delta_z = float(cargo_after[2] - cargo_before[2])
    print(
        f"[CARGO] z={cargo_before[2]:.4f} -> {cargo_after[2]:.4f}, "
        f"delta={cargo_delta_z:.4f} m"
    )

    if cargo_delta_z >= 0.005:
        print("[PASS] cargo_pod moved upward with the lift")
    else:
        print("[WARN] lift moved up, but cargo_pod did not rise enough")
        print("       We can tune the final Y alignment after this test")

    print()
    print("============================================")
    print(" TEST COMPLETE")
    print("============================================")
    print("[INFO] Lift stays UP for visual inspection")
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
        print("[SYSTEM] Y-axis lift test closed")


if __name__ == "__main__":
    main()
