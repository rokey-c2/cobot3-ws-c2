"""IW Hub dock + articulation lift + delivery test.

No ROS2/Nav2/LiDAR logic is used.
The lift is controlled through the articulation API instead of editing the
USD drive target directly. This avoids the issue where the authored target
changed but the live PhysX articulation did not move after wheel control.
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
from isaacsim.core.utils.types import ArticulationAction
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
LIFT_COLLISION_PATH = f"{ROBOT_PRIM_PATH}/lift/Collision"
LIFT_JOINT_PATH = f"{ROBOT_PRIM_PATH}/lift_joint"

WHEEL_DOF_NAMES = ["left_wheel_joint", "right_wheel_joint"]
WHEEL_RADIUS = 0.08
WHEEL_BASE = 0.58

POD_X = 10.5
POD_Y = -1.5
POD_Z = 0.5
CARGO_MASS_KG = 20.0

TEST_START_X = POD_X
TEST_START_Y = float(START_XY[1])
TARGET_YAW = math.radians(90.0)

LIFT_LOCAL_CENTER_X = -0.255899694280196
TARGET_ROOT_Y = POD_Y - LIFT_LOCAL_CENTER_X

DELIVERY_X = 1.30104
DELIVERY_Y = -0.06065

ROTATE_SPEED = 0.35
Y_DRIVE_SPEED = 0.08
LOADED_LINEAR_SPEED = 0.07
LOADED_ANGULAR_SPEED = 0.08

YAW_TOLERANCE = math.radians(0.7)
Y_TOLERANCE = 0.005
MAX_DOCK_CENTER_ERROR = 0.025
DELIVERY_TOLERANCE = 0.08
MIN_LIFT_JOINT_MOTION = 0.030
MIN_CARGO_LIFT = 0.005

CAMERA_EYE = (12.3, -3.5, 2.6)
CAMERA_TARGET = (10.5, -1.3, 0.25)


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(q):
    w, x, y, z = [float(v) for v in q]
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _spawn_cargo(stage):
    UsdGeom.Xform.Define(stage, "/World/Cargo")
    prim = stage.DefinePrim(CARGO_PRIM_PATH, "Xform")
    prim.GetReferences().AddReference(str(CARGO_USD))

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(POD_X, POD_Y, POD_Z))

    add_cargo_pod_physics(stage, CARGO_PRIM_PATH, mass_kg=CARGO_MASS_KG)
    print(f"[CARGO] spawned at x={POD_X:.5f}, y={POD_Y:.5f}, z={POD_Z:.3f}")
    print("[CARGO] RigidBody + compound PhysicsColliders enabled")


def _spawn_iw_hub(stage):
    UsdGeom.Xform.Define(stage, "/World/Robots")
    prim = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
    prim.GetReferences().AddReference(IW_HUB_USD, SOURCE_ROBOT_PRIM)

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(TEST_START_X, TEST_START_Y, 0.0))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    stage.Load(ROBOT_PRIM_PATH)

    # This local physics test does not use LiDAR.
    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    lidar_paths = [
        p.GetPath()
        for p in Usd.PrimRange(robot_prim)
        if "lidar" in p.GetPath().pathString.lower()
    ]
    for path in sorted(lidar_paths, key=lambda p: p.pathString.count("/"), reverse=True):
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid() and prim.IsActive():
            try:
                prim.SetActive(False)
            except Exception:
                pass

    print(f"[IW HUB] test start x={TEST_START_X:.5f}, y={TEST_START_Y:.5f}")


def _world_position(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    p = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array([float(p[0]), float(p[1]), float(p[2])])


def _lift_upper_limit(stage):
    prim = stage.GetPrimAtPath(LIFT_JOINT_PATH)
    return float(prim.GetAttribute("physics:upperLimit").Get())


def _step_seconds(world, seconds):
    dt = float(world.get_physics_dt())
    for _ in range(max(1, int(round(seconds / dt)))):
        world.step(render=True)


def _stop(robot, controller):
    robot.apply_wheel_actions(
        controller.forward(np.array([0.0, 0.0], dtype=float))
    )


def _hold_lift(robot, lift_index, target):
    robot.apply_action(
        ArticulationAction(
            joint_positions=np.array([float(target)], dtype=float),
            joint_indices=np.array([int(lift_index)], dtype=np.int32),
        )
    )


def _move_lift(world, robot, lift_index, target, seconds=5.0):
    dt = float(world.get_physics_dt())
    for _ in range(max(1, int(round(seconds / dt)))):
        _hold_lift(robot, lift_index, target)
        world.step(render=True)


def _rotate_to(world, robot, controller, target_yaw, timeout=25.0):
    dt = float(world.get_physics_dt())
    for _ in range(int(timeout / dt)):
        _, q = robot.get_world_pose()
        error = _wrap_angle(target_yaw - _yaw_from_quaternion(q))
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


def _drive_y_to_dock(world, robot, controller, timeout=40.0):
    dt = float(world.get_physics_dt())
    for _ in range(int(timeout / dt)):
        p, q = robot.get_world_pose()
        error_y = TARGET_ROOT_Y - float(p[1])
        yaw_error = _wrap_angle(TARGET_YAW - _yaw_from_quaternion(q))

        if abs(error_y) <= Y_TOLERANCE:
            _stop(robot, controller)
            world.step(render=True)
            return True

        linear = math.copysign(
            min(Y_DRIVE_SPEED, max(0.02, 0.40 * abs(error_y))),
            error_y,
        )
        angular = float(np.clip(1.3 * yaw_error, -0.08, 0.08))
        robot.apply_wheel_actions(
            controller.forward(np.array([linear, angular], dtype=float))
        )
        world.step(render=True)

    _stop(robot, controller)
    return False


def _drive_to_delivery(world, robot, controller, lift_index, lift_target, timeout=220.0):
    """Slow direct drive while continuously holding the lift at max."""

    dt = float(world.get_physics_dt())
    for _ in range(int(timeout / dt)):
        p, q = robot.get_world_pose()
        x = float(p[0])
        y = float(p[1])
        yaw = _yaw_from_quaternion(q)

        dx = DELIVERY_X - x
        dy = DELIVERY_Y - y
        distance = math.hypot(dx, dy)

        if distance <= DELIVERY_TOLERANCE:
            _stop(robot, controller)
            _hold_lift(robot, lift_index, lift_target)
            world.step(render=True)
            return True

        desired_yaw = math.atan2(dy, dx)
        heading_error = _wrap_angle(desired_yaw - yaw)

        linear = 0.0
        if abs(heading_error) <= math.radians(6.0):
            linear = min(
                LOADED_LINEAR_SPEED,
                max(0.025, 0.20 * distance),
            )

        angular = float(np.clip(
            1.0 * heading_error,
            -LOADED_ANGULAR_SPEED,
            LOADED_ANGULAR_SPEED,
        ))

        robot.apply_wheel_actions(
            controller.forward(np.array([linear, angular], dtype=float))
        )
        _hold_lift(robot, lift_index, lift_target)
        world.step(render=True)

    _stop(robot, controller)
    return False


def _print_pose(label, robot):
    p, q = robot.get_world_pose()
    yaw = math.degrees(_yaw_from_quaternion(q))
    print(
        f"[{label}] x={p[0]:.5f}, y={p[1]:.5f}, "
        f"z={p[2]:.5f}, yaw={yaw:.2f} deg"
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
            name="cargo_delivery_v3_iw_hub",
            wheel_dof_names=WHEEL_DOF_NAMES,
            create_robot=False,
        )
    )
    controller = DifferentialController(
        name="cargo_delivery_v3_controller",
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

    lift_index = int(robot.get_dof_index("lift_joint"))
    lift_max = _lift_upper_limit(stage)

    print()
    print("============================================")
    print(" IW HUB ARTICULATION LIFT + DELIVERY V3")
    print("============================================")
    print(f"[POD] x={POD_X:.5f}, y={POD_Y:.5f}")
    print(f"[DOCK ROOT Y] y={TARGET_ROOT_Y:.5f}")
    print(f"[LIFT DOF INDEX] {lift_index}")
    print(f"[LIFT MAX] {lift_max:.8f} m")
    print(f"[DELIVERY] x={DELIVERY_X:.5f}, y={DELIVERY_Y:.5f}")
    print("[INFO] ROS2/Nav2/LiDAR are NOT used")
    print("============================================")

    _move_lift(world, robot, lift_index, 0.0, seconds=1.0)
    _print_pose("START", robot)

    print("\n[STEP 1] rotate to +90 deg")
    if not _rotate_to(world, robot, controller, TARGET_YAW):
        raise RuntimeError("Failed to rotate to +90 degrees")

    print("\n[STEP 2] move along Y under cargo_pod")
    if not _drive_y_to_dock(world, robot, controller):
        raise RuntimeError("Failed to reach docking pose")

    _stop(robot, controller)
    _step_seconds(world, 1.0)
    _print_pose("DOCK ROOT", robot)

    cargo_center = _world_position(stage, CARGO_PRIM_PATH)
    lift_center = _world_position(stage, LIFT_COLLISION_PATH)
    center_error = math.hypot(
        float(lift_center[0] - cargo_center[0]),
        float(lift_center[1] - cargo_center[1]),
    )
    print(f"[DOCK CHECK] center error={center_error:.5f} m")
    if center_error > MAX_DOCK_CENTER_ERROR:
        raise RuntimeError("Lift center alignment failed")

    print("\n[STEP 3] articulation position-control lift to maximum")
    joint_before = float(robot.get_joint_positions(
        joint_indices=np.array([lift_index], dtype=np.int32)
    )[0])
    cargo_before = _world_position(stage, CARGO_PRIM_PATH)

    print(f"[LIFT JOINT BEFORE] {joint_before:.6f} m")
    print(f"[CARGO BEFORE] z={cargo_before[2]:.6f}")

    _move_lift(world, robot, lift_index, lift_max, seconds=5.0)

    joint_after = float(robot.get_joint_positions(
        joint_indices=np.array([lift_index], dtype=np.int32)
    )[0])
    cargo_after = _world_position(stage, CARGO_PRIM_PATH)

    joint_delta = joint_after - joint_before
    cargo_delta = float(cargo_after[2] - cargo_before[2])

    print(f"[LIFT JOINT AFTER ] {joint_after:.6f} m")
    print(f"[LIFT JOINT DELTA ] {joint_delta:.6f} m")
    print(f"[CARGO AFTER] z={cargo_after[2]:.6f}")
    print(f"[CARGO DELTA] z={cargo_delta:.6f} m")

    if joint_delta < MIN_LIFT_JOINT_MOTION:
        raise RuntimeError("Physical lift joint still did not move")
    print("[PASS] physical lift joint moved")

    if cargo_delta < MIN_CARGO_LIFT:
        raise RuntimeError("Lift moved, but cargo_pod did not rise")
    print("[PASS] cargo_pod lifted")

    print("\n[STEP 4] carry lifted cargo to requested destination")
    print(f"[TARGET] x={DELIVERY_X:.5f}, y={DELIVERY_Y:.5f}")

    if not _drive_to_delivery(
        world,
        robot,
        controller,
        lift_index,
        lift_max,
    ):
        raise RuntimeError("Failed to reach delivery coordinate")

    _stop(robot, controller)
    _hold_lift(robot, lift_index, lift_max)
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
    print("[PASS] dock -> lift -> delivery completed")
    print("[INFO] Lift remains at physical maximum")
    print("[INFO] Ctrl+C closes Isaac Sim")
    print("============================================")

    try:
        while simulation_app.is_running():
            _hold_lift(robot, lift_index, lift_max)
            world.step(render=True)
    except KeyboardInterrupt:
        print("\n[SYSTEM] Ctrl+C received")
    finally:
        _stop(robot, controller)
        world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
