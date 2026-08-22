"""Scripted IW Hub cargo pickup + transport test without ROS2/Nav2.

Temporary test flow:
1. Spawn the IW Hub at the project's current START_XY.
2. Drive to the east side of cargo_pod.
3. Back slowly under the pod using the same final pose that already passed
   the standalone lift test.
4. Raise the existing NVIDIA lift_joint from 0.00 m to 0.04 m.
5. If the pod was actually lifted, drive it to the selected destination.
6. Stop with the lift kept UP for visual inspection.

This file is only for the current PC test. Later the travel parts can be
replaced by Nav2 while the docking/lift sequence is kept.
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

# No ROS2 bridge and no Nav2 in this test.
enable_extension("isaacsim.robot.wheeled_robots")
simulation_app.update()

from isaacsim.robot.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot


ISAAC_SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ISAAC_SIM_DIR))

from cargo.cargo_pod_physics import add_cargo_pod_physics
from project_config.robot_config import IW_HUB_USD, ROBOT_REGISTRY, START_XY


WORLD_USD = (
    ISAAC_SIM_DIR
    / "usd"
    / "enva_small_warehouse_p3020_marker"
    / "World0.usd"
)
CARGO_USD = ISAAC_SIM_DIR / "usd" / "cargo" / "cargo_box.usd"

ROBOT_PRIM_PATH = "/World/Robots/amr_a"
SOURCE_ROBOT_PRIM = Sdf.Path("/World/iw_hub_ROS")
LIFT_JOINT_PATH = f"{ROBOT_PRIM_PATH}/lift_joint"
LIFT_COLLISION_PATH = f"{ROBOT_PRIM_PATH}/lift/Collision"
CARGO_PRIM_PATH = "/World/Cargo/cargo_pod"

WHEEL_DOF_NAMES = ["left_wheel_joint", "right_wheel_joint"]
WHEEL_RADIUS = 0.08
WHEEL_BASE = 0.58

# cargo_pod position.
POD_X = 10.5
POD_Y = -1.5
POD_Z = 0.5

# Destination selected from the Isaac Sim Transform panel.
DELIVERY_X = 1.30104
DELIVERY_Y = -0.06065

# Measured from NVIDIA IW Hub Navigation robot.
# Lift plate center is about 0.2559 m behind the robot root on local X.
LIFT_LOCAL_CENTER_X = -0.255899694280196

# IMPORTANT:
# The standalone lift test already succeeded with yaw=0 and the robot root
# shifted +0.2559 m on world X. Reuse that exact geometry here instead of
# trying a different -90 degree docking pose.
DOCK_YAW = 0.0
DOCK_ROOT_X = POD_X - LIFT_LOCAL_CENTER_X
DOCK_ROOT_Y = POD_Y

# Approach from the east side, then reverse slowly into the pod.
PRE_DOCK_X = POD_X + 1.40
PRE_DOCK_Y = POD_Y

LIFT_DOWN = 0.0
LIFT_UP = 0.04
CARGO_MASS_KG = 20.0

# Slow speeds because this test has no obstacle avoidance.
EMPTY_LINEAR_SPEED = 0.30
EMPTY_ANGULAR_SPEED = 0.55
DOCK_LINEAR_SPEED = 0.035
DOCK_ANGULAR_SPEED = 0.10
LOADED_LINEAR_SPEED = 0.16
LOADED_ANGULAR_SPEED = 0.14

POSITION_TOLERANCE = 0.05
DOCK_POSITION_TOLERANCE = 0.008
YAW_TOLERANCE = math.radians(2.0)

# Start with both the cargo pod and the nearby robot visible.
START_CAMERA_EYE = (12.6, -3.9, 2.3)
START_CAMERA_TARGET = (10.6, -0.8, 0.35)


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(quaternion):
    """Convert Isaac quaternion [w, x, y, z] to planar yaw."""

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

    print(
        f"[CARGO] spawned at "
        f"({POD_X:.3f}, {POD_Y:.3f}, {POD_Z:.3f})"
    )


def _spawn_iw_hub(stage):
    UsdGeom.Xform.Define(stage, "/World/Robots")

    prim = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
    prim.GetReferences().AddReference(
        IW_HUB_USD,
        SOURCE_ROBOT_PRIM,
    )

    start_yaw_degrees = 0.0
    if ROBOT_REGISTRY:
        start_yaw_degrees = float(
            ROBOT_REGISTRY[0].get("spawn_yaw", 0.0)
        )

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(
        Gf.Vec3d(float(START_XY[0]), float(START_XY[1]), 0.0)
    )
    xform.AddRotateXYZOp().Set(
        Gf.Vec3f(0.0, 0.0, start_yaw_degrees)
    )

    stage.Load(ROBOT_PRIM_PATH)

    print(
        f"[IW HUB] spawned at START "
        f"({START_XY[0]:.3f}, {START_XY[1]:.3f}, 0.000)"
    )


def _disable_lidar_prims(stage):
    """Disable lidar prims only inside this temporary scripted test."""

    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        return

    paths = []
    for prim in Usd.PrimRange(robot_prim):
        path_text = prim.GetPath().pathString.lower()
        name_text = prim.GetName().lower()
        if "lidar" in path_text or "lidar" in name_text:
            paths.append(prim.GetPath())

    # Deactivate shallowest matching roots only. Descendants are disabled too.
    selected = []
    for path in sorted(paths, key=lambda item: item.pathString.count("/")):
        if any(path.HasPrefix(parent) for parent in selected):
            continue
        selected.append(path)

    for path in selected:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid() and prim.IsActive():
            try:
                prim.SetActive(False)
            except Exception:
                pass

    print(f"[TEST] disabled lidar-related prim roots: {len(selected)}")


def _set_lift_target(stage, target_position):
    joint = stage.GetPrimAtPath(LIFT_JOINT_PATH)
    if not joint.IsValid():
        raise RuntimeError(f"lift_joint not found: {LIFT_JOINT_PATH}")

    target_attr = joint.GetAttribute(
        "drive:linear:physics:targetPosition"
    )
    if not target_attr.IsValid():
        raise RuntimeError(
            "lift_joint has no drive:linear:physics:targetPosition"
        )

    lower = joint.GetAttribute("physics:lowerLimit").Get()
    upper = joint.GetAttribute("physics:upperLimit").Get()

    if lower is not None and target_position < float(lower):
        raise ValueError(
            f"Lift target {target_position} is below lower limit {lower}"
        )
    if upper is not None and target_position > float(upper) + 1e-6:
        raise ValueError(
            f"Lift target {target_position} is above upper limit {upper}"
        )

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
    physics_dt = float(world.get_physics_dt())
    steps = max(1, int(round(float(seconds) / physics_dt)))

    for _ in range(steps):
        world.step(render=True)


def _stop_robot(robot, controller):
    robot.apply_wheel_actions(
        controller.forward(np.array([0.0, 0.0], dtype=float))
    )


def _rotate_to(
    world,
    robot,
    controller,
    target_yaw,
    max_angular_speed,
    timeout_seconds=25.0,
):
    physics_dt = float(world.get_physics_dt())
    max_steps = int(timeout_seconds / physics_dt)

    for _ in range(max_steps):
        _, orientation = robot.get_world_pose()
        current_yaw = _yaw_from_quaternion(orientation)
        error = _wrap_angle(target_yaw - current_yaw)

        if abs(error) <= YAW_TOLERANCE:
            _stop_robot(robot, controller)
            world.step(render=True)
            return True

        angular = float(
            np.clip(1.5 * error, -max_angular_speed, max_angular_speed)
        )
        robot.apply_wheel_actions(
            controller.forward(np.array([0.0, angular], dtype=float))
        )
        world.step(render=True)

    _stop_robot(robot, controller)
    return False


def _drive_to(
    world,
    robot,
    controller,
    goal_x,
    goal_y,
    max_linear_speed,
    max_angular_speed,
    position_tolerance,
    timeout_seconds=120.0,
):
    """Simple forward point-to-point driving."""

    physics_dt = float(world.get_physics_dt())
    max_steps = int(timeout_seconds / physics_dt)

    for _ in range(max_steps):
        position, orientation = robot.get_world_pose()
        x = float(position[0])
        y = float(position[1])
        yaw = _yaw_from_quaternion(orientation)

        dx = float(goal_x) - x
        dy = float(goal_y) - y
        distance = math.hypot(dx, dy)

        if distance <= position_tolerance:
            _stop_robot(robot, controller)
            world.step(render=True)
            return True

        desired_yaw = math.atan2(dy, dx)
        heading_error = _wrap_angle(desired_yaw - yaw)

        if abs(heading_error) > math.radians(12.0):
            linear = 0.0
        else:
            linear = min(max_linear_speed, max(0.04, 0.7 * distance))

        angular = float(
            np.clip(
                1.5 * heading_error,
                -max_angular_speed,
                max_angular_speed,
            )
        )

        robot.apply_wheel_actions(
            controller.forward(np.array([linear, angular], dtype=float))
        )
        world.step(render=True)

    _stop_robot(robot, controller)
    return False


def _reverse_to(
    world,
    robot,
    controller,
    goal_x,
    goal_y,
    max_linear_speed,
    max_angular_speed,
    position_tolerance,
    timeout_seconds=45.0,
):
    """Back slowly toward a target while steering the robot rear-first."""

    physics_dt = float(world.get_physics_dt())
    max_steps = int(timeout_seconds / physics_dt)

    for _ in range(max_steps):
        position, orientation = robot.get_world_pose()
        x = float(position[0])
        y = float(position[1])
        yaw = _yaw_from_quaternion(orientation)

        dx = float(goal_x) - x
        dy = float(goal_y) - y
        distance = math.hypot(dx, dy)

        if distance <= position_tolerance:
            _stop_robot(robot, controller)
            world.step(render=True)
            return True

        # When reversing, the robot's rear direction should point at the goal.
        desired_body_yaw = _wrap_angle(math.atan2(dy, dx) + math.pi)
        heading_error = _wrap_angle(desired_body_yaw - yaw)

        if abs(heading_error) > math.radians(10.0):
            linear = 0.0
        else:
            linear = -min(
                max_linear_speed,
                max(0.012, 0.45 * distance),
            )

        angular = float(
            np.clip(
                1.4 * heading_error,
                -max_angular_speed,
                max_angular_speed,
            )
        )

        robot.apply_wheel_actions(
            controller.forward(np.array([linear, angular], dtype=float))
        )
        world.step(render=True)

    _stop_robot(robot, controller)
    return False


def _print_pose(label, robot):
    position, orientation = robot.get_world_pose()
    yaw = math.degrees(_yaw_from_quaternion(orientation))
    print(
        f"[{label}] "
        f"x={position[0]:.3f}, y={position[1]:.3f}, "
        f"z={position[2]:.3f}, yaw={yaw:.1f} deg"
    )


def _print_dock_check(stage):
    cargo = _world_position(stage, CARGO_PRIM_PATH)
    lift = _world_position(stage, LIFT_COLLISION_PATH)
    error_xy = math.hypot(cargo[0] - lift[0], cargo[1] - lift[1])

    print(
        f"[DOCK CHECK] cargo center x={cargo[0]:.4f}, y={cargo[1]:.4f}"
    )
    print(
        f"[DOCK CHECK] lift center  x={lift[0]:.4f}, y={lift[1]:.4f}"
    )
    print(f"[DOCK CHECK] center error = {error_xy:.4f} m")

    return error_xy


def main():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(f"Warehouse USD not found: {WORLD_USD}")
    if not CARGO_USD.is_file():
        raise FileNotFoundError(f"Cargo USD not found: {CARGO_USD}")

    print()
    print("============================================")
    print(" IW HUB SCRIPTED CARGO TRANSPORT TEST")
    print("============================================")
    print("[INFO] ROS2 bridge : NOT started")
    print("[INFO] Nav2        : NOT started")
    print("[INFO] LiDAR       : disabled in this test where possible")
    print(f"[START] ({START_XY[0]:.3f}, {START_XY[1]:.3f})")
    print(f"[CARGO] ({POD_X:.3f}, {POD_Y:.3f})")
    print(f"[GOAL ] ({DELIVERY_X:.5f}, {DELIVERY_Y:.5f})")
    print("============================================")
    print()

    result = open_stage(str(WORLD_USD))
    if result is False:
        raise RuntimeError(f"Failed to open warehouse USD: {WORLD_USD}")

    for _ in range(5):
        simulation_app.update()

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    _spawn_cargo(stage)
    _spawn_iw_hub(stage)
    _disable_lidar_prims(stage)

    robot = world.scene.add(
        WheeledRobot(
            prim_path=ROBOT_PRIM_PATH,
            name="scripted_transport_iw_hub",
            wheel_dof_names=WHEEL_DOF_NAMES,
            create_robot=False,
        )
    )

    controller = DifferentialController(
        name="scripted_transport_controller",
        wheel_radius=WHEEL_RADIUS,
        wheel_base=WHEEL_BASE,
    )

    print("[WORLD] resetting simulation")
    world.reset()
    world.play()

    set_camera_view(
        eye=START_CAMERA_EYE,
        target=START_CAMERA_TARGET,
        camera_prim_path="/OmniverseKit_Persp",
    )

    _set_lift_target(stage, LIFT_DOWN)

    print("[VIEW] cargo/start camera applied")
    print("[TEST] showing the initial scene for 3 seconds...")
    _step_seconds(world, 3.0)
    _print_pose("START", robot)

    print()
    print("============================================")
    print(" 1. GO TO EAST PRE-DOCK")
    print("============================================")
    if not _drive_to(
        world,
        robot,
        controller,
        PRE_DOCK_X,
        PRE_DOCK_Y,
        EMPTY_LINEAR_SPEED,
        EMPTY_ANGULAR_SPEED,
        POSITION_TOLERANCE,
        timeout_seconds=40.0,
    ):
        raise RuntimeError("Failed to reach cargo pre-dock point")
    _print_pose("PRE-DOCK", robot)

    print()
    print("============================================")
    print(" 2. ALIGN FOR REVERSE DOCKING")
    print("============================================")
    if not _rotate_to(
        world,
        robot,
        controller,
        DOCK_YAW,
        DOCK_ANGULAR_SPEED,
        timeout_seconds=25.0,
    ):
        raise RuntimeError("Failed to align for cargo docking")
    _print_pose("ALIGNED", robot)

    print()
    print("============================================")
    print(" 3. REVERSE UNDER POD")
    print("============================================")
    print(
        f"[DOCK TARGET] root x={DOCK_ROOT_X:.4f}, "
        f"y={DOCK_ROOT_Y:.4f}, yaw=0 deg"
    )

    if not _reverse_to(
        world,
        robot,
        controller,
        DOCK_ROOT_X,
        DOCK_ROOT_Y,
        DOCK_LINEAR_SPEED,
        DOCK_ANGULAR_SPEED,
        DOCK_POSITION_TOLERANCE,
        timeout_seconds=50.0,
    ):
        raise RuntimeError("Failed to reverse under cargo pod")

    _stop_robot(robot, controller)
    _step_seconds(world, 2.0)
    _print_pose("DOCKED", robot)

    dock_error = _print_dock_check(stage)
    if dock_error > 0.035:
        raise RuntimeError(
            f"Lift center is too far from cargo center: {dock_error:.4f} m"
        )

    cargo_before_lift = _world_position(stage, CARGO_PRIM_PATH)
    print(f"[CARGO] before lift z={cargo_before_lift[2]:.4f} m")

    print()
    print("============================================")
    print(" 4. LIFT UP")
    print("============================================")
    _set_lift_target(stage, LIFT_UP)
    _step_seconds(world, 4.0)

    cargo_after_lift = _world_position(stage, CARGO_PRIM_PATH)
    lifted_delta = cargo_after_lift[2] - cargo_before_lift[2]
    print(
        f"[CARGO] after lift z={cargo_after_lift[2]:.4f} m, "
        f"delta={lifted_delta:.4f} m"
    )

    if lifted_delta < 0.005:
        raise RuntimeError(
            "Cargo was not lifted. Stop transport test and check docking."
        )

    print("[PASS] Cargo lift confirmed. Starting loaded transport.")

    print()
    print("============================================")
    print(" 5. TRANSPORT CARGO TO DESTINATION")
    print("============================================")
    print(
        f"[GOAL] x={DELIVERY_X:.5f}, y={DELIVERY_Y:.5f} "
        "(scripted, no obstacle avoidance)"
    )

    if not _drive_to(
        world,
        robot,
        controller,
        DELIVERY_X,
        DELIVERY_Y,
        LOADED_LINEAR_SPEED,
        LOADED_ANGULAR_SPEED,
        POSITION_TOLERANCE,
        timeout_seconds=160.0,
    ):
        raise RuntimeError("Failed to reach the delivery coordinate")

    _stop_robot(robot, controller)
    _step_seconds(world, 2.0)

    _print_pose("GOAL", robot)
    cargo_goal = _world_position(stage, CARGO_PRIM_PATH)
    print(
        f"[CARGO] goal position: "
        f"x={cargo_goal[0]:.3f}, y={cargo_goal[1]:.3f}, "
        f"z={cargo_goal[2]:.3f}"
    )

    print()
    print("============================================")
    print(" TEST COMPLETE")
    print("============================================")
    print("[PASS] IW Hub reached the selected destination.")
    print("[INFO] Lift remains UP intentionally.")
    print("[INFO] Simulation stays open for visual inspection.")
    print("[INFO] Press Ctrl+C in the terminal to close it.")
    print("============================================")

    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        print()
        print("[SYSTEM] Ctrl+C received")
    finally:
        _stop_robot(robot, controller)
        world.stop()
        simulation_app.close()
        print("[SYSTEM] Cargo transport test closed")


if __name__ == "__main__":
    main()
