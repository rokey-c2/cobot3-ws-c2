"""Diagnose IW Hub lift control after WheeledRobot initialization.

This test does not use ROS2, Nav2, LiDAR, cargo, or driving.
It checks whether the live articulation controller has usable position-control
gains on lift_joint, explicitly switches that DOF to position mode, restores
the authored gains, and commands 0.04 m.
"""

from pathlib import Path
import sys
import numpy as np

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import Gf, Sdf, UsdGeom

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.types import ArticulationAction

enable_extension("isaacsim.robot.wheeled_robots")
simulation_app.update()

from isaacsim.robot.wheeled_robots.robots import WheeledRobot

ISAAC_SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ISAAC_SIM_DIR))

from project_config.robot_config import IW_HUB_USD

WORLD_USD = (
    ISAAC_SIM_DIR
    / "usd"
    / "enva_small_warehouse_p3020_marker"
    / "World0.usd"
)

ROBOT_PRIM_PATH = "/World/Robots/amr_a"
SOURCE_ROBOT_PRIM = Sdf.Path("/World/iw_hub_ROS")
LIFT_COLLISION_PATH = f"{ROBOT_PRIM_PATH}/lift/Collision"
LIFT_JOINT_PATH = f"{ROBOT_PRIM_PATH}/lift_joint"
WHEEL_DOF_NAMES = ["left_wheel_joint", "right_wheel_joint"]

# Values inspected directly from NVIDIA IW Hub Navigation USD.
LIFT_KP = 1_000_000.0
LIFT_KD = 1_000.0


def world_z(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    cache = UsdGeom.XformCache()
    p = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return float(p[2])


def step_seconds(world, seconds):
    dt = float(world.get_physics_dt())
    for _ in range(max(1, int(round(seconds / dt)))):
        world.step(render=True)


def main():
    if open_stage(str(WORLD_USD)) is False:
        raise RuntimeError(f"Failed to open warehouse USD: {WORLD_USD}")

    for _ in range(5):
        simulation_app.update()

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    UsdGeom.Xform.Define(stage, "/World/Robots")
    prim = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
    prim.GetReferences().AddReference(IW_HUB_USD, SOURCE_ROBOT_PRIM)

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(10.5, -1.5, 0.0))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    stage.Load(ROBOT_PRIM_PATH)

    robot = world.scene.add(
        WheeledRobot(
            prim_path=ROBOT_PRIM_PATH,
            name="lift_control_mode_probe",
            wheel_dof_names=WHEEL_DOF_NAMES,
            create_robot=False,
        )
    )

    world.reset()
    world.play()
    step_seconds(world, 1.0)

    lift_index = int(robot.get_dof_index("lift_joint"))
    controller = robot.get_articulation_controller()

    kps_before, kds_before = controller.get_gains()
    max_efforts_before = controller.get_max_efforts()
    joint_before = float(robot.get_joint_positions()[lift_index])
    z_before = world_z(stage, LIFT_COLLISION_PATH)

    print()
    print("============================================")
    print(" IW HUB LIFT CONTROL MODE DIAGNOSTIC")
    print("============================================")
    print(f"[LIFT DOF INDEX] {lift_index}")
    print(f"[BEFORE KP] {float(kps_before[lift_index]):.6f}")
    print(f"[BEFORE KD] {float(kds_before[lift_index]):.6f}")
    print(f"[BEFORE MAX EFFORT] {float(max_efforts_before[lift_index]):.6f}")
    print(f"[BEFORE JOINT] {joint_before:.6f} m")
    print(f"[BEFORE LIFT Z] {z_before:.6f} m")

    # Explicitly restore the lift DOF to position control.
    controller.switch_dof_control_mode(
        dof_index=lift_index,
        mode="position",
    )

    kps, kds = controller.get_gains()
    kps = np.array(kps, dtype=float, copy=True)
    kds = np.array(kds, dtype=float, copy=True)
    kps[lift_index] = LIFT_KP
    kds[lift_index] = LIFT_KD
    controller.set_gains(kps=kps, kds=kds, save_to_usd=False)

    # Use a finite, high temporary effort cap for this diagnostic.
    controller.set_max_efforts(
        np.array([100000.0], dtype=float),
        joint_indices=np.array([lift_index], dtype=np.int32),
    )

    kps_after, kds_after = controller.get_gains()
    max_efforts_after = controller.get_max_efforts()

    print()
    print(f"[AFTER KP] {float(kps_after[lift_index]):.6f}")
    print(f"[AFTER KD] {float(kds_after[lift_index]):.6f}")
    print(f"[AFTER MAX EFFORT] {float(max_efforts_after[lift_index]):.6f}")

    target = 0.04
    print(f"[COMMAND] lift_joint -> {target:.6f} m")

    dt = float(world.get_physics_dt())
    for _ in range(max(1, int(round(4.0 / dt)))):
        controller.apply_action(
            ArticulationAction(
                joint_positions=np.array([target], dtype=float),
                joint_indices=np.array([lift_index], dtype=np.int32),
            )
        )
        world.step(render=True)

    joint_after = float(robot.get_joint_positions()[lift_index])
    z_after = world_z(stage, LIFT_COLLISION_PATH)

    print()
    print(f"[AFTER JOINT] {joint_after:.6f} m")
    print(f"[AFTER LIFT Z] {z_after:.6f} m")
    print(f"[JOINT DELTA] {joint_after - joint_before:.6f} m")
    print(f"[LIFT Z DELTA] {z_after - z_before:.6f} m")

    if joint_after - joint_before >= 0.03:
        print("[PASS] lift_joint moves after restoring position mode/gains")
    else:
        print("[FAIL] lift_joint still does not move")
        print("[NEXT] The joint geometry exists, but the live PhysX articulation needs deeper inspection")

    print("[INFO] Isaac stays open. Ctrl+C to close.")

    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        pass
    finally:
        world.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
