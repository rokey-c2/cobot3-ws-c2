"""Standalone IW Hub cargo lift physics test.

This test intentionally does not start ROS 2, Nav2, or the ROS 2 bridge.
It places the current NVIDIA IW Hub directly under the cargo pod and drives
only the existing lift_joint from 0.00 m -> 0.04 m -> 0.00 m.
"""

from pathlib import Path
import sys

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})


import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom

from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage


ISAAC_SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ISAAC_SIM_DIR))

from cargo.cargo_pod_physics import add_cargo_pod_physics
from project_config.robot_config import IW_HUB_USD


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
CARGO_PRIM_PATH = "/World/Cargo/cargo_pod"

POD_X = 10.5
POD_Y = -1.5
POD_Z = 0.5
ROBOT_Z = 0.0

LIFT_DOWN = 0.0
LIFT_UP = 0.04
CARGO_MASS_KG = 20.0


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
        f"[TEST] cargo spawned at "
        f"({POD_X:.2f}, {POD_Y:.2f}, {POD_Z:.2f})"
    )


def _spawn_iw_hub(stage):
    UsdGeom.Xform.Define(stage, "/World/Robots")

    prim = stage.DefinePrim(ROBOT_PRIM_PATH, "Xform")
    prim.GetReferences().AddReference(
        IW_HUB_USD,
        SOURCE_ROBOT_PRIM,
    )

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(POD_X, POD_Y, ROBOT_Z))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    stage.Load(ROBOT_PRIM_PATH)

    print(
        f"[TEST] IW Hub spawned directly under cargo at "
        f"({POD_X:.2f}, {POD_Y:.2f}, {ROBOT_Z:.2f})"
    )


def _set_lift_target(stage, target_position):
    joint = stage.GetPrimAtPath(LIFT_JOINT_PATH)
    if not joint.IsValid():
        raise RuntimeError(
            f"lift_joint not found: {LIFT_JOINT_PATH}"
        )

    target_attr = joint.GetAttribute(
        "drive:linear:physics:targetPosition"
    )
    if not target_attr.IsValid():
        raise RuntimeError(
            "lift_joint has no linear drive targetPosition"
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

    return (
        float(position[0]),
        float(position[1]),
        float(position[2]),
    )


def _step_seconds(world, seconds):
    physics_dt = float(world.get_physics_dt())
    steps = max(1, int(round(float(seconds) / physics_dt)))

    for _ in range(steps):
        world.step(render=True)


def main():
    if not WORLD_USD.is_file():
        raise FileNotFoundError(f"Warehouse USD not found: {WORLD_USD}")
    if not CARGO_USD.is_file():
        raise FileNotFoundError(f"Cargo USD not found: {CARGO_USD}")

    print()
    print("============================================")
    print(" IW HUB CARGO LIFT PHYSICS TEST")
    print("============================================")
    print("[INFO] ROS2 bridge: NOT started")
    print("[INFO] Nav2       : NOT started")
    print("[INFO] LiDAR test : NOT started")
    print("[TEST] IW Hub is spawned directly under the pod")
    print("[TEST] Lift sequence: 0.00 -> 0.04 -> 0.00 m")
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

    print("[WORLD] resetting simulation")
    world.reset()
    world.play()

    print("[TEST] settling robot and cargo for 3 seconds...")
    _set_lift_target(stage, LIFT_DOWN)
    _step_seconds(world, 3.0)

    cargo_down_before = _world_position(stage, CARGO_PRIM_PATH)
    print(
        f"[CARGO] settled position: "
        f"x={cargo_down_before[0]:.4f}, "
        f"y={cargo_down_before[1]:.4f}, "
        f"z={cargo_down_before[2]:.4f}"
    )

    print()
    print("============================================")
    print(" LIFT UP")
    print("============================================")
    _set_lift_target(stage, LIFT_UP)
    _step_seconds(world, 4.0)

    cargo_up = _world_position(stage, CARGO_PRIM_PATH)
    lift_delta = cargo_up[2] - cargo_down_before[2]

    print(
        f"[CARGO] UP position: "
        f"x={cargo_up[0]:.4f}, "
        f"y={cargo_up[1]:.4f}, "
        f"z={cargo_up[2]:.4f}"
    )
    print(f"[CARGO] lifted delta Z = {lift_delta:.4f} m")

    print()
    print("============================================")
    print(" LIFT DOWN")
    print("============================================")
    _set_lift_target(stage, LIFT_DOWN)
    _step_seconds(world, 4.0)

    cargo_down_after = _world_position(stage, CARGO_PRIM_PATH)
    return_error = abs(cargo_down_after[2] - cargo_down_before[2])

    print(
        f"[CARGO] DOWN position: "
        f"x={cargo_down_after[0]:.4f}, "
        f"y={cargo_down_after[1]:.4f}, "
        f"z={cargo_down_after[2]:.4f}"
    )
    print(f"[CARGO] return Z error = {return_error:.4f} m")

    print()
    print("============================================")
    print(" TEST RESULT")
    print("============================================")

    if lift_delta >= 0.005:
        print("[PASS] Cargo moved upward with the IW Hub lift.")
    else:
        print("[FAIL] Cargo did not move upward enough.")
        print("       Check lift contact height / collision alignment.")

    if return_error <= 0.02:
        print("[PASS] Cargo returned close to its original floor height.")
    else:
        print("[WARN] Cargo did not return close to its original Z height.")

    print()
    print("[INFO] Simulation will stay open for visual inspection.")
    print("[INFO] Press Ctrl+C in the terminal to close it.")
    print("============================================")

    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        print()
        print("[SYSTEM] Ctrl+C received")
    finally:
        world.stop()
        simulation_app.close()
        print("[SYSTEM] Lift test closed")


if __name__ == "__main__":
    main()
