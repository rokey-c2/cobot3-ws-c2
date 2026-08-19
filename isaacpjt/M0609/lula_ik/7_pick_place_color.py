#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PC A - Isaac Sim / Doosan M0609 color sorting scenario

Scenario
1) Spawn one blue/green cube at a random pick-area position.
2) Publish the wrist RGB camera as ROS 2 topic /rgb (sensor_msgs/msg/Image).
3) PC B detects color and publishes:
       /color_id : std_msgs/msg/Int32
       1 = blue
       2 = green
4) Receive /color_id on PC A.
5) Pick the cube and place it on the matching marker.

Tested conceptually against Isaac Sim 4.5~6.x APIs.  The USD scene is expected
to contain an M0609 and the wrist camera.  The script also creates a ground
plane, markers and cubes if they are not already present.

IMPORTANT:
- Run this script with Isaac Sim's Python, not system Python.
- ROS_DOMAIN_ID must be the same on PC A and PC B.
- The exact robot/camera prim paths are auto-detected where possible.
- If your USD uses different paths, change the CONFIG section below.
"""

import os
import sys
import time
import math
import random
import traceback
from enum import Enum

import numpy as np

# -----------------------------------------------------------------------------
# CONFIG
# -----------------------------------------------------------------------------

USD_PATH = os.environ.get(
    "M0609_SCENE_USD",
    "/mnt/data/m0609_camera_cube.usd",
)

ROBOT_ROOT_HINTS = [
    "/m0609",
    "/World/m0609",
]

# Camera is known from the supplied USD strings to be under this branch.
CAMERA_HINTS = [
    "realsense_d455",
    "RSD",
    "camera",
]

# Preferred end-effector/gripper prims. The script falls back to link_6.
EE_HINTS = [
    "onrobot_rg2ft",
    "tool0",
    "flange",
    "link_6",
]

# Arm DOFs. If these are present, the script uses them in this order.
ARM_JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6",
]

# ROS 2 interface
RGB_TOPIC = "/rgb"
COLOR_TOPIC = "/color_id"
COLOR_BLUE = 1
COLOR_GREEN = 2

# Image settings
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CAMERA_FRAME_ID = "m0609_wrist_camera"

# Workspace coordinates are expressed in the M0609 base frame.
# x/y values can be changed to match the table in the supplied USD.
# Conservative region near the M0609 base.  Keeping the cube close to the
# center avoids unreachable edge positions when the arm approaches from above.
PICK_X_RANGE = (0.35, 0.50)
PICK_Y_RANGE = (-0.15, 0.15)

BLUE_MARKER_LOCAL = (0.48, -0.52, 0.012)
GREEN_MARKER_LOCAL = (0.48, 0.52, 0.012)

FLOOR_Z = 0.0
CUBE_SIZE = 0.045
CUBE_Z = FLOOR_Z + CUBE_SIZE / 2.0
MARKER_SIZE = 0.13

# End-effector target offset.
# The selected EE prim is moved to this point.  If the selected prim is
# the gripper base, this is normally close to the gripper center.
EE_LOCAL_OFFSET = np.array([0.0, 0.0, 0.0])

# A fixed top-down-ish tool orientation is not imposed by the numerical IK.
# The controller uses position-only IK, preserving the current wrist posture.
# This is deliberately safer for a supplied/unknown M0609 configuration.

# Numerical IK
IK_MAX_ITER = 80
IK_POSITION_TOL = 0.008       # 8 mm
IK_DAMPING = 0.08
IK_STEP_SCALE = 0.55
IK_FINITE_DIFF = 0.002        # rad
IK_SETTLE_STEPS = 2

# Motion
HOME_Q = None                 # None = current arm joint state
GRIPPER_OPEN_TIME = 0.6
GRIPPER_CLOSE_TIME = 0.6
POST_MOVE_SETTLE = 0.5
PLAY_STARTUP_STEPS = 3      # Let PhysX create articulation handles after Play.

# -----------------------------------------------------------------------------
# Isaac Sim imports
# -----------------------------------------------------------------------------

try:
    from isaacsim import SimulationApp
except ImportError:
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RaytracedLighting",
    # Standalone Python does not load the ROS 2 bridge by default.  Enabling
    # it here also makes Isaac Sim's bundled rclpy available on Isaac Sim 5.x.
    "extra_args": ["--enable", "isaacsim.ros2.bridge"],
})

import omni
import omni.usd
from pxr import (
    Gf,
    Sdf,
    Usd,
    UsdGeom,
    UsdPhysics,
    PhysxSchema,
)

# Prefer the legacy Core API because it is still widely available in Isaac
# Sim 4.x/5.x and is convenient for this standalone script.
try:
    from omni.isaac.core import World
    from omni.isaac.core.articulations import Articulation
    from omni.isaac.core.objects import DynamicCuboid
    from omni.isaac.core.prims import XFormPrim
    from omni.isaac.core.utils.prims import is_prim_path_valid
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    CORE_STYLE = "legacy"
except ImportError:
    from isaacsim.core.api import World
    from isaacsim.core.api.articulations import Articulation
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.core.api.prims import XFormPrim
    from isaacsim.core.utils.prims import is_prim_path_valid
    from isaacsim.core.utils.stage import add_reference_to_stage
    CORE_STYLE = "new"

# ROS 2
try:
    import rclpy
    from std_msgs.msg import Int32
except Exception:
    rclpy = None
    Int32 = None


# -----------------------------------------------------------------------------
# Utility
# -----------------------------------------------------------------------------

def log(msg):
    print(f"[PC-A] {msg}", flush=True)


def find_prim_by_tokens(stage, tokens, type_name=None):
    tokens = [t.lower() for t in tokens]
    best = None
    best_score = -1
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        low = path.lower()
        if type_name and prim.GetTypeName() != type_name:
            continue
        score = sum(1 for t in tokens if t in low)
        if score > best_score:
            best_score = score
            best = prim if score > 0 else best
    return best


def find_camera_prim(stage):
    # Camera prims are explicitly typed Camera.
    candidates = []
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Camera):
            p = str(prim.GetPath())
            low = p.lower()
            score = 0
            for token in CAMERA_HINTS:
                if token.lower() in low:
                    score += 10
            if "rgb" in low:
                score += 2
            candidates.append((score, p))
    if not candidates:
        return None
    candidates.sort(reverse=True)
    return stage.GetPrimAtPath(candidates[0][1])


def find_robot_root(stage):
    for p in ROBOT_ROOT_HINTS:
        prim = stage.GetPrimAtPath(p)
        if prim and prim.IsValid():
            return prim

    prim = find_prim_by_tokens(stage, ["m0609"])
    if prim:
        return prim

    # Fallback: find an articulation root.
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim

    return None


def find_ee_prim(stage, robot_root):
    root_path = str(robot_root.GetPath())
    candidates = []
    for prim in Usd.PrimRange(robot_root):
        p = str(prim.GetPath())
        low = p.lower()
        score = 0
        for i, token in enumerate(EE_HINTS):
            if token.lower() in low:
                score += 100 - i * 10
        if "camera" in low or "realsense" in low or "rsd" in low:
            score -= 80
        if prim.IsA(UsdGeom.Xformable):
            candidates.append((score, p))
    candidates.sort(reverse=True)
    if candidates and candidates[0][0] > 0:
        return stage.GetPrimAtPath(candidates[0][1])

    p = root_path.rstrip("/") + "/link_6"
    prim = stage.GetPrimAtPath(p)
    if prim and prim.IsValid():
        return prim

    # Last resort: robot root.
    return robot_root


def prim_world_pose(prim):
    xform = UsdGeom.Xformable(prim)
    mat = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = np.array([mat[3][0], mat[3][1], mat[3][2]], dtype=np.float64)

    # Convert matrix rotation to quaternion (w, x, y, z).
    r = np.array([
        [mat[0][0], mat[0][1], mat[0][2]],
        [mat[1][0], mat[1][1], mat[1][2]],
        [mat[2][0], mat[2][1], mat[2][2]],
    ], dtype=np.float64)
    qw = math.sqrt(max(0.0, 1.0 + r[0,0] + r[1,1] + r[2,2])) / 2.0
    qx = math.copysign(
        math.sqrt(max(0.0, 1.0 + r[0,0] - r[1,1] - r[2,2])) / 2.0,
        r[2,1] - r[1,2],
    )
    qy = math.copysign(
        math.sqrt(max(0.0, 1.0 - r[0,0] + r[1,1] - r[2,2])) / 2.0,
        r[0,2] - r[2,0],
    )
    qz = math.copysign(
        math.sqrt(max(0.0, 1.0 - r[0,0] - r[1,1] + r[2,2])) / 2.0,
        r[1,0] - r[0,1],
    )
    return t, np.array([qw, qx, qy, qz], dtype=np.float64)


def robot_local_to_world(robot_root, local_xyz):
    pos, quat = prim_world_pose(robot_root.GetPrim() if hasattr(robot_root, "GetPrim") else robot_root)

    # Quaternion to rotation matrix, wxyz.
    w, x, y, z = quat
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    return pos + R @ np.asarray(local_xyz, dtype=np.float64)


def set_prim_color(prim, rgba):
    # Material creation is intentionally simple and self-contained.
    stage = prim.GetStage()
    mat_path = prim.GetPath().AppendPath(Sdf.Path("Looks/Material"))
    mat = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, mat_path.AppendPath(Sdf.Path("Shader")))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(float(rgba[0]), float(rgba[1]), float(rgba[2]))
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


# Need UsdShade only after utility definition.
from pxr import UsdShade


def ensure_ground(stage):
    path = "/World/PC_A_Ground"
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        return

    mesh = UsdGeom.Mesh.Define(stage, path)
    points = [
        Gf.Vec3f(-2, -2, 0),
        Gf.Vec3f( 2, -2, 0),
        Gf.Vec3f( 2,  2, 0),
        Gf.Vec3f(-2,  2, 0),
    ]
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateDoubleSidedAttr(True)

    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    set_prim_color(mesh.GetPrim(), (0.72, 0.72, 0.72, 1.0))


def make_visual_box(stage, path, center, size, color, rigid=False):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3d(*size))
    set_prim_color(cube.GetPrim(), color)
    if rigid:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        rb = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        rb.CreateKinematicEnabledAttr(False)
    return cube.GetPrim()


def ensure_markers(stage, robot_root):
    blue = robot_local_to_world(robot_root, BLUE_MARKER_LOCAL)
    green = robot_local_to_world(robot_root, GREEN_MARKER_LOCAL)

    make_visual_box(
        stage, "/World/PC_A_BlueMarker",
        (blue[0], blue[1], blue[2]),
        (MARKER_SIZE, MARKER_SIZE, 0.015),
        (0.05, 0.20, 1.0, 1.0),
        rigid=False,
    )
    make_visual_box(
        stage, "/World/PC_A_GreenMarker",
        (green[0], green[1], green[2]),
        (MARKER_SIZE, MARKER_SIZE, 0.015),
        (0.05, 0.95, 0.15, 1.0),
        rigid=False,
    )


def spawn_cube(stage, robot_root):
    # Remove previous scenario cube.
    old = stage.GetPrimAtPath("/World/PC_A_TargetCube")
    if old and old.IsValid():
        stage.RemovePrim(old.GetPath())

    color_id = random.choice([COLOR_BLUE, COLOR_GREEN])
    x = random.uniform(*PICK_X_RANGE)
    y = random.uniform(*PICK_Y_RANGE)
    local = (x, y, CUBE_Z)
    world = robot_local_to_world(robot_root, local)

    color = (0.05, 0.20, 1.0, 1.0) if color_id == COLOR_BLUE else (0.05, 0.95, 0.15, 1.0)

    prim = make_visual_box(
        stage, "/World/PC_A_TargetCube",
        (world[0], world[1], world[2]),
        (CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
        color,
        rigid=True,
    )

    log(f"Spawned {'BLUE' if color_id == 1 else 'GREEN'} cube at world {world}")
    log("PC B should detect the RGB image and publish /color_id.")
    return prim, color_id, world


# -----------------------------------------------------------------------------
# ROS 2
# -----------------------------------------------------------------------------

class RosBridge:
    def __init__(self):
        self.node = None
        self.sub = None
        self.latest_color = None
        self.last_color_time = 0.0

    def start(self):
        if rclpy is None:
            raise RuntimeError(
                "rclpy is unavailable. Start Isaac Sim from a ROS 2 sourced terminal "
                "and enable the Isaac Sim ROS 2 extension."
            )
        try:
            rclpy.init()
        except Exception:
            pass

        self.node = rclpy.create_node("pc_a_m0609_sorter")
        self.sub = self.node.create_subscription(
            Int32,
            COLOR_TOPIC,
            self._color_callback,
            10,
        )
        log(f"ROS2 subscriber ready: {COLOR_TOPIC} [Int32]")

    def _color_callback(self, msg):
        value = int(msg.data)
        if value in (COLOR_BLUE, COLOR_GREEN):
            self.latest_color = value
            self.last_color_time = time.time()
            log(f"Received {COLOR_TOPIC}: {value}")

    def spin_once(self):
        if self.node:
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def stop(self):
        try:
            if self.node and self.sub:
                self.node.destroy_subscription(self.sub)
            if self.node:
                self.node.destroy_node()
            if rclpy:
                rclpy.try_shutdown()
        except Exception:
            pass


# -----------------------------------------------------------------------------
# ROS2 Camera Helper graph
# -----------------------------------------------------------------------------

def setup_ros2_rgb_camera(camera_prim_path):
    """
    Uses Replicator to create a render product and Isaac Sim's ROS2 Camera
    Helper to publish sensor_msgs/msg/Image on /rgb.
    """
    try:
        import omni.replicator.core as rep
        import omni.graph.core as og

        render_product = rep.create.render_product(
            camera_prim_path,
            (IMAGE_WIDTH, IMAGE_HEIGHT),
            name="PC_A_RGB_RenderProduct",
        )
        render_product_path = str(render_product.path)

        graph_path = "/World/PC_A_ROS2_RGB"

        # Isaac Sim 5.x/6.x uses isaacsim.ros2.nodes.  Older 4.x releases
        # expose the same helper through omni.isaac.ros2_bridge.
        ext_mgr = omni.kit.app.get_app().get_extension_manager()
        if ext_mgr.is_extension_enabled("isaacsim.ros2.nodes"):
            camera_helper_type = "isaacsim.ros2.nodes.ROS2CameraHelper"
        else:
            camera_helper_type = "omni.isaac.ros2_bridge.ROS2CameraHelper"

        graph_spec = {
            "nodes": [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CameraHelper", camera_helper_type),
            ],
                "attributes": {
                    "CameraHelper.inputs:type": "rgb",
                    "CameraHelper.inputs:topicName": RGB_TOPIC.lstrip("/"),
                    "CameraHelper.inputs:frameId": CAMERA_FRAME_ID,
                    "CameraHelper.inputs:renderProductPath": render_product_path,
                    "CameraHelper.inputs:enabled": True,
                },
                "connect": [
                    ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
                ],
        }
        try:
            og.Controller.edit(
                {
                    "graph_path": graph_path,
                    "evaluator_name": "execution",
                },
                graph_spec,
            )
        except Exception:
            # Some 4.x builds require the explicit simulation pipeline stage.
            og.Controller.edit(
                {
                    "graph_path": graph_path,
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                },
                graph_spec,
            )
        log(f"ROS2 RGB publisher ready: {RGB_TOPIC}")
        log(f"Camera: {camera_prim_path}")
        return render_product_path

    except Exception as exc:
        log(f"ROS2 Camera Helper setup failed: {exc}")
        log("The rest of the sorting controller can still run, but /rgb will not publish.")
        traceback.print_exc()
        return None


# -----------------------------------------------------------------------------
# Numerical position IK
# -----------------------------------------------------------------------------

class NumericalIK:
    """
    Position-only finite-difference IK.

    It does not depend on a robot-specific Lula config.  It uses the actual
    loaded USD articulation and its selected end-effector prim, so it is useful
    for a custom M0609 + RG2 + camera USD.

    The controller deliberately uses only the first six arm joints.
    """

    def __init__(self, world, articulation, ee_prim, arm_indices):
        self.world = world
        self.art = articulation
        self.ee_prim = ee_prim
        self.arm_indices = np.asarray(arm_indices, dtype=np.int32)

    def get_q(self):
        q = np.asarray(self.art.get_joint_positions(), dtype=np.float64)
        return q.copy()

    def set_q(self, q):
        self.art.set_joint_positions(
            q,
            joint_indices=self.arm_indices,
        )
        for _ in range(IK_SETTLE_STEPS):
            self.world.step(render=False)

    def ee_position(self):
        p, _ = prim_world_pose(self.ee_prim)
        return p + EE_LOCAL_OFFSET

    def solve(self, target, on_iteration=None):
        """Move to ``target`` and optionally update an object held by the tool."""
        q_all = self.get_q()
        q = q_all[self.arm_indices].copy()

        for _ in range(IK_MAX_ITER):
            self.set_q(q)

            p = self.ee_position()
            err = np.asarray(target) - p
            err_norm = float(np.linalg.norm(err))

            if err_norm < IK_POSITION_TOL:
                self.set_q(q)
                if on_iteration:
                    on_iteration()
                    self.world.step(render=True)
                log(f"IK converged: error={err_norm:.4f} m")
                return True

            J = np.zeros((3, 6), dtype=np.float64)

            for j in range(6):
                q_test = q.copy()
                q_test[j] += IK_FINITE_DIFF
                self.set_q(q_test)
                p_plus = self.ee_position()

                q_test[j] -= 2.0 * IK_FINITE_DIFF
                self.set_q(q_test)
                p_minus = self.ee_position()

                J[:, j] = (p_plus - p_minus) / (2.0 * IK_FINITE_DIFF)

            self.set_q(q)

            # The finite-difference probes above temporarily move the arm.
            # Update the carried cube only after returning to the actual
            # solution, so it remains attached to the gripper visually.
            if on_iteration:
                on_iteration()
                self.world.step(render=True)

            # Damped least squares: dq = J^T (J J^T + λ²I)^-1 e
            A = J @ J.T + (IK_DAMPING ** 2) * np.eye(3)
            dq = J.T @ np.linalg.solve(A, err)

            norm = np.linalg.norm(dq)
            if norm > 0.20:
                dq *= 0.20 / norm

            q = q + IK_STEP_SCALE * dq

        log("IK did not converge.")
        return False


# -----------------------------------------------------------------------------
# Gripper
# -----------------------------------------------------------------------------

class GripperController:
    def __init__(self, articulation):
        self.art = articulation
        # Isaac Sim 5.x SingleArticulation exposes the joint/DOF names as
        # ``dof_names``; older Articulation wrappers use get_joint_names().
        if hasattr(self.art, "dof_names"):
            self.names = list(self.art.dof_names)
        else:
            self.names = list(self.art.get_joint_names())
        self.indices = []
        for i, name in enumerate(self.names):
            low = name.lower()
            if any(k in low for k in [
                "finger", "gripper", "knuckle", "inner", "outer",
            ]):
                if i >= 6:
                    self.indices.append(i)

        self.indices = np.asarray(sorted(set(self.indices)), dtype=np.int32)

        log(f"Detected gripper DOFs: {[self.names[i] for i in self.indices]}")

    def set_open(self):
        if len(self.indices) == 0:
            log("No gripper DOFs auto-detected; grasp is simulated by cube following EE.")
            return
        q = np.asarray(self.art.get_joint_positions(), dtype=np.float64)
        if q.ndim != 1 or q.size <= int(self.indices.max()):
            # Immediately after a timeline restart Isaac Sim can take a few
            # updates to recreate the PhysX articulation view.  Opening the
            # gripper is cosmetic at spawn time, so defer it instead of
            # failing the entire episode.
            log("Gripper is not ready yet; skipping initial open command.")
            return
        # RG2-style fingers generally open with a larger positive joint value.
        # This is configurable; if your asset is inverted, swap OPEN_POS/CLOSE_POS.
        for idx in self.indices:
            q[idx] = 0.04
        # When joint_indices are supplied, Isaac Sim expects a value for only
        # those indices, not the full articulation joint vector.
        self.art.set_joint_positions(q[self.indices], joint_indices=self.indices)

    def set_close(self):
        if len(self.indices) == 0:
            return
        q = np.asarray(self.art.get_joint_positions(), dtype=np.float64)
        if q.ndim != 1 or q.size <= int(self.indices.max()):
            log("Gripper is not ready yet; skipping close command.")
            return
        for idx in self.indices:
            q[idx] = 0.0
        self.art.set_joint_positions(q[self.indices], joint_indices=self.indices)


# -----------------------------------------------------------------------------
# Carrying / placing
# -----------------------------------------------------------------------------

def set_cube_world_position(cube_prim, world_pos):
    xform = UsdGeom.Xformable(cube_prim)
    ops = xform.GetOrderedXformOps()
    if len(ops) == 0:
        op = xform.AddTranslateOp()
    else:
        op = None
        for candidate in ops:
            if candidate.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op = candidate
                break
        if op is None:
            op = xform.AddTranslateOp()
    op.Set(Gf.Vec3d(*map(float, world_pos)))


def set_cube_kinematic(cube_prim, enabled=True):
    rb = UsdPhysics.RigidBodyAPI(cube_prim)
    if not rb:
        rb = UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    attr = rb.GetKinematicEnabledAttr()
    if not attr:
        attr = rb.CreateKinematicEnabledAttr()
    attr.Set(bool(enabled))


# -----------------------------------------------------------------------------
# Main scenario state machine
# -----------------------------------------------------------------------------

class State(Enum):
    INIT = 0
    WAIT_COLOR = 1
    MOVE_ABOVE_PICK = 2
    MOVE_PICK = 3
    CLOSE_GRIPPER = 4
    CARRY = 5
    MOVE_ABOVE_PLACE = 6
    MOVE_PLACE = 7
    OPEN_GRIPPER = 8
    FINISH = 9
    ERROR = 10


class Scenario:
    def __init__(self, world, stage):
        self.world = world
        self.stage = stage

        self.robot_root_prim = find_robot_root(stage)
        if self.robot_root_prim is None:
            raise RuntimeError("M0609 robot root was not found in the USD.")

        self.robot_root = XFormPrim(str(self.robot_root_prim.GetPath()))
        self.robot_root.initialize()

        self.ee_prim = find_ee_prim(stage, self.robot_root_prim)
        if self.ee_prim is None:
            raise RuntimeError("End-effector prim could not be found.")

        self.ee = XFormPrim(str(self.ee_prim.GetPath()))
        self.ee.initialize()

        log(f"Robot root: {self.robot_root_prim.GetPath()}")
        log(f"End effector candidate: {self.ee_prim.GetPath()}")

        self.art = Articulation(str(self.robot_root_prim.GetPath()))
        self.world.scene.add(self.art)
        self.world.reset()

        joint_names = list(self.art.dof_names)
        log(f"Robot DOFs: {joint_names}")

        arm_indices = []
        for expected in ARM_JOINT_NAMES:
            found = None
            for i, name in enumerate(joint_names):
                if name == expected:
                    found = i
                    break
            if found is None:
                raise RuntimeError(
                    f"Required arm joint '{expected}' not found. "
                    f"Available joints: {joint_names}"
                )
            arm_indices.append(found)

        self.ik = NumericalIK(
            world,
            self.art,
            self.ee_prim,
            arm_indices,
        )
        self.gripper = GripperController(self.art)

        self.ros = RosBridge()
        # A missing ROS 2 Python environment must not prevent the visual
        # scene from starting.  Without rclpy, the cube remains spawned in
        # WAIT_COLOR until ROS 2 is configured and the script is restarted.
        try:
            self.ros.start()
        except Exception as exc:
            log(f"ROS2 unavailable; starting visual scene without /color_id: {exc}")

        self.cube_prim = None
        self.spawn_color = None
        self.spawn_world = None
        self.last_color = None

        self.state = State.INIT
        self.state_time = time.time()
        self.carrying = False

    def transition(self, state):
        self.state = state
        self.state_time = time.time()
        log(f"STATE -> {state.name}")

    def spawn(self):
        ensure_ground(self.stage)
        ensure_markers(self.stage, self.robot_root_prim)
        self.cube_prim, self.spawn_color, self.spawn_world = spawn_cube(
            self.stage,
            self.robot_root_prim,
        )
        set_cube_kinematic(self.cube_prim, True)

        self.gripper.set_open()
        self.transition(State.WAIT_COLOR)

    def marker_target(self, color_id):
        local = BLUE_MARKER_LOCAL if color_id == COLOR_BLUE else GREEN_MARKER_LOCAL
        p = robot_local_to_world(self.robot_root_prim, local)
        return p

    def update_carried_cube(self):
        """Keep the kinematic target cube directly below the end effector."""
        if not self.carrying or self.cube_prim is None:
            return
        carry_pos = self.ik.ee_position().copy()
        carry_pos[2] -= 0.03
        set_cube_world_position(self.cube_prim, carry_pos)

    def move_to(self, target):
        follow_cube = self.update_carried_cube if self.carrying else None
        ok = self.ik.solve(np.asarray(target), on_iteration=follow_cube)
        self.update_carried_cube()
        self.world.step(render=True)
        time.sleep(POST_MOVE_SETTLE)
        return ok

    def tick(self):
        self.ros.spin_once()

        if self.state == State.INIT:
            self.spawn()
            return

        if self.state == State.WAIT_COLOR:
            if self.ros.latest_color not in (COLOR_BLUE, COLOR_GREEN):
                return

            self.last_color = self.ros.latest_color

            # Optional consistency check. PC B should identify the visible cube.
            if self.last_color != self.spawn_color:
                log(
                    f"WARNING: PC B reported {self.last_color}, "
                    f"but spawned cube was {self.spawn_color}. "
                    "Using PC B result as the source of truth."
                )

            # Approach from above the cube.
            above = self.spawn_world.copy()
            above[2] += 0.16
            if not self.move_to(above):
                self.transition(State.ERROR)
                return
            self.transition(State.MOVE_PICK)
            return

        if self.state == State.MOVE_PICK:
            target = self.spawn_world.copy()
            target[2] += 0.055
            if not self.move_to(target):
                self.transition(State.ERROR)
                return
            self.transition(State.CLOSE_GRIPPER)
            return

        if self.state == State.CLOSE_GRIPPER:
            self.gripper.set_close()
            time.sleep(GRIPPER_CLOSE_TIME)

            # The supplied RG2 configuration can vary.  To make the scenario
            # deterministic, the cube is now kinematically carried by the EE.
            self.carrying = True
            self.transition(State.CARRY)
            return

        if self.state == State.CARRY:
            # After /color_id is received and the gripper closes, retain the
            # cube at the tool while moving it to the selected colour marker.
            self.update_carried_cube()
            self.world.step(render=True)

            target = self.marker_target(self.last_color)
            target[2] += 0.16
            if not self.move_to(target):
                self.transition(State.ERROR)
                return
            self.transition(State.MOVE_PLACE)
            return

        if self.state == State.MOVE_PLACE:
            target = self.marker_target(self.last_color)
            target[2] += CUBE_SIZE / 2.0 + 0.02
            if not self.move_to(target):
                self.transition(State.ERROR)
                return

            set_cube_world_position(self.cube_prim, target)
            self.world.step(render=True)
            self.transition(State.OPEN_GRIPPER)
            return

        if self.state == State.OPEN_GRIPPER:
            self.gripper.set_open()
            time.sleep(GRIPPER_OPEN_TIME)
            self.carrying = False

            # Leave cube on marker.
            target = self.marker_target(self.last_color)
            target[2] += CUBE_SIZE / 2.0 + 0.012
            set_cube_world_position(self.cube_prim, target)
            set_cube_kinematic(self.cube_prim, False)

            self.transition(State.FINISH)
            return

        if self.state == State.FINISH:
            log(
                f"FINISHED: color_id={self.last_color}, "
                f"placed on {'BLUE' if self.last_color == 1 else 'GREEN'} marker."
            )
            # New episode can be started by resetting the timeline.
            return

        if self.state == State.ERROR:
            return

    def reset_episode(self):
        self.ros.latest_color = None
        self.last_color = None
        self.carrying = False
        self.spawn()
        self.transition(State.WAIT_COLOR)


# -----------------------------------------------------------------------------
# Standalone entry
# -----------------------------------------------------------------------------

def main():
    log(f"Opening USD: {USD_PATH}")

    if not os.path.isfile(USD_PATH):
        raise FileNotFoundError(
            f"USD file not found: {USD_PATH}\n"
            "Set M0609_SCENE_USD=/path/to/m0609_camera_cube.usd"
        )

    omni.usd.get_context().open_stage(USD_PATH)

    # Wait until USD is loaded.
    for _ in range(120):
        simulation_app.update()
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            break

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Failed to open USD stage.")

    log(f"Stage loaded: {stage.GetRootLayer().identifier}")

    camera_prim = find_camera_prim(stage)
    if camera_prim:
        log(f"Camera detected: {camera_prim.GetPath()}")
        setup_ros2_rgb_camera(str(camera_prim.GetPath()))
    else:
        log("WARNING: no Camera prim found. /rgb cannot be published.")

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 120.0,
        rendering_dt=1.0 / 30.0,
    )

    # The stage already contains the M0609, so do not add another robot.
    world.reset()

    scenario = Scenario(world, stage)

    # Leave the timeline paused.  Each user Play action starts a fresh episode
    # and therefore creates a cube at a new random position.
    world.stop()
    log("Timeline is paused. Press Play to spawn a random cube.")
    log(f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '<not set>')}")
    log("PC B command should publish /color_id as std_msgs/msg/Int32.")

    try:
        was_playing = False
        pending_episode_steps = None
        while simulation_app.is_running():
            world.step(render=True)
            is_playing = world.is_playing()

            # Stopped -> Play: discard the previous target and start an
            # episode with a new randomly placed blue/green cube.  Delay it
            # briefly so PhysX has created the articulation handles first.
            if is_playing and not was_playing:
                log("Play pressed: starting a new random-spawn episode.")
                pending_episode_steps = PLAY_STARTUP_STEPS

            if is_playing:
                if pending_episode_steps is not None:
                    pending_episode_steps -= 1
                    if pending_episode_steps <= 0:
                        scenario.reset_episode()
                        pending_episode_steps = None
                else:
                    scenario.tick()

            was_playing = is_playing
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
    finally:
        try:
            scenario.ros.stop()
        except Exception:
            pass
        try:
            world.stop()
        except Exception:
            pass
        simulation_app.close()


if __name__ == "__main__":
    main()
