"""Generate synthetic parcel-box detection data (RGB + 2D bbox) using Isaac Sim Replicator."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import random

import numpy as np
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.semantics import add_labels
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

assets_root_path = get_assets_root_path()
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()


def random_z_quat():
    """Random Gf.Quatd representing a rotation about the Z axis only."""
    angle = random.uniform(0, 2 * np.pi)
    return Gf.Quatd(float(np.cos(angle / 2)), Gf.Vec3d(0.0, 0.0, float(np.sin(angle / 2))))


def add_colliders(root_prim):
    for desc_prim in Usd.PrimRange(root_prim):
        if desc_prim.IsA(UsdGeom.Mesh) or desc_prim.IsA(UsdGeom.Gprim):
            collision_api = UsdPhysics.CollisionAPI.Apply(desc_prim)
            collision_api.CreateCollisionEnabledAttr(True)
            if desc_prim.IsA(UsdGeom.Mesh):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
                mesh_collision_api.CreateApproximationAttr().Set("convexHull")


def add_rigid_body(prim):
    rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid_body_api.CreateRigidBodyEnabledAttr(True)


# Physics scene
UsdPhysics.Scene.Define(stage, "/PhysicsScene")
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/PhysicsScene"))

# Light
light = stage.DefinePrim("/World/Lights/DistantLight", "DistantLight")
light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(1000.0)

# Ground plane (stand-in for a table/conveyor surface) - static collider (no RigidBodyAPI => static)
ground = stage.DefinePrim("/World/Ground", "Cube")
UsdGeom.Xformable(ground).AddScaleOp().Set((2.0, 2.0, 0.02))
UsdGeom.Xformable(ground).AddTranslateOp().Set((0, 0, -0.01))
add_colliders(ground)

# Parcel box assets (built-in Isaac Sim cardboard box props)
box_urls = [
    "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01_414.usd",
    "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04_1847.usd",
]
NUM_BOXES = 5
SPAWN_XY_RANGE = 0.6  # +/- meters: wide enough that boxes don't all overlap
box_prims = []
for i in range(NUM_BOXES):
    # Wrap the reference in its own child prim so our xform ops never collide with whatever
    # xformOps the referenced asset's own root prim already carries (referenced box assets
    # already ship their own xformOp:orient on their root, on the same prim path).
    container_path = f"/World/Boxes/box_{i}"
    container = stage.DefinePrim(container_path, "Xform")
    geo_prim = stage.DefinePrim(f"{container_path}/geo", "Xform")
    geo_prim.GetReferences().AddReference(assets_root_path + random.choice(box_urls))

    xf = UsdGeom.Xformable(container)
    xf.AddTranslateOp().Set(
        (
            random.uniform(-SPAWN_XY_RANGE, SPAWN_XY_RANGE),
            random.uniform(-SPAWN_XY_RANGE, SPAWN_XY_RANGE),
            0.3 + i * 0.15,  # staggered drop height so they don't spawn already overlapping
        )
    )
    xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(random_z_quat())
    add_labels(container, labels=["box"], instance_name="class")
    add_colliders(container)
    add_rigid_body(container)
    box_prims.append(container)

# Camera (stand-in for the gripper-mounted RealSense; replace with measured offset later)
# USD cameras look down their local -Z axis by default, so with no rotation a camera
# placed above the origin already looks straight down at the boxes on the ground.
# NOTE: the default clippingRange near plane is 1.0m -- with the camera also ~1.0m up,
# everything below fell inside/behind the near clip and rendered as solid black.
cam = stage.DefinePrim("/World/Camera", "Camera")
UsdGeom.Xformable(cam).AddTranslateOp().Set((0, 0, 3.0))
cam.GetAttribute("clippingRange").Set((0.01, 10000))
cam.GetAttribute("focalLength").Set(15.0)  # ~70 deg FOV, similar to a RealSense

simulation_app.update()

# Render Product + Writer
rp = rep.create.render_product(cam.GetPath(), (640, 480))
writer = rep.writers.get("BasicWriter")
writer.initialize(
    output_dir="_out_parcel_sdg",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=False,
)
writer.attach([rp])

rep.orchestrator.set_capture_on_play(False)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

# Once physics is running, rigid bodies must be teleported through the physics API
# (RigidPrim), not by writing USD xformOps directly -- PhysX owns the transform once
# simulating, and Set()-ing the old xformOp handles crashes with "invalid prim".
boxes_view = RigidPrim(f"/World/Boxes/box_*")
boxes_view.initialize()

# Let the boxes fall and settle before the first capture
for _ in range(60):
    simulation_app.update()


def rethrow_boxes():
    positions = np.array(
        [
            (random.uniform(-SPAWN_XY_RANGE, SPAWN_XY_RANGE), random.uniform(-SPAWN_XY_RANGE, SPAWN_XY_RANGE), 0.3)
            for _ in range(NUM_BOXES)
        ]
    )
    angles = np.random.uniform(0, 2 * np.pi, size=NUM_BOXES)
    orientations = np.stack([np.cos(angles / 2), np.zeros(NUM_BOXES), np.zeros(NUM_BOXES), np.sin(angles / 2)], axis=1)
    boxes_view.set_world_poses(positions=positions, orientations=orientations)
    boxes_view.set_velocities(np.zeros((NUM_BOXES, 6)))
    # Let the re-thrown boxes fall and settle before the next capture
    for _ in range(40):
        simulation_app.update()


num_frames = 100
for i in range(num_frames):
    if i % 10 == 0:
        rethrow_boxes()
    simulation_app.update()
    rep.orchestrator.step()

rep.orchestrator.wait_until_complete()
timeline.stop()
simulation_app.close()
