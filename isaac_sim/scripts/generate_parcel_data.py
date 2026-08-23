"""Generate synthetic parcel-box detection data (RGB + 2D bbox) using Isaac Sim Replicator.

Simulates a wrist-mounted (eye-in-hand) camera viewing boxes from varied angles/distances,
across two scenarios: boxes piled on an AMR platform, and boxes spread on a conveyor belt.
"""

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


def random_pose_looking_at(origin, radius, elevation_deg_min=25, elevation_deg_max=85):
    """Random camera position + orientation on an (upper-hemisphere) sphere around `origin`,
    aimed at `origin`. Mimics a wrist camera approaching the pile from varied angles/distances,
    while staying above the surface (never looking up through the floor)."""
    theta = random.uniform(0, 2 * np.pi)
    phi = np.radians(random.uniform(elevation_deg_min, elevation_deg_max))
    x = radius * np.cos(theta) * np.cos(phi)
    y = radius * np.sin(theta) * np.cos(phi)
    z = radius * np.sin(phi)
    origin_v = Gf.Vec3d(*origin)
    location = origin_v + Gf.Vec3d(x, y, z)
    direction = (origin_v - location).GetNormalized()
    # USD cameras look down their local -Z axis by default.
    rotation = Gf.Rotation(Gf.Vec3d(0, 0, -1), direction)
    return location, Gf.Quatd(rotation.GetQuat())


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

# Lights
distant_light = stage.DefinePrim("/World/Lights/DistantLight", "DistantLight")
distant_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(1200.0)
dome_light = stage.DefinePrim("/World/Lights/DomeLight", "DomeLight")
dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(400.0)

# Two scenarios: boxes piled on an AMR platform vs. spread out on a conveyor belt.
# spawn_xy_range is (x_half_range, y_half_range) -- square for the AMR pile, elongated
# for the conveyor strip.
# Real box sizes range up to 0.70 x 0.50 x 0.50m (SM_CardBoxA_01_414) -- drop_height_step
# must exceed the tallest box's dimension, or consecutive boxes spawn already overlapping
# and PhysX can fail to depenetrate them (they end up visibly stuck inside each other).
SCENARIOS = {
    "amr_stack": {
        "platform_size": (1.4, 1.4, 0.06),
        "platform_height": 0.9,
        "spawn_xy_range": (0.55, 0.55),
        "num_boxes": 5,
        "drop_height_min": 0.15,
        "drop_height_step": 0.55,
        "cam_radius_range": (1.2, 2.0),
        "cam_target_z_offset": 0.2,
    },
    "conveyor": {
        "platform_size": (2.0, 0.8, 0.06),
        "platform_height": 0.5,
        "spawn_xy_range": (0.85, 0.32),
        "num_boxes": 4,
        "drop_height_min": 0.1,
        "drop_height_step": 0.55,
        "cam_radius_range": (0.9, 1.6),
        "cam_target_z_offset": 0.05,
    },
}
FRAMES_PER_SCENE = 8  # how many camera angles to capture per box arrangement
MAX_BOXES = max(cfg["num_boxes"] for cfg in SCENARIOS.values())
PARK_Z = -5.0  # where unused boxes are stashed out of camera view

# Platform: a single reusable prim, repositioned/rescaled per scenario each rethrow.
platform = stage.DefinePrim("/World/Platform", "Cube")
platform_xf = UsdGeom.Xformable(platform)
platform_translate_op = platform_xf.AddTranslateOp()
platform_scale_op = platform_xf.AddScaleOp()
add_colliders(platform)

# Parcel box assets (built-in Isaac Sim cardboard box props)
box_urls = [
    "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01_414.usd",
    "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04_1847.usd",
]
box_prims = []
for i in range(MAX_BOXES):
    # Wrap the reference in its own child prim so our xform ops never collide with whatever
    # xformOps the referenced asset's own root prim already carries (referenced box assets
    # already ship their own xformOp:orient on their root, on the same prim path).
    container_path = f"/World/Boxes/box_{i}"
    container = stage.DefinePrim(container_path, "Xform")
    geo_prim = stage.DefinePrim(f"{container_path}/geo", "Xform")
    geo_prim.GetReferences().AddReference(assets_root_path + random.choice(box_urls))

    xf = UsdGeom.Xformable(container)
    xf.AddTranslateOp().Set((0, 0, PARK_Z))
    xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(random_z_quat())
    add_labels(container, labels=["box"], instance_name="class")
    add_colliders(container)
    add_rigid_body(container)
    box_prims.append(container)

# Camera (stand-in for the gripper-mounted RealSense; orbits and looks at the box pile,
# mimicking the arm approaching from varied angles instead of one fixed top-down shot).
cam = stage.DefinePrim("/World/Camera", "Camera")
cam_xf = UsdGeom.Xformable(cam)
cam_translate_op = cam_xf.AddTranslateOp()
cam_translate_op.Set((0, 0, 2.0))
cam_orient_op = cam_xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
cam_orient_op.Set(Gf.Quatd(1, 0, 0, 0))
# NOTE: default clippingRange near plane is 1.0m, which clipped out close-up views entirely.
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
boxes_view = RigidPrim("/World/Boxes/box_*")
boxes_view.initialize()

for _ in range(20):
    simulation_app.update()


def setup_scenario():
    """Pick a random scenario, reposition the platform, and re-throw the boxes for it."""
    name = random.choice(list(SCENARIOS.keys()))
    cfg = SCENARIOS[name]

    platform_translate_op.Set((0, 0, cfg["platform_height"]))
    platform_scale_op.Set(cfg["platform_size"])

    x_range, y_range = cfg["spawn_xy_range"]
    num_active = cfg["num_boxes"]
    positions = []
    for i in range(MAX_BOXES):
        if i < num_active:
            positions.append(
                (
                    random.uniform(-x_range, x_range),
                    random.uniform(-y_range, y_range),
                    cfg["platform_height"]
                    + cfg["platform_size"][2] / 2
                    + cfg["drop_height_min"]
                    + i * cfg["drop_height_step"],
                )
            )
        else:
            positions.append((0.0, 0.0, PARK_Z))
    positions = np.array(positions)

    angles = np.random.uniform(0, 2 * np.pi, size=MAX_BOXES)
    orientations = np.stack(
        [np.cos(angles / 2), np.zeros(MAX_BOXES), np.zeros(MAX_BOXES), np.sin(angles / 2)], axis=1
    )
    boxes_view.set_world_poses(positions=positions, orientations=orientations)
    boxes_view.set_velocities(np.zeros((MAX_BOXES, 6)))

    wait_until_settled(num_active)

    return name, cfg


def wait_until_settled(num_active, max_steps=300, linear_speed_threshold=0.02, min_steps=20):
    """Step physics until every active box's linear speed drops below the threshold (i.e. it has
    actually landed, not just been given enough time to -- fixes boxes still visibly falling/
    bouncing at capture time), or until max_steps as a hard safety cap."""
    for step in range(max_steps):
        simulation_app.update()
        if step < min_steps:
            continue
        velocities = boxes_view.get_velocities()[:num_active]
        linear_speed = np.linalg.norm(velocities[:, :3], axis=1)
        if np.all(linear_speed < linear_speed_threshold):
            break


def randomize_camera(cfg):
    target = (0.0, 0.0, cfg["platform_height"] + cfg["platform_size"][2] / 2 + cfg["cam_target_z_offset"])
    radius = random.uniform(*cfg["cam_radius_range"])
    location, orientation = random_pose_looking_at(target, radius)
    cam_translate_op.Set(location)
    cam_orient_op.Set(orientation)
    # Give the renderer a few frames to catch up with the new camera transform before capturing.
    for _ in range(5):
        simulation_app.update()


num_frames = 1000
scenario_cfg = None
for i in range(num_frames):
    # Re-throw the boxes only every FRAMES_PER_SCENE frames (that's the slow part, physics
    # settle); in between, just move the camera around the same arrangement and shoot again --
    # still gives varied-angle views, matches a robot orbiting/approaching the same real pile.
    if i % FRAMES_PER_SCENE == 0:
        _scenario_name, scenario_cfg = setup_scenario()
    randomize_camera(scenario_cfg)
    rep.orchestrator.step()

rep.orchestrator.wait_until_complete()
timeline.stop()
simulation_app.close()
