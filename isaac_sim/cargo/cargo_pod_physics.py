"""Rigid-body physics and simple parcel assets for the cargo pod."""

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade


BLUE = Gf.Vec3f(0.05, 0.25, 0.95)
PARCEL_BROWN = Gf.Vec3f(0.55, 0.32, 0.12)

# The STEP model is 1.0 x 1.0 x 1.0 m.
# Local Z = -0.5 is the bottom of the four legs.
# Local Z = -0.25 is the underside of the cargo floor.
# This leaves about 0.25 m of clearance for the IW Hub.


def _create_collision_box(stage, path, center, size):
    """Create one visible box collider under the cargo rigid body."""

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([BLUE])
    cube.CreateDisplayOpacityAttr([0.45])

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    # Keep the simplified physics boxes visible while we debug docking/lift.
    UsdGeom.Imageable(cube.GetPrim()).MakeVisible()


def _apply_blue_material(stage, prim_path):
    """Override the referenced STEP visual with a blue material."""

    cargo_prim = stage.GetPrimAtPath(prim_path)
    if not cargo_prim.IsValid():
        return

    material_path = f"{prim_path}/Looks/BlueCargoPod"
    shader_path = f"{material_path}/Shader"

    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, shader_path)
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(BLUE)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(),
        "surface",
    )

    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not path.startswith(f"{prim_path}/"):
            continue
        if path.startswith(f"{prim_path}/Looks/"):
            continue
        if prim.IsA(UsdGeom.Gprim):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def add_cargo_pod_physics(stage, prim_path, mass_kg=20.0):
    """Turn the referenced STEP mesh into one compound rigid body.

    Physics uses simple box colliders so the open space below the pod stays
    open for the IW Hub. The visible STEP mesh and debug colliders are both
    recolored blue for the current warehouse scenario.
    """

    cargo_prim = stage.GetPrimAtPath(prim_path)
    if not cargo_prim.IsValid():
        raise RuntimeError(f"Cargo prim does not exist: {prim_path}")

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(cargo_prim)
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)

    mass = UsdPhysics.MassAPI.Apply(cargo_prim)
    mass.CreateMassAttr(float(mass_kg))

    collision_root_path = f"{prim_path}/PhysicsColliders"
    UsdGeom.Xform.Define(stage, collision_root_path)

    # Four 100 mm x 100 mm legs, 250 mm high.
    leg_size = (0.10, 0.10, 0.25)
    leg_z = -0.375

    leg_centers = {
        "leg_front_left": (0.45, 0.45, leg_z),
        "leg_front_right": (0.45, -0.45, leg_z),
        "leg_rear_left": (-0.45, 0.45, leg_z),
        "leg_rear_right": (-0.45, -0.45, leg_z),
    }

    for name, center in leg_centers.items():
        _create_collision_box(
            stage,
            f"{collision_root_path}/{name}",
            center,
            leg_size,
        )

    # The cargo floor is about 50 mm thick.
    # Its underside is at local Z = -0.25 m, which is where the lift contacts.
    _create_collision_box(
        stage,
        f"{collision_root_path}/floor",
        (0.0, 0.0, -0.225),
        (1.0, 1.0, 0.05),
    )

    # Thin wall colliders keep parcel objects inside the pod.
    wall_height = 0.70
    wall_z = 0.15
    wall_thickness = 0.02

    _create_collision_box(
        stage,
        f"{collision_root_path}/wall_front",
        (0.49, 0.0, wall_z),
        (wall_thickness, 1.0, wall_height),
    )
    _create_collision_box(
        stage,
        f"{collision_root_path}/wall_rear",
        (-0.49, 0.0, wall_z),
        (wall_thickness, 1.0, wall_height),
    )
    _create_collision_box(
        stage,
        f"{collision_root_path}/wall_left",
        (0.0, 0.49, wall_z),
        (1.0, wall_thickness, wall_height),
    )
    _create_collision_box(
        stage,
        f"{collision_root_path}/wall_right",
        (0.0, -0.49, wall_z),
        (1.0, wall_thickness, wall_height),
    )

    _apply_blue_material(stage, prim_path)

    print(
        f"[CARGO] physics enabled for {prim_path} "
        f"(mass={float(mass_kg):.1f} kg, color=blue, compound collision)"
    )


def add_parcel_box(stage, prim_path, center, size, mass_kg=15.0):
    """Create one dynamic parcel box sized to fit inside the cargo pod.

    The parcel is intentionally a separate rigid body, not a child rigid body
    of the cargo pod, so Isaac/PhysX can simulate it resting inside the pod.
    """

    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([PARCEL_BROWN])

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    mass = UsdPhysics.MassAPI.Apply(cube.GetPrim())
    mass.CreateMassAttr(float(mass_kg))

    print(
        f"[PARCEL] spawned {prim_path}: "
        f"size={tuple(float(v) for v in size)} m, "
        f"mass={float(mass_kg):.1f} kg"
    )
