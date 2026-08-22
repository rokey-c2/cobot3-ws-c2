"""Simple rigid-body physics for the STEP cargo pod."""

from pxr import Gf, UsdGeom, UsdPhysics


# The STEP model is 1.0 x 1.0 x 1.0 m.
# Local Z = -0.5 is the bottom of the four legs.
# Local Z = -0.25 is the underside of the cargo floor.
# This leaves about 0.25 m of clearance for the IW Hub.


def _create_collision_box(stage, path, center, size):
    """Create one visible box collider under the cargo rigid body."""

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.25, 0.1)])
    cube.CreateDisplayOpacityAttr([0.45])

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    # Keep the simplified physics boxes visible while we debug docking/lift.
    # Later this can be switched off without changing the collision geometry.
    UsdGeom.Imageable(cube.GetPrim()).MakeVisible()


def add_cargo_pod_physics(stage, prim_path, mass_kg=20.0):
    """Turn the referenced STEP mesh into one compound rigid body.

    The visual STEP mesh is left unchanged. Physics uses several simple
    box colliders so the open space below the pod stays open for the IW Hub.
    The colliders are intentionally visible during the current test stage.
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

    # Thin wall colliders keep future parcel objects inside the pod.
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

    print(
        f"[CARGO] physics enabled for {prim_path} "
        f"(mass={float(mass_kg):.1f} kg, compound box collision, visible=True)"
    )
