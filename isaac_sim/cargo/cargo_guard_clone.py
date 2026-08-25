"""Runtime clone/physics helper for the warehouse cargo guard."""

from pxr import Gf, Usd, UsdGeom, UsdPhysics


SOURCE_GUARD_NAME = "cargo_box_gaurd_size_201"


def _find_unique_source_prim(stage, source_name, destination_path):
    """Find the already-authored warehouse guard without guessing its path."""

    matches = []
    for prim in stage.TraverseAll():
        if not prim.IsValid():
            continue
        path = prim.GetPath().pathString
        if path == destination_path:
            continue
        if prim.GetName() == source_name:
            matches.append(prim)

    if len(matches) != 1:
        paths = [prim.GetPath().pathString for prim in matches]
        raise RuntimeError(
            f"Expected exactly one source prim named {source_name!r}, "
            f"found {len(matches)}: {paths}"
        )

    return matches[0]


def _ensure_compound_collision(root_prim):
    """Use the cloned visual meshes as collision only if none already exist."""

    collision_prims = [
        prim
        for prim in Usd.PrimRange(root_prim)
        if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    if collision_prims:
        return len(collision_prims), False

    created = 0
    for prim in Usd.PrimRange(root_prim):
        if prim == root_prim or prim.IsInstanceProxy():
            continue
        if not prim.IsA(UsdGeom.Gprim):
            continue

        UsdPhysics.CollisionAPI.Apply(prim)
        if prim.IsA(UsdGeom.Mesh):
            mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_collision.CreateApproximationAttr().Set("convexHull")
        created += 1

    if created == 0:
        raise RuntimeError(
            f"Cargo guard {root_prim.GetPath()} has no collision-capable geometry"
        )

    return created, True


def _disable_nested_rigid_bodies(root_prim):
    """The cloned cargo root owns the only active rigid body."""

    for prim in Usd.PrimRange(root_prim):
        if prim == root_prim or prim.IsInstanceProxy():
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)


def _world_bounds(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_],
        useExtentsHint=True,
    )
    aligned = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if aligned.IsEmpty():
        raise RuntimeError(f"Cargo guard has empty bounds: {prim_path}")
    return aligned


def spawn_cargo_guard_clone(
    stage,
    destination_path,
    spawn_xyz,
    spawn_yaw=0.0,
    source_name=SOURCE_GUARD_NAME,
    mass_kg=20.0,
):
    """Clone the existing warehouse guard and place its root at an exact pose.

    The source prim path is discovered from the loaded warehouse at runtime, so
    this does not guess a binary-USD path.  An internal reference preserves the
    exact source object's geometry/materials while the destination root gets a
    stronger transform override.
    """

    source_prim = _find_unique_source_prim(
        stage,
        str(source_name),
        str(destination_path),
    )

    destination = stage.DefinePrim(str(destination_path), "Xform")
    destination.GetReferences().ClearReferences()
    destination.GetReferences().AddInternalReference(source_prim.GetPath())
    destination.SetActive(True)

    xform = UsdGeom.Xformable(destination)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in spawn_xyz]))
    if float(spawn_yaw) != 0.0:
        xform.AddRotateZOp().Set(float(spawn_yaw))

    stage.Load(str(destination_path))
    destination = stage.GetPrimAtPath(str(destination_path))
    if not destination.IsValid():
        raise RuntimeError(
            f"Failed to create cargo guard clone: {destination_path}"
        )

    _disable_nested_rigid_bodies(destination)
    collision_count, collision_created = _ensure_compound_collision(destination)

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(destination)
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)

    mass = UsdPhysics.MassAPI.Apply(destination)
    mass.CreateMassAttr(float(mass_kg))

    bounds = _world_bounds(stage, str(destination_path))
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    size = maximum - minimum

    print(
        f"[CARGO GUARD] cloned {source_prim.GetPath()} -> {destination_path}; "
        f"pose=({float(spawn_xyz[0]):.3f}, {float(spawn_xyz[1]):.3f}, "
        f"{float(spawn_xyz[2]):.3f}), yaw={float(spawn_yaw):.1f} deg"
    )
    print(
        "[CARGO GUARD] world bounds: "
        f"min=({float(minimum[0]):.3f}, {float(minimum[1]):.3f}, {float(minimum[2]):.3f}), "
        f"max=({float(maximum[0]):.3f}, {float(maximum[1]):.3f}, {float(maximum[2]):.3f}), "
        f"size=({float(size[0]):.3f}, {float(size[1]):.3f}, {float(size[2]):.3f}) m"
    )
    print(
        f"[CARGO GUARD] mass={float(mass_kg):.1f} kg, "
        f"collision_shapes={collision_count}, "
        f"generated_collision={collision_created}"
    )

    return str(destination_path)


def validate_parcel_layer(stage, guard_path, parcel_configs):
    """Fail early rather than silently place a parcel outside the guard XY box."""

    bounds = _world_bounds(stage, str(guard_path))
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()

    for config in parcel_configs:
        center = tuple(float(v) for v in config["spawn_xyz"])
        max_size = tuple(float(v) for v in config["max_size_xyz"])
        half_x = 0.5 * max_size[0]
        half_y = 0.5 * max_size[1]

        inside_x = (
            center[0] - half_x >= float(minimum[0])
            and center[0] + half_x <= float(maximum[0])
        )
        inside_y = (
            center[1] - half_y >= float(minimum[1])
            and center[1] + half_y <= float(maximum[1])
        )

        if not (inside_x and inside_y):
            raise RuntimeError(
                f"Parcel {config['name']} would be outside cargo guard XY bounds: "
                f"center={center}, size={max_size}, "
                f"guard_min=({float(minimum[0]):.3f}, {float(minimum[1]):.3f}), "
                f"guard_max=({float(maximum[0]):.3f}, {float(maximum[1]):.3f})"
            )

    print(
        f"[CARGO GUARD] validated {len(parcel_configs)} parcels inside "
        f"{guard_path} in one XY layer"
    )
