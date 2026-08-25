"""Spawn a second cargo_box_gaurd_size_201 from the live warehouse prim.

The source guard is already composed in the loaded warehouse USD.  Its root
has no standalone external asset reference, and flattening the whole stage
creates prototype references that cannot be copied by themselves.  Instead,
this helper creates a new root and internally references each of the source
root's direct children.  That keeps the original composed geometry/materials
inside the same live stage without copying broken Flattened_Prototype paths.
"""

from pxr import Gf, Usd, UsdGeom, UsdPhysics


SOURCE_GUARD_NAME = "cargo_box_gaurd_size_201"
POSE_TOLERANCE_M = 1.0e-6


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


def _source_root_scale(source_prim):
    """Preserve only the source root scale; destination translation/yaw are new."""

    try:
        xformable = UsdGeom.Xformable(source_prim)
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                value = op.Get()
                if value is not None:
                    return Gf.Vec3d(
                        float(value[0]),
                        float(value[1]),
                        float(value[2]),
                    )
    except Exception:
        pass

    return Gf.Vec3d(1.0, 1.0, 1.0)


def _spawn_from_child_references(stage, source_prim, destination_path):
    """Create a second guard by referencing each source child in-place.

    Referencing the live source children avoids the broken
    /Flattened_Prototype_* paths produced by stage.Flatten().  The children keep
    their original local transforms, geometry and material relationships while
    the new root supplies the requested cargo pose.
    """

    destination_path = str(destination_path)
    existing = stage.GetPrimAtPath(destination_path)
    if existing.IsValid():
        stage.RemovePrim(destination_path)

    source_children = list(source_prim.GetChildren())
    if not source_children:
        raise RuntimeError(
            f"Source cargo guard has no direct children: {source_prim.GetPath()}"
        )

    destination = stage.DefinePrim(destination_path, "Xform")

    created_children = []
    for source_child in source_children:
        child_name = source_child.GetName()
        child_path = f"{destination_path}/{child_name}"
        type_name = str(source_child.GetTypeName() or "Xform")

        destination_child = stage.DefinePrim(child_path, type_name)
        destination_child.GetReferences().AddInternalReference(
            source_child.GetPath()
        )

        # A strong local non-instance opinion lets PhysX/collision APIs inspect
        # the referenced subtree instead of leaving it as an instance proxy.
        destination_child.SetInstanceable(False)
        created_children.append(child_path)

    stage.Load(destination_path)
    destination = stage.GetPrimAtPath(destination_path)
    if not destination.IsValid():
        raise RuntimeError(
            f"Spawned cargo guard prim is invalid: {destination_path}"
        )

    print(
        f"[CARGO GUARD] live child references created: "
        f"{source_prim.GetPath()} -> {destination_path}; "
        f"children={len(created_children)}"
    )
    for child_path in created_children:
        print(f"[CARGO GUARD]   child: {child_path}")

    return destination


def _set_exact_root_pose(destination, spawn_xyz, spawn_yaw, source_scale):
    """Set the exact former cargo_pod root pose and preserve source scale."""

    xform = UsdGeom.Xformable(destination)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(
        Gf.Vec3d(*[float(value) for value in spawn_xyz])
    )

    if abs(float(spawn_yaw)) > 1.0e-9:
        xform.AddRotateZOp().Set(float(spawn_yaw))

    if any(abs(float(source_scale[i]) - 1.0) > 1.0e-9 for i in range(3)):
        xform.AddScaleOp().Set(
            Gf.Vec3f(
                float(source_scale[0]),
                float(source_scale[1]),
                float(source_scale[2]),
            )
        )


def _ensure_compound_collision(root_prim):
    """Use referenced visual geometry as collision only if none already exist."""

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
    """The spawned cargo root owns the only active rigid body."""

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
        child_info = []
        for child in prim.GetChildren():
            child_info.append(
                f"{child.GetPath()}(type={child.GetTypeName()}, "
                f"active={child.IsActive()}, loaded={child.IsLoaded()})"
            )
        raise RuntimeError(
            f"Cargo guard has empty bounds: {prim_path}; "
            f"children={child_info}"
        )
    return aligned


def _verify_exact_root_pose(stage, prim_path, expected_xyz):
    prim = stage.GetPrimAtPath(prim_path)
    transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    position = transform.ExtractTranslation()

    actual = tuple(float(position[i]) for i in range(3))
    expected = tuple(float(v) for v in expected_xyz)

    for axis_name, actual_value, expected_value in zip(
        ("x", "y", "z"), actual, expected
    ):
        if abs(actual_value - expected_value) > POSE_TOLERANCE_M:
            raise RuntimeError(
                f"Cargo guard {axis_name} coordinate mismatch: "
                f"actual={actual_value:.9f}, expected={expected_value:.9f}"
            )

    print(
        "[CARGO GUARD] exact root pose verified: "
        f"({actual[0]:.6f}, {actual[1]:.6f}, {actual[2]:.6f})"
    )


def spawn_cargo_guard_clone(
    stage,
    destination_path,
    spawn_xyz,
    spawn_yaw=0.0,
    source_name=SOURCE_GUARD_NAME,
    mass_kg=20.0,
):
    """Spawn cargo_box_gaurd_size_201 at the exact former cargo_pod pose."""

    source_prim = _find_unique_source_prim(
        stage,
        str(source_name),
        str(destination_path),
    )
    source_path = source_prim.GetPath().pathString
    destination_path = str(destination_path)
    source_scale = _source_root_scale(source_prim)

    destination = _spawn_from_child_references(
        stage,
        source_prim,
        destination_path,
    )
    _set_exact_root_pose(
        destination,
        spawn_xyz,
        spawn_yaw,
        source_scale,
    )
    _verify_exact_root_pose(stage, destination_path, spawn_xyz)

    # Bounds are checked BEFORE physics editing.  If the referenced children did
    # not compose visually, fail here with useful child diagnostics instead of
    # continuing with an invisible Xform-only cargo object.
    bounds = _world_bounds(stage, destination_path)

    _disable_nested_rigid_bodies(destination)
    collision_count, collision_created = _ensure_compound_collision(destination)

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(destination)
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)

    mass = UsdPhysics.MassAPI.Apply(destination)
    mass.CreateMassAttr(float(mass_kg))

    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    size = maximum - minimum

    print(
        f"[CARGO GUARD] spawned from live children {source_path} -> "
        f"{destination_path}; pose=({float(spawn_xyz[0]):.3f}, "
        f"{float(spawn_xyz[1]):.3f}, {float(spawn_xyz[2]):.3f}), "
        f"yaw={float(spawn_yaw):.1f} deg, "
        f"source_scale=({float(source_scale[0]):.3f}, "
        f"{float(source_scale[1]):.3f}, {float(source_scale[2]):.3f})"
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

    return destination_path


def resolve_parcel_layer(
    stage,
    guard_path,
    parcel_configs,
    cargo_center_xy,
    bottom_clearance_m=0.03,
    wall_clearance_m=0.02,
):
    """Resolve one 2x2 parcel layer inside the actual spawned guard bounds."""

    bounds = _world_bounds(stage, str(guard_path))
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    center_x = float(cargo_center_xy[0])
    center_y = float(cargo_center_xy[1])

    resolved = []
    common_z = None

    for config in parcel_configs:
        max_size = tuple(float(v) for v in config["max_size_xyz"])
        offset_xy = tuple(float(v) for v in config["offset_xy"])
        parcel_x = center_x + offset_xy[0]
        parcel_y = center_y + offset_xy[1]

        half_x = 0.5 * max_size[0]
        half_y = 0.5 * max_size[1]
        half_z = 0.5 * max_size[2]

        if common_z is None:
            common_z = float(minimum[2]) + half_z + float(bottom_clearance_m)

        inside_x = (
            parcel_x - half_x >= float(minimum[0]) + wall_clearance_m
            and parcel_x + half_x <= float(maximum[0]) - wall_clearance_m
        )
        inside_y = (
            parcel_y - half_y >= float(minimum[1]) + wall_clearance_m
            and parcel_y + half_y <= float(maximum[1]) - wall_clearance_m
        )

        if not (inside_x and inside_y):
            raise RuntimeError(
                f"Parcel {config['name']} does not fit inside cargo guard: "
                f"center=({parcel_x:.3f}, {parcel_y:.3f}), size={max_size}, "
                f"guard_min=({float(minimum[0]):.3f}, {float(minimum[1]):.3f}), "
                f"guard_max=({float(maximum[0]):.3f}, {float(maximum[1]):.3f})"
            )

        item = dict(config)
        item["spawn_xyz"] = (parcel_x, parcel_y, common_z)
        resolved.append(item)

    print(
        "[CARGO GUARD] parcel layer resolved INSIDE guard: "
        f"center=({center_x:.3f}, {center_y:.3f}), z={common_z:.3f}, "
        f"count={len(resolved)}"
    )
    for item in resolved:
        p = item["spawn_xyz"]
        print(
            f"[CARGO GUARD]   {item['name']}: "
            f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})"
        )

    return resolved
