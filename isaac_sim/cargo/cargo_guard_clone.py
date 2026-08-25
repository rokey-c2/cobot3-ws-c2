"""Runtime clone/physics helper for the warehouse cargo guard."""

import omni.usd
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
    """Duplicate cargo_box_gaurd_size_201 at the exact old cargo_pod pose.

    The source prim path is discovered from the loaded warehouse at runtime.
    omni.usd.duplicate_prim() is used instead of an internal reference because
    the source guard can be composed from referenced/binary warehouse layers.
    """

    source_prim = _find_unique_source_prim(
        stage,
        str(source_name),
        str(destination_path),
    )
    source_path = source_prim.GetPath().pathString
    destination_path = str(destination_path)

    existing = stage.GetPrimAtPath(destination_path)
    if existing.IsValid():
        stage.RemovePrim(destination_path)

    duplicated = omni.usd.duplicate_prim(
        stage,
        source_path,
        destination_path,
        duplicate_layers=True,
    )
    if not duplicated:
        raise RuntimeError(
            f"Failed to duplicate cargo guard {source_path} -> {destination_path}"
        )

    stage.Load(destination_path)
    destination = stage.GetPrimAtPath(destination_path)
    if not destination.IsValid():
        raise RuntimeError(
            f"Duplicated cargo guard prim is invalid: {destination_path}"
        )

    # Override the duplicated source transform so the new guard is at the exact
    # old cargo_pod pose: (10.5, -1.5, 0.5), yaw=0 deg.
    xform = UsdGeom.Xformable(destination)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in spawn_xyz]))
    if float(spawn_yaw) != 0.0:
        xform.AddRotateZOp().Set(float(spawn_yaw))

    _verify_exact_root_pose(stage, destination_path, spawn_xyz)

    _disable_nested_rigid_bodies(destination)
    collision_count, collision_created = _ensure_compound_collision(destination)

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(destination)
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)

    mass = UsdPhysics.MassAPI.Apply(destination)
    mass.CreateMassAttr(float(mass_kg))

    bounds = _world_bounds(stage, destination_path)
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    size = maximum - minimum

    print(
        f"[CARGO GUARD] duplicated {source_path} -> {destination_path}; "
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

    return destination_path


def resolve_parcel_layer(
    stage,
    guard_path,
    parcel_configs,
    cargo_center_xy,
    bottom_clearance_m=0.03,
    wall_clearance_m=0.02,
):
    """Resolve one 2x2 parcel layer inside the ACTUAL cloned guard bounds."""

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
