"""Spawn a second cargo_box_gaurd_size_201 using the source object's real USD reference."""

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


SOURCE_GUARD_NAME = "cargo_box_gaurd_size_201"
POSE_TOLERANCE_M = 1.0e-6


def _find_unique_source_prim(stage, source_name, destination_path):
    """Find the already-authored warehouse guard without guessing its prim path."""

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


def _reference_items_from_spec(prim_spec):
    """Return all authored Sdf.Reference items from one prim spec."""

    reference_list = prim_spec.referenceList
    items = []

    for attr_name in (
        "explicitItems",
        "prependedItems",
        "appendedItems",
        "addedItems",
    ):
        try:
            values = getattr(reference_list, attr_name)
        except Exception:
            values = None
        if values:
            items.extend(list(values))

    return items


def _discover_source_reference(source_prim):
    """Resolve the actual external USD reference used by the source guard.

    This intentionally follows the same pattern that worked for cargo_pod:
    find the asset reference -> DefinePrim -> AddReference -> set transform.
    """

    candidates = []

    for prim_spec in source_prim.GetPrimStack():
        for reference in _reference_items_from_spec(prim_spec):
            asset_path = str(reference.assetPath or "").strip()
            if not asset_path:
                continue

            try:
                resolved_asset = Sdf.ComputeAssetPathRelativeToLayer(
                    prim_spec.layer,
                    asset_path,
                )
            except Exception:
                resolved_asset = asset_path

            candidates.append(
                (
                    str(resolved_asset),
                    reference.primPath,
                    prim_spec.layer.identifier,
                )
            )

    if not candidates:
        raise RuntimeError(
            f"No external USD reference found on source guard "
            f"{source_prim.GetPath()}. Cannot safely spawn a second guard."
        )

    asset_path, prim_path, layer_identifier = candidates[0]
    print(
        "[CARGO GUARD] source reference discovered: "
        f"asset={asset_path}, prim={prim_path}, layer={layer_identifier}"
    )
    return asset_path, prim_path


def _ensure_compound_collision(root_prim):
    """Use referenced visual meshes as collision only if none already exist."""

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
    """Spawn cargo_box_gaurd_size_201 exactly like the old cargo_pod asset."""

    source_prim = _find_unique_source_prim(
        stage,
        str(source_name),
        str(destination_path),
    )
    asset_path, source_asset_prim_path = _discover_source_reference(source_prim)
    destination_path = str(destination_path)

    existing = stage.GetPrimAtPath(destination_path)
    if existing.IsValid():
        stage.RemovePrim(destination_path)

    destination = stage.DefinePrim(destination_path, "Xform")
    references = destination.GetReferences()

    if str(source_asset_prim_path):
        references.AddReference(asset_path, source_asset_prim_path)
    else:
        references.AddReference(asset_path)

    destination.SetActive(True)

    xform = UsdGeom.Xformable(destination)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in spawn_xyz]))
    if float(spawn_yaw) != 0.0:
        xform.AddRotateZOp().Set(float(spawn_yaw))

    stage.Load(destination_path)
    destination = stage.GetPrimAtPath(destination_path)
    if not destination.IsValid():
        raise RuntimeError(f"Spawned cargo guard prim is invalid: {destination_path}")

    child_count = sum(1 for _ in destination.GetChildren())
    if child_count == 0:
        raise RuntimeError(
            f"Cargo guard reference loaded but no child geometry appeared: "
            f"{destination_path} <- {asset_path}"
        )

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
        f"[CARGO GUARD] referenced {asset_path} -> {destination_path}; "
        f"children={child_count}; pose=({float(spawn_xyz[0]):.3f}, "
        f"{float(spawn_xyz[1]):.3f}, {float(spawn_xyz[2]):.3f}), "
        f"yaw={float(spawn_yaw):.1f} deg"
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
