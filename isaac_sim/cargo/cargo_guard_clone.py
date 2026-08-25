"""Spawn the cargo guard from its standalone USD asset.

This module intentionally avoids copying or internally referencing the already
composed warehouse prim.  The guard is spawned the same way the original
cargo_pod was: create a new prim, add an external USD reference, set the pose,
then add runtime physics.
"""

from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics


SOURCE_GUARD_NAME = "cargo_box_gaurd_size_201"
POSE_TOLERANCE_M = 1.0e-6
GUARD_USD = (
    Path(__file__).resolve().parents[1]
    / "usd"
    / "warehouse_final_final"
    / "cargo"
    / "cargo box_gaurd_size_200.usd"
)


def _world_bounds(stage, prim_path):
    prim = stage.GetPrimAtPath(str(prim_path))
    if not prim.IsValid():
        raise RuntimeError(f"Cargo guard prim is invalid: {prim_path}")

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
    prim = stage.GetPrimAtPath(str(prim_path))
    transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    position = transform.ExtractTranslation()

    actual = tuple(float(position[i]) for i in range(3))
    expected = tuple(float(value) for value in expected_xyz)

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


def _disable_nested_rigid_bodies(root_prim):
    """Keep one authoritative rigid body on the spawned cargo root."""

    for prim in Usd.PrimRange(root_prim):
        if prim == root_prim or prim.IsInstanceProxy():
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)


def _ensure_collision_shapes(root_prim):
    """Use authored collision when present, otherwise generate it from meshes."""

    existing = [
        prim
        for prim in Usd.PrimRange(root_prim)
        if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    if existing:
        return len(existing), False

    created = 0
    for prim in Usd.PrimRange(root_prim):
        if prim == root_prim or prim.IsInstanceProxy():
            continue
        if not prim.IsA(UsdGeom.Gprim):
            continue

        UsdPhysics.CollisionAPI.Apply(prim)
        if prim.IsA(UsdGeom.Mesh):
            mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
            # The guard is concave/open, so convex decomposition preserves the
            # usable interior better than one solid convex hull.
            mesh_collision.CreateApproximationAttr().Set("convexDecomposition")
        created += 1

    if created == 0:
        raise RuntimeError(
            f"Cargo guard {root_prim.GetPath()} has no collision-capable geometry"
        )

    return created, True


def spawn_cargo_guard_clone(
    stage,
    destination_path,
    spawn_xyz,
    spawn_yaw=0.0,
    source_name=SOURCE_GUARD_NAME,
    mass_kg=20.0,
):
    """Spawn one cargo guard by directly referencing its standalone USD.

    ``source_name`` is kept only for compatibility with the existing mission
    call/config.  No live warehouse prim is copied anymore.
    """

    del source_name
    destination_path = str(destination_path)
    asset_path = str(GUARD_USD)

    if not GUARD_USD.is_file():
        raise FileNotFoundError(f"Cargo guard USD not found: {GUARD_USD}")

    existing = stage.GetPrimAtPath(destination_path)
    if existing.IsValid():
        stage.RemovePrim(destination_path)

    root = stage.DefinePrim(destination_path, "Xform")

    # Keep the asset's own root transform/material structure untouched under a
    # wrapper.  The wrapper owns the requested world pose and rigid-body state.
    asset_prim_path = f"{destination_path}/Asset"
    asset_prim = stage.DefinePrim(asset_prim_path)
    asset_prim.GetReferences().AddReference(asset_path)

    root_xform = UsdGeom.Xformable(root)
    root_xform.ClearXformOpOrder()
    root_xform.AddTranslateOp().Set(
        Gf.Vec3d(*[float(value) for value in spawn_xyz])
    )
    if abs(float(spawn_yaw)) > 1.0e-9:
        root_xform.AddRotateZOp().Set(float(spawn_yaw))

    stage.Load(destination_path)
    root = stage.GetPrimAtPath(destination_path)
    if not root.IsValid():
        raise RuntimeError(
            f"Spawned cargo guard prim is invalid: {destination_path}"
        )

    _verify_exact_root_pose(stage, destination_path, spawn_xyz)
    bounds = _world_bounds(stage, destination_path)

    _disable_nested_rigid_bodies(root)
    collision_count, collision_created = _ensure_collision_shapes(root)

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(root)
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)

    mass = UsdPhysics.MassAPI.Apply(root)
    mass.CreateMassAttr(float(mass_kg))

    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    size = maximum - minimum

    print(
        f"[CARGO GUARD] standalone USD referenced: {asset_path} -> "
        f"{destination_path}; pose=({float(spawn_xyz[0]):.3f}, "
        f"{float(spawn_xyz[1]):.3f}, {float(spawn_xyz[2]):.3f}), "
        f"yaw={float(spawn_yaw):.1f} deg"
    )
    print(
        "[CARGO GUARD] world bounds: "
        f"min=({float(minimum[0]):.3f}, {float(minimum[1]):.3f}, "
        f"{float(minimum[2]):.3f}), "
        f"max=({float(maximum[0]):.3f}, {float(maximum[1]):.3f}, "
        f"{float(maximum[2]):.3f}), "
        f"size=({float(size[0]):.3f}, {float(size[1]):.3f}, "
        f"{float(size[2]):.3f}) m"
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
    """Resolve one 2x2 parcel layer inside the spawned guard bounds."""

    bounds = _world_bounds(stage, str(guard_path))
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    center_x = float(cargo_center_xy[0])
    center_y = float(cargo_center_xy[1])

    resolved = []
    common_z = None

    for config in parcel_configs:
        max_size = tuple(float(value) for value in config["max_size_xyz"])
        offset_xy = tuple(float(value) for value in config["offset_xy"])
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
                f"guard_min=({float(minimum[0]):.3f}, "
                f"{float(minimum[1]):.3f}), "
                f"guard_max=({float(maximum[0]):.3f}, "
                f"{float(maximum[1]):.3f})"
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
        position = item["spawn_xyz"]
        print(
            f"[CARGO GUARD]   {item['name']}: "
            f"({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
        )

    return resolved
