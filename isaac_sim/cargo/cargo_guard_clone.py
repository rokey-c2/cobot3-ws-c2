"""Clone the already-visible warehouse cargo guard for the AMR mission.

The warehouse already contains the correctly sized object:
    /World/cargo_box_gaurd_size_201

Do NOT load a guessed standalone USD or apply a manual scale.  Instead, make
one real duplicate of that prim, move only the duplicate root to the old
cargo_pod pose, then add the same simple runtime compound physics used by the
baseline cargo_pod.
"""

import omni.usd

from pxr import Gf, Usd, UsdGeom, UsdPhysics


SOURCE_GUARD_NAME = "cargo_box_gaurd_size_201"
POSE_TOLERANCE_M = 1.0e-6
SIZE_TOLERANCE_M = 1.0e-4


def _world_bounds(stage, prim_path):
    prim = stage.GetPrimAtPath(str(prim_path))
    if not prim.IsValid():
        raise RuntimeError(f"Cargo guard prim is invalid: {prim_path}")

    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_],
        useExtentsHint=True,
    )
    bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if bounds.IsEmpty():
        raise RuntimeError(f"Cargo guard has empty bounds: {prim_path}")
    return bounds


def _size_tuple(bounds):
    size = bounds.GetMax() - bounds.GetMin()
    return tuple(float(size[i]) for i in range(3))


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
        "[CARGO GUARD] exact clone root pose verified: "
        f"({actual[0]:.6f}, {actual[1]:.6f}, {actual[2]:.6f})"
    )


def _verify_same_visual_size(source_bounds, clone_bounds):
    source_size = _size_tuple(source_bounds)
    clone_size = _size_tuple(clone_bounds)

    for axis_name, source_value, clone_value in zip(
        ("x", "y", "z"), source_size, clone_size
    ):
        if abs(source_value - clone_value) > SIZE_TOLERANCE_M:
            raise RuntimeError(
                f"Cargo guard clone size mismatch on {axis_name}: "
                f"source={source_value:.6f} m, clone={clone_value:.6f} m"
            )

    print(
        "[CARGO GUARD] clone size matches original exactly: "
        f"source=({source_size[0]:.3f}, {source_size[1]:.3f}, "
        f"{source_size[2]:.3f}) m, "
        f"clone=({clone_size[0]:.3f}, {clone_size[1]:.3f}, "
        f"{clone_size[2]:.3f}) m"
    )


def _disable_nested_physics(root_prim):
    """The duplicate root owns the one authoritative runtime rigid body."""

    for prim in Usd.PrimRange(root_prim):
        if prim == root_prim or prim.IsInstanceProxy():
            continue

        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)


def _create_collision_box(stage, path, center, size):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()


def _add_baseline_style_physics(stage, root_prim, bounds, root_xyz):
    """Create 4 legs + floor + 4 walls, scaled to the real cloned guard."""

    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    size = maximum - minimum

    sx = float(size[0])
    sy = float(size[1])
    sz = float(size[2])
    if min(sx, sy, sz) <= 1.0e-4:
        raise RuntimeError(
            f"Invalid cargo guard bounds: size=({sx:.6f}, {sy:.6f}, {sz:.6f})"
        )

    root_x, root_y, root_z = [float(v) for v in root_xyz]
    local_min_x = float(minimum[0]) - root_x
    local_max_x = float(maximum[0]) - root_x
    local_min_y = float(minimum[1]) - root_y
    local_max_y = float(maximum[1]) - root_y
    local_min_z = float(minimum[2]) - root_z
    local_max_z = float(maximum[2]) - root_z

    center_x = 0.5 * (local_min_x + local_max_x)
    center_y = 0.5 * (local_min_y + local_max_y)

    floor_thickness = max(0.02, 0.05 * sz)
    floor_bottom_z = local_min_z + 0.25 * sz
    floor_center_z = floor_bottom_z + 0.5 * floor_thickness
    floor_top_z = floor_bottom_z + floor_thickness

    leg_width_x = max(0.05, 0.10 * sx)
    leg_width_y = max(0.05, 0.10 * sy)
    leg_height = max(0.02, floor_bottom_z - local_min_z)
    leg_center_z = local_min_z + 0.5 * leg_height
    leg_x = 0.5 * sx - 0.5 * leg_width_x
    leg_y = 0.5 * sy - 0.5 * leg_width_y

    collider_root = f"{root_prim.GetPath()}/PhysicsColliders"
    UsdGeom.Xform.Define(stage, collider_root)

    leg_centers = {
        "leg_front_left": (center_x + leg_x, center_y + leg_y, leg_center_z),
        "leg_front_right": (center_x + leg_x, center_y - leg_y, leg_center_z),
        "leg_rear_left": (center_x - leg_x, center_y + leg_y, leg_center_z),
        "leg_rear_right": (center_x - leg_x, center_y - leg_y, leg_center_z),
    }
    for name, center in leg_centers.items():
        _create_collision_box(
            stage,
            f"{collider_root}/{name}",
            center,
            (leg_width_x, leg_width_y, leg_height),
        )

    _create_collision_box(
        stage,
        f"{collider_root}/floor",
        (center_x, center_y, floor_center_z),
        (sx, sy, floor_thickness),
    )

    wall_thickness = max(0.01, 0.02 * min(sx, sy))
    wall_height = max(0.05, local_max_z - floor_top_z)
    wall_center_z = floor_top_z + 0.5 * wall_height

    _create_collision_box(
        stage,
        f"{collider_root}/wall_front",
        (local_max_x - 0.5 * wall_thickness, center_y, wall_center_z),
        (wall_thickness, sy, wall_height),
    )
    _create_collision_box(
        stage,
        f"{collider_root}/wall_rear",
        (local_min_x + 0.5 * wall_thickness, center_y, wall_center_z),
        (wall_thickness, sy, wall_height),
    )
    _create_collision_box(
        stage,
        f"{collider_root}/wall_left",
        (center_x, local_max_y - 0.5 * wall_thickness, wall_center_z),
        (sx, wall_thickness, wall_height),
    )
    _create_collision_box(
        stage,
        f"{collider_root}/wall_right",
        (center_x, local_min_y + 0.5 * wall_thickness, wall_center_z),
        (sx, wall_thickness, wall_height),
    )

    print(
        "[CARGO GUARD] baseline-style compound collision created: "
        f"size=({sx:.3f}, {sy:.3f}, {sz:.3f}) m"
    )

    return 9


def _find_source_prim(stage, source_name):
    expected_path = f"/World/{source_name}"
    source = stage.GetPrimAtPath(expected_path)
    if source.IsValid():
        return source

    matches = [
        prim
        for prim in stage.TraverseAll()
        if prim.IsValid() and prim.GetName() == source_name
    ]
    if len(matches) != 1:
        paths = [prim.GetPath().pathString for prim in matches]
        raise RuntimeError(
            f"Expected one source cargo guard named {source_name!r}; "
            f"found {len(matches)}: {paths}"
        )
    return matches[0]


def spawn_cargo_guard_clone(
    stage,
    destination_path,
    spawn_xyz,
    spawn_yaw=0.0,
    source_name=SOURCE_GUARD_NAME,
    mass_kg=20.0,
):
    """Duplicate the existing correctly sized guard and move the copy only."""

    destination_path = str(destination_path)
    source = _find_source_prim(stage, str(source_name))
    source_path = source.GetPath().pathString
    source_bounds = _world_bounds(stage, source_path)

    existing = stage.GetPrimAtPath(destination_path)
    if existing.IsValid():
        stage.RemovePrim(destination_path)

    # This is a real duplicate of the already-visible warehouse object.  No
    # guessed USD path and no 0.001/1000 scale conversion is used.
    duplicate_ok = omni.usd.duplicate_prim(
        stage,
        source_path,
        destination_path,
        duplicate_layers=True,
    )
    if duplicate_ok is False:
        raise RuntimeError(
            f"Failed to duplicate cargo guard {source_path} -> {destination_path}"
        )

    stage.Load(destination_path)
    clone = stage.GetPrimAtPath(destination_path)
    if not clone.IsValid():
        raise RuntimeError(f"Cargo guard clone is invalid: {destination_path}")

    # Preserve all child geometry/materials from the original.  Replace only
    # the root transform with the old cargo_pod pose. Original screenshot:
    # rotation=0 deg, scale=(1,1,1), so no manual scale is authored here.
    xform = UsdGeom.Xformable(clone)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(
        Gf.Vec3d(*[float(value) for value in spawn_xyz])
    )
    if abs(float(spawn_yaw)) > 1.0e-9:
        xform.AddRotateZOp().Set(float(spawn_yaw))

    _verify_exact_root_pose(stage, destination_path, spawn_xyz)
    clone_bounds = _world_bounds(stage, destination_path)
    _verify_same_visual_size(source_bounds, clone_bounds)

    _disable_nested_physics(clone)
    collision_count = _add_baseline_style_physics(
        stage,
        clone,
        clone_bounds,
        spawn_xyz,
    )

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(clone)
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)

    mass = UsdPhysics.MassAPI.Apply(clone)
    mass.CreateMassAttr(float(mass_kg))

    minimum = clone_bounds.GetMin()
    maximum = clone_bounds.GetMax()
    size = _size_tuple(clone_bounds)

    print(
        f"[CARGO GUARD] duplicated existing prim: {source_path} -> "
        f"{destination_path}; pose=({float(spawn_xyz[0]):.3f}, "
        f"{float(spawn_xyz[1]):.3f}, {float(spawn_xyz[2]):.3f}), "
        f"yaw={float(spawn_yaw):.1f} deg"
    )
    print(
        "[CARGO GUARD] clone world bounds: "
        f"min=({float(minimum[0]):.3f}, {float(minimum[1]):.3f}, "
        f"{float(minimum[2]):.3f}), "
        f"max=({float(maximum[0]):.3f}, {float(maximum[1]):.3f}, "
        f"{float(maximum[2]):.3f}), "
        f"size=({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}) m"
    )
    print(
        f"[CARGO GUARD] mass={float(mass_kg):.1f} kg, "
        f"collision_shapes={collision_count}"
    )

    return destination_path


def resolve_parcel_layer(
    stage,
    guard_path,
    parcel_configs,
    cargo_center_xy,
    floor_clearance_m=0.005,
    wall_clearance_m=0.02,
):
    """Place the four existing parcels as one 2x2 layer on the cloned guard."""

    del cargo_center_xy
    bounds = _world_bounds(stage, str(guard_path))
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    size = maximum - minimum

    # Use the actual cloned guard visual center, not a guessed asset center.
    center_x = 0.5 * (float(minimum[0]) + float(maximum[0]))
    center_y = 0.5 * (float(minimum[1]) + float(maximum[1]))

    # Same floor proportion used by the runtime compound collision above.
    floor_top_world_z = float(minimum[2]) + 0.30 * float(size[2])

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
            common_z = floor_top_world_z + half_z + float(floor_clearance_m)

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
                f"Parcel {config['name']} does not fit on cargo guard: "
                f"center=({parcel_x:.3f}, {parcel_y:.3f}), "
                f"size={max_size}, "
                f"guard_min=({float(minimum[0]):.3f}, "
                f"{float(minimum[1]):.3f}), "
                f"guard_max=({float(maximum[0]):.3f}, "
                f"{float(maximum[1]):.3f})"
            )

        item = dict(config)
        item["spawn_xyz"] = (parcel_x, parcel_y, common_z)
        resolved.append(item)

    print(
        "[CARGO GUARD] four-parcel 2x2 layer resolved on clone: "
        f"center=({center_x:.3f}, {center_y:.3f}), "
        f"z={common_z:.3f}, count={len(resolved)}"
    )
    for item in resolved:
        px, py, pz = item["spawn_xyz"]
        print(
            f"[CARGO GUARD]   {item['name']}: "
            f"({px:.3f}, {py:.3f}, {pz:.3f})"
        )

    return resolved
