"""Spawn cargo_box_gaurd_size_201 with the proven baseline cargo_pod flow.

Flow:
1) Define /World/Cargo/cargo_box_gaurd_size_201.
2) AddReference() the standalone guard USD directly on that prim.
3) Apply the exact requested pose.
4) Add simple runtime compound colliders (4 legs + floor + 4 walls).
5) Apply one rigid body + mass on the cargo root.

The visual USD is not required to expose editable Mesh/Gprim descendants. This
matches the old cargo_pod approach, where the visual and runtime physics were
kept separate.
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
    bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if bounds.IsEmpty():
        raise RuntimeError(f"Cargo guard has empty bounds: {prim_path}")
    return bounds


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


def _disable_referenced_physics(root_prim):
    """Disable physics inside the visual asset; root physics is authoritative."""

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
    """Create baseline-style 4-leg + floor + 4-wall compound colliders."""

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

    # Same proportions as the successful baseline cargo_pod physics:
    # leg clearance 25% of height, floor thickness 5% of height.
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


def spawn_cargo_guard_clone(
    stage,
    destination_path,
    spawn_xyz,
    spawn_yaw=0.0,
    source_name=SOURCE_GUARD_NAME,
    mass_kg=20.0,
):
    """Spawn the guard using the exact baseline cargo_pod creation pattern."""

    del source_name
    destination_path = str(destination_path)
    asset_path = str(GUARD_USD)

    if not GUARD_USD.is_file():
        raise FileNotFoundError(f"Cargo guard USD not found: {GUARD_USD}")

    existing = stage.GetPrimAtPath(destination_path)
    if existing.IsValid():
        stage.RemovePrim(destination_path)

    # Baseline cargo_pod method:
    #   prim = stage.DefinePrim(..., "Xform")
    #   prim.GetReferences().AddReference(str(usd_path))
    root = stage.DefinePrim(destination_path, "Xform")
    root.GetReferences().AddReference(asset_path)

    xform = UsdGeom.Xformable(root)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(
        Gf.Vec3d(*[float(value) for value in spawn_xyz])
    )

    if abs(float(spawn_yaw)) > 1.0e-9:
        xform.AddRotateZOp().Set(float(spawn_yaw))

    stage.Load(destination_path)

    root = stage.GetPrimAtPath(destination_path)
    if not root.IsValid():
        raise RuntimeError(
            f"Spawned cargo guard prim is invalid: {destination_path}"
        )

    _verify_exact_root_pose(stage, destination_path, spawn_xyz)
    bounds = _world_bounds(stage, destination_path)

    # Just like baseline cargo_pod_physics.py, the visual asset does not need
    # collision-capable geometry. Runtime physics is authored separately.
    _disable_referenced_physics(root)
    collision_count = _add_baseline_style_physics(
        stage,
        root,
        bounds,
        spawn_xyz,
    )

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
    """Place four parcels as one 2x2 layer on top of the runtime floor."""

    bounds = _world_bounds(stage, str(guard_path))
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    size = maximum - minimum

    center_x = float(cargo_center_xy[0])
    center_y = float(cargo_center_xy[1])

    # Runtime floor top matches _add_baseline_style_physics():
    # 25% clearance + 5% floor thickness = 30% above visual bottom.
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
                f"Parcel {config['name']} does not fit inside cargo guard: "
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
        "[CARGO GUARD] parcel layer resolved: "
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
