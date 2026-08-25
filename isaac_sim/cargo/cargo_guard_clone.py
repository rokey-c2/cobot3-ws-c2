"""Clone the existing warehouse cargo guard for the AMR mission.

The source guard already has the correct X/Y size in the warehouse.  The
mission clone keeps the old cargo_pod root pose, raises only the visual body by
8 cm, and fills the added clearance with four longer runtime legs so the IW Hub
can drive underneath it.
"""

import math

import omni.usd

from pxr import Gf, Usd, UsdGeom, UsdPhysics


SOURCE_GUARD_NAME = "cargo_box_gaurd_size_201"
POSE_TOLERANCE_M = 1.0e-6
SIZE_TOLERANCE_M = 1.0e-3
LEG_EXTENSION_M = 0.08


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


def _world_position(prim):
    matrix = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    point = matrix.ExtractTranslation()
    return tuple(float(point[i]) for i in range(3))


def _verify_exact_root_pose(stage, prim_path, expected_xyz):
    prim = stage.GetPrimAtPath(str(prim_path))
    actual = _world_position(prim)
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


def _normalize_visual_size(stage, visual, source_bounds, visual_path):
    """Normalize the duplicated visual to the already-correct source size."""

    source_size = _size_tuple(source_bounds)
    visual_bounds = _world_bounds(stage, visual_path)
    visual_size = _size_tuple(visual_bounds)

    if all(
        abs(source_value - visual_value) <= SIZE_TOLERANCE_M
        for source_value, visual_value in zip(source_size, visual_size)
    ):
        return visual_bounds

    factors = []
    for axis_name, source_value, visual_value in zip(
        ("x", "y", "z"), source_size, visual_size
    ):
        if source_value <= 1.0e-9 or visual_value <= 1.0e-9:
            raise RuntimeError(
                f"Cannot normalize cargo guard {axis_name}: "
                f"source={source_value}, visual={visual_value}"
            )
        factor = source_value / visual_value
        if not math.isfinite(factor) or factor <= 0.0:
            raise RuntimeError(
                f"Invalid cargo guard scale factor on {axis_name}: {factor}"
            )
        factors.append(factor)

    xform = UsdGeom.Xformable(visual)
    xform.AddScaleOp(
        UsdGeom.XformOp.PrecisionDouble,
        "matchSourceSize",
    ).Set(Gf.Vec3d(*factors))

    visual_bounds = _world_bounds(stage, visual_path)
    normalized_size = _size_tuple(visual_bounds)
    for axis_name, source_value, normalized_value in zip(
        ("x", "y", "z"), source_size, normalized_size
    ):
        if abs(source_value - normalized_value) > SIZE_TOLERANCE_M:
            raise RuntimeError(
                f"Cargo guard normalized size mismatch on {axis_name}: "
                f"source={source_value:.6f}, clone={normalized_value:.6f}"
            )

    print(
        "[CARGO GUARD] duplicate visual auto-normalized: "
        f"scale=({factors[0]:.6f}, {factors[1]:.6f}, {factors[2]:.6f})"
    )
    return visual_bounds


def _disable_nested_physics(root_prim):
    for prim in Usd.PrimRange(root_prim):
        if prim == root_prim or prim.IsInstanceProxy():
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)


def _create_collision_box(stage, path, center, size, visible=False):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    if visible:
        UsdGeom.Gprim(cube.GetPrim()).CreateDisplayColorAttr().Set(
            [Gf.Vec3f(0.20, 0.20, 0.20)]
        )
    else:
        UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()


def _world_to_root_local(world_point, root_xyz, yaw_deg):
    dx = float(world_point[0]) - float(root_xyz[0])
    dy = float(world_point[1]) - float(root_xyz[1])
    dz = float(world_point[2]) - float(root_xyz[2])

    angle = math.radians(-float(yaw_deg))
    c = math.cos(angle)
    s = math.sin(angle)
    return (
        c * dx - s * dy,
        s * dx + c * dy,
        dz,
    )


def _floor_geometry(visual_bounds):
    minimum = visual_bounds.GetMin()
    size = visual_bounds.GetMax() - minimum
    sz = float(size[2])

    floor_thickness = max(0.02, 0.05 * sz)
    floor_bottom_z = float(minimum[2]) + 0.25 * sz
    floor_center_z = floor_bottom_z + 0.5 * floor_thickness
    floor_top_z = floor_bottom_z + floor_thickness
    return floor_thickness, floor_bottom_z, floor_center_z, floor_top_z


def _add_runtime_physics(
    stage,
    root_prim,
    visual_bounds,
    root_xyz,
    spawn_yaw,
    ground_bottom_world_z,
):
    """Add a 1 m cargo envelope with 8 cm longer visible/collision legs."""

    minimum = visual_bounds.GetMin()
    maximum = visual_bounds.GetMax()
    size = maximum - minimum

    sx = float(size[0])
    sy = float(size[1])
    sz = float(size[2])
    if min(sx, sy, sz) <= 1.0e-4:
        raise RuntimeError(
            f"Invalid cargo guard bounds: size=({sx:.6f}, {sy:.6f}, {sz:.6f})"
        )

    center_x = 0.5 * (float(minimum[0]) + float(maximum[0]))
    center_y = 0.5 * (float(minimum[1]) + float(maximum[1]))

    floor_thickness, floor_bottom_z, floor_center_z, floor_top_z = (
        _floor_geometry(visual_bounds)
    )

    leg_width_x = max(0.05, 0.10 * sx)
    leg_width_y = max(0.05, 0.10 * sy)
    leg_height = max(0.02, floor_bottom_z - float(ground_bottom_world_z))
    leg_center_z = float(ground_bottom_world_z) + 0.5 * leg_height
    leg_x = 0.5 * sx - 0.5 * leg_width_x
    leg_y = 0.5 * sy - 0.5 * leg_width_y

    collider_root = f"{root_prim.GetPath()}/PhysicsColliders"
    UsdGeom.Xform.Define(stage, collider_root)

    leg_centers_world = {
        "leg_front_left": (center_x + leg_x, center_y + leg_y, leg_center_z),
        "leg_front_right": (center_x + leg_x, center_y - leg_y, leg_center_z),
        "leg_rear_left": (center_x - leg_x, center_y + leg_y, leg_center_z),
        "leg_rear_right": (center_x - leg_x, center_y - leg_y, leg_center_z),
    }
    for name, world_center in leg_centers_world.items():
        _create_collision_box(
            stage,
            f"{collider_root}/{name}",
            _world_to_root_local(world_center, root_xyz, spawn_yaw),
            (leg_width_x, leg_width_y, leg_height),
            visible=True,
        )

    _create_collision_box(
        stage,
        f"{collider_root}/floor",
        _world_to_root_local(
            (center_x, center_y, floor_center_z), root_xyz, spawn_yaw
        ),
        (sx, sy, floor_thickness),
    )

    wall_thickness = max(0.01, 0.02 * min(sx, sy))
    wall_height = max(0.05, float(maximum[2]) - floor_top_z)
    wall_center_z = floor_top_z + 0.5 * wall_height

    wall_specs = {
        "wall_front": (
            (float(maximum[0]) - 0.5 * wall_thickness, center_y, wall_center_z),
            (wall_thickness, sy, wall_height),
        ),
        "wall_rear": (
            (float(minimum[0]) + 0.5 * wall_thickness, center_y, wall_center_z),
            (wall_thickness, sy, wall_height),
        ),
        "wall_left": (
            (center_x, float(maximum[1]) - 0.5 * wall_thickness, wall_center_z),
            (sx, wall_thickness, wall_height),
        ),
        "wall_right": (
            (center_x, float(minimum[1]) + 0.5 * wall_thickness, wall_center_z),
            (sx, wall_thickness, wall_height),
        ),
    }
    for name, (world_center, world_size) in wall_specs.items():
        _create_collision_box(
            stage,
            f"{collider_root}/{name}",
            _world_to_root_local(world_center, root_xyz, spawn_yaw),
            world_size,
        )

    print(
        "[CARGO GUARD] longer legs applied: "
        f"extension={LEG_EXTENSION_M:.3f} m, "
        f"ground_clearance={leg_height:.3f} m"
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
    """Create a stable wrapper root and duplicate the warehouse guard under it."""

    destination_path = str(destination_path)
    source = _find_source_prim(stage, str(source_name))
    source_path = source.GetPath().pathString
    source_bounds = _world_bounds(stage, source_path)
    source_root_xyz = _world_position(source)
    source_bottom_offset_z = float(source_bounds.GetMin()[2]) - source_root_xyz[2]

    existing = stage.GetPrimAtPath(destination_path)
    if existing.IsValid():
        stage.RemovePrim(destination_path)

    root = UsdGeom.Xform.Define(stage, destination_path).GetPrim()
    root_xform = UsdGeom.Xformable(root)
    root_xform.ClearXformOpOrder()
    root_xform.AddTranslateOp().Set(
        Gf.Vec3d(*[float(value) for value in spawn_xyz])
    )
    if abs(float(spawn_yaw)) > 1.0e-9:
        root_xform.AddRotateZOp().Set(float(spawn_yaw))

    visual_path = f"{destination_path}/Visual"
    duplicate_ok = omni.usd.duplicate_prim(
        stage,
        source_path,
        visual_path,
        duplicate_layers=True,
    )
    if duplicate_ok is False:
        raise RuntimeError(
            f"Failed to duplicate cargo guard {source_path} -> {visual_path}"
        )

    stage.Load(destination_path)
    visual = stage.GetPrimAtPath(visual_path)
    if not visual.IsValid():
        raise RuntimeError(f"Cargo guard visual clone is invalid: {visual_path}")

    # Clear the source world transform on the duplicated visual.  Only its body
    # is lifted by 8 cm; the parent root stays at the exact old cargo_pod pose.
    visual_xform = UsdGeom.Xformable(visual)
    visual_xform.ClearXformOpOrder()
    visual_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, LEG_EXTENSION_M))

    visual_bounds = _normalize_visual_size(
        stage,
        visual,
        source_bounds,
        visual_path,
    )

    _verify_exact_root_pose(stage, destination_path, spawn_xyz)
    _disable_nested_physics(visual)

    ground_bottom_world_z = float(spawn_xyz[2]) + source_bottom_offset_z
    collision_count = _add_runtime_physics(
        stage,
        root,
        visual_bounds,
        spawn_xyz,
        spawn_yaw,
        ground_bottom_world_z,
    )

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(root)
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)

    mass = UsdPhysics.MassAPI.Apply(root)
    mass.CreateMassAttr(float(mass_kg))

    size = _size_tuple(visual_bounds)
    print(
        f"[CARGO GUARD] cloned {source_path} -> {destination_path}; "
        f"pose=({float(spawn_xyz[0]):.3f}, {float(spawn_xyz[1]):.3f}, "
        f"{float(spawn_xyz[2]):.3f}), yaw={float(spawn_yaw):.1f} deg"
    )
    print(
        "[CARGO GUARD] visual body raised for AMR clearance: "
        f"{LEG_EXTENSION_M:.3f} m; "
        f"visual_size=({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}) m"
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
    """Place the four parcels as one 2x2 layer on the raised cargo floor."""

    del cargo_center_xy
    visual_path = f"{str(guard_path)}/Visual"
    bounds = _world_bounds(stage, visual_path)
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()

    center_x = 0.5 * (float(minimum[0]) + float(maximum[0]))
    center_y = 0.5 * (float(minimum[1]) + float(maximum[1]))
    _, _, _, floor_top_world_z = _floor_geometry(bounds)

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
                f"center=({parcel_x:.3f}, {parcel_y:.3f}), size={max_size}"
            )

        item = dict(config)
        item["spawn_xyz"] = (parcel_x, parcel_y, common_z)
        resolved.append(item)

    print(
        "[CARGO GUARD] four-parcel 2x2 layer resolved: "
        f"center=({center_x:.3f}, {center_y:.3f}), "
        f"z={common_z:.3f}, count={len(resolved)}"
    )
    return resolved
