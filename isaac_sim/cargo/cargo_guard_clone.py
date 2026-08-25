"""Cargo guard clone for the IW Hub mission.

The visual shape is cloned from the already-visible warehouse
``/World/cargo_box_gaurd_size_201``.  Its mission physics height is matched to
the cargo_pod geometry that was proven to lift correctly at commit
8f084b640c7d50dfd98d51d7df0d968a373874bd:

- cargo root world Z: 0.50 m
- leg bottom: local Z=-0.50 m -> world Z=0.00 m
- leg height: 0.25 m
- floor underside: local Z=-0.25 m -> world Z=0.25 m
- floor top: local Z=-0.20 m -> world Z=0.30 m

The source guard visual is raised so its lowest point starts at the proven
floor-underside height.  Four visible/collision legs and all compound
colliders use the exact baseline cargo_pod dimensions.
"""

import math

import omni.usd

from pxr import Gf, Usd, UsdGeom, UsdPhysics


SOURCE_GUARD_NAME = "cargo_box_gaurd_size_201"
POSE_TOLERANCE_M = 1.0e-6
SIZE_TOLERANCE_M = 1.0e-3

# Exact cargo_pod geometry from commit 8f084b... .
BASELINE_LEG_SIZE = (0.10, 0.10, 0.25)
BASELINE_LEG_Z = -0.375
BASELINE_LEG_XY = 0.45
BASELINE_FLOOR_SIZE = (1.0, 1.0, 0.05)
BASELINE_FLOOR_CENTER_Z = -0.225
BASELINE_FLOOR_BOTTOM_Z = -0.25
BASELINE_FLOOR_TOP_Z = -0.20
BASELINE_WALL_HEIGHT = 0.70
BASELINE_WALL_Z = 0.15
BASELINE_WALL_THICKNESS = 0.02
BASELINE_WALL_XY = 0.49


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
    """Normalize duplicate_prim unit expansion back to the source dimensions."""

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

    UsdGeom.Xformable(visual).AddScaleOp(
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


def _align_visual_to_baseline_floor(stage, visual, visual_path, root_xyz):
    """Put the cloned guard body at the proven cargo_pod floor height."""

    bounds = _world_bounds(stage, visual_path)
    current_min_z = float(bounds.GetMin()[2])
    target_min_z = float(root_xyz[2]) + BASELINE_FLOOR_BOTTOM_Z
    delta_z = target_min_z - current_min_z

    xform = UsdGeom.Xformable(visual)
    xform.AddTranslateOp(
        UsdGeom.XformOp.PrecisionDouble,
        "matchBaselineFloorHeight",
    ).Set(Gf.Vec3d(0.0, 0.0, delta_z))

    aligned = _world_bounds(stage, visual_path)
    actual_min_z = float(aligned.GetMin()[2])
    if abs(actual_min_z - target_min_z) > SIZE_TOLERANCE_M:
        raise RuntimeError(
            "Cargo guard visual floor-height alignment failed: "
            f"actual_min_z={actual_min_z:.6f}, target={target_min_z:.6f}"
        )

    print(
        "[CARGO GUARD] visual aligned to baseline cargo_pod height: "
        f"bottom={actual_min_z:.3f} m, floor_underside={target_min_z:.3f} m"
    )
    return aligned


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


def _add_baseline_cargo_pod_physics(stage, root_prim):
    """Recreate the exact compound collision that lifted successfully."""

    collider_root = f"{root_prim.GetPath()}/PhysicsColliders"
    UsdGeom.Xform.Define(stage, collider_root)

    leg_centers = {
        "leg_front_left": (
            BASELINE_LEG_XY,
            BASELINE_LEG_XY,
            BASELINE_LEG_Z,
        ),
        "leg_front_right": (
            BASELINE_LEG_XY,
            -BASELINE_LEG_XY,
            BASELINE_LEG_Z,
        ),
        "leg_rear_left": (
            -BASELINE_LEG_XY,
            BASELINE_LEG_XY,
            BASELINE_LEG_Z,
        ),
        "leg_rear_right": (
            -BASELINE_LEG_XY,
            -BASELINE_LEG_XY,
            BASELINE_LEG_Z,
        ),
    }
    for name, center in leg_centers.items():
        _create_collision_box(
            stage,
            f"{collider_root}/{name}",
            center,
            BASELINE_LEG_SIZE,
            visible=True,
        )

    _create_collision_box(
        stage,
        f"{collider_root}/floor",
        (0.0, 0.0, BASELINE_FLOOR_CENTER_Z),
        BASELINE_FLOOR_SIZE,
    )

    wall_specs = {
        "wall_front": (
            (BASELINE_WALL_XY, 0.0, BASELINE_WALL_Z),
            (
                BASELINE_WALL_THICKNESS,
                1.0,
                BASELINE_WALL_HEIGHT,
            ),
        ),
        "wall_rear": (
            (-BASELINE_WALL_XY, 0.0, BASELINE_WALL_Z),
            (
                BASELINE_WALL_THICKNESS,
                1.0,
                BASELINE_WALL_HEIGHT,
            ),
        ),
        "wall_left": (
            (0.0, BASELINE_WALL_XY, BASELINE_WALL_Z),
            (
                1.0,
                BASELINE_WALL_THICKNESS,
                BASELINE_WALL_HEIGHT,
            ),
        ),
        "wall_right": (
            (0.0, -BASELINE_WALL_XY, BASELINE_WALL_Z),
            (
                1.0,
                BASELINE_WALL_THICKNESS,
                BASELINE_WALL_HEIGHT,
            ),
        ),
    }
    for name, (center, size) in wall_specs.items():
        _create_collision_box(
            stage,
            f"{collider_root}/{name}",
            center,
            size,
        )

    print(
        "[CARGO GUARD] baseline cargo_pod clearance restored: "
        "leg_height=0.250 m, floor_underside=0.250 m world Z, "
        "floor_top=0.300 m world Z"
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
    """Clone the guard visual and use the proven cargo_pod lift geometry."""

    destination_path = str(destination_path)
    source = _find_source_prim(stage, str(source_name))
    source_path = source.GetPath().pathString
    source_bounds = _world_bounds(stage, source_path)

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

    visual_xform = UsdGeom.Xformable(visual)
    visual_xform.ClearXformOpOrder()
    visual_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

    _normalize_visual_size(stage, visual, source_bounds, visual_path)
    visual_bounds = _align_visual_to_baseline_floor(
        stage,
        visual,
        visual_path,
        spawn_xyz,
    )

    _verify_exact_root_pose(stage, destination_path, spawn_xyz)
    _disable_nested_physics(visual)

    collision_count = _add_baseline_cargo_pod_physics(stage, root)

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
        "[CARGO GUARD] visual_size="
        f"({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}) m; "
        f"mass={float(mass_kg):.1f} kg; "
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
    """Place four parcels in one 2x2 layer on the proven 0.30 m floor top."""

    del stage, guard_path

    center_x = float(cargo_center_xy[0])
    center_y = float(cargo_center_xy[1])
    floor_top_world_z = 0.30

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

        # Baseline cargo_pod is exactly 1.0 x 1.0 m in XY.
        inside_x = (
            parcel_x - half_x >= center_x - 0.5 + wall_clearance_m
            and parcel_x + half_x <= center_x + 0.5 - wall_clearance_m
        )
        inside_y = (
            parcel_y - half_y >= center_y - 0.5 + wall_clearance_m
            and parcel_y + half_y <= center_y + 0.5 - wall_clearance_m
        )
        if not (inside_x and inside_y):
            raise RuntimeError(
                f"Parcel {config['name']} does not fit on baseline cargo floor: "
                f"center=({parcel_x:.3f}, {parcel_y:.3f}), size={max_size}"
            )

        item = dict(config)
        item["spawn_xyz"] = (parcel_x, parcel_y, common_z)
        resolved.append(item)

    print(
        "[CARGO GUARD] four-parcel baseline layer resolved: "
        f"center=({center_x:.3f}, {center_y:.3f}), "
        f"floor_top={floor_top_world_z:.3f}, "
        f"parcel_z={common_z:.3f}, count={len(resolved)}"
    )
    return resolved
