"""Rigid-body physics and parcel assets for the cargo pod."""

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


BLUE = Gf.Vec3f(0.05, 0.25, 0.95)

# The STEP cargo pod is 1.0 x 1.0 x 1.0 m.
# Local Z = -0.5 is the bottom of the four legs.
# Local Z = -0.25 is the underside of the cargo floor.
# This leaves about 0.25 m of clearance for the IW Hub.


def _create_collision_box(stage, path, center, size):
    """Create one visible box collider under the cargo rigid body."""

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([BLUE])
    cube.CreateDisplayOpacityAttr([0.45])

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    # Keep the simplified cargo colliders visible while docking/lift is tested.
    UsdGeom.Imageable(cube.GetPrim()).MakeVisible()


def _apply_blue_material(stage, prim_path):
    """Override the referenced STEP visual with a blue material."""

    cargo_prim = stage.GetPrimAtPath(prim_path)
    if not cargo_prim.IsValid():
        return

    material_path = f"{prim_path}/Looks/BlueCargoPod"
    shader_path = f"{material_path}/Shader"

    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, shader_path)
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(BLUE)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(),
        "surface",
    )

    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not path.startswith(f"{prim_path}/"):
            continue
        if path.startswith(f"{prim_path}/Looks/"):
            continue
        if prim.IsA(UsdGeom.Gprim):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def add_cargo_pod_physics(stage, prim_path, mass_kg=20.0):
    """Turn the referenced STEP mesh into one compound rigid body."""

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
    # Its underside is local Z=-0.25 m, where the IW Hub lift contacts it.
    _create_collision_box(
        stage,
        f"{collision_root_path}/floor",
        (0.0, 0.0, -0.225),
        (1.0, 1.0, 0.05),
    )

    # Thin wall colliders keep parcel objects inside the pod.
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

    _apply_blue_material(stage, prim_path)

    print(
        f"[CARGO] physics enabled for {prim_path} "
        f"(mass={float(mass_kg):.1f} kg, color=blue, compound collision)"
    )


def _disable_referenced_physics(asset_prim):
    """Disable physics authored inside a referenced visual asset.

    The parcel root below owns the one authoritative rigid body and collider.
    This avoids nested rigid bodies or duplicate collision shapes if the prop
    asset later gains its own physics schemas.
    """

    for prim in Usd.PrimRange(asset_prim):
        if prim.IsInstanceProxy():
            continue

        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            api = UsdPhysics.RigidBodyAPI(prim)
            api.CreateRigidBodyEnabledAttr(False)

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            api = UsdPhysics.CollisionAPI(prim)
            api.CreateCollisionEnabledAttr(False)


def add_parcel_asset(
    stage,
    prim_path,
    asset_url,
    center,
    max_size,
    mass_kg=15.0,
):
    """Spawn a referenced cardboard-box asset as a dynamic 15 kg parcel.

    The NVIDIA visual asset keeps its original materials and shape. Its visual
    is uniformly scaled to fit inside ``max_size`` without distortion. A
    separate invisible box collider is created from the resulting dimensions,
    while the parcel root owns gravity-driven rigid-body physics and mass.
    """

    root = UsdGeom.Xform.Define(stage, prim_path)
    root_xform = UsdGeom.Xformable(root.GetPrim())
    root_xform.AddTranslateOp().Set(Gf.Vec3d(*center))

    scale_path = f"{prim_path}/VisualScale"
    offset_path = f"{scale_path}/VisualOffset"
    asset_path = f"{offset_path}/Asset"

    scale_prim = UsdGeom.Xform.Define(stage, scale_path)
    offset_prim = UsdGeom.Xform.Define(stage, offset_path)
    asset_prim = stage.DefinePrim(asset_path)
    asset_prim.GetReferences().AddReference(str(asset_url))

    # Measure the referenced asset at runtime, so the code remains valid even
    # if the source USD's native dimensions or units differ from our scene.
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_],
        useExtentsHint=True,
    )
    local_range = bbox_cache.ComputeLocalBound(asset_prim).GetRange()

    if local_range.IsEmpty():
        raise RuntimeError(
            f"Parcel asset has no measurable bounds: {asset_url}"
        )

    source_min = local_range.GetMin()
    source_max = local_range.GetMax()
    source_center = (source_min + source_max) * 0.5
    source_size = source_max - source_min

    source_dims = tuple(float(source_size[i]) for i in range(3))
    target_dims = tuple(float(v) for v in max_size)

    if any(v <= 1.0e-6 for v in source_dims):
        raise RuntimeError(
            f"Invalid parcel asset bounds {source_dims}: {asset_url}"
        )

    uniform_scale = min(
        target_dims[i] / source_dims[i]
        for i in range(3)
    )
    final_dims = tuple(
        source_dims[i] * uniform_scale
        for i in range(3)
    )

    UsdGeom.Xformable(scale_prim.GetPrim()).AddScaleOp().Set(
        Gf.Vec3f(uniform_scale, uniform_scale, uniform_scale)
    )
    UsdGeom.Xformable(offset_prim.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(
            -float(source_center[0]),
            -float(source_center[1]),
            -float(source_center[2]),
        )
    )

    _disable_referenced_physics(asset_prim)

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)

    mass = UsdPhysics.MassAPI.Apply(root.GetPrim())
    mass.CreateMassAttr(float(mass_kg))

    # Stable simple collider: the NVIDIA USD is the visual, while PhysX uses
    # one invisible box matching the final scaled bounding dimensions.
    collider = UsdGeom.Cube.Define(stage, f"{prim_path}/PhysicsCollider")
    collider.CreateSizeAttr(1.0)
    collider_xform = UsdGeom.Xformable(collider.GetPrim())
    collider_xform.AddScaleOp().Set(Gf.Vec3f(*final_dims))
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    UsdGeom.Imageable(collider.GetPrim()).MakeInvisible()


def add_parcel_asset_scaled(
    stage,
    prim_path,
    asset_url,
    center,
    scale_xyz,
    box_id,
    mass_kg=15.0,
):
    """Spawn a referenced cardboard-box asset at an explicit, non-uniform
    scale (e.g. (0.75, 0.75, 0.5) to flatten it), tagged with a box_id
    custom int attribute.

    Unlike add_parcel_asset (which uniformly fits the asset's aspect ratio
    inside a target max_size box), this applies scale_xyz directly since the
    caller already knows the exact desired shape. ``center`` is the
    geometric center of the final scaled box, matching add_parcel_asset's
    convention.
    """

    root = UsdGeom.Xform.Define(stage, prim_path)
    root_xform = UsdGeom.Xformable(root.GetPrim())
    root_xform.AddTranslateOp().Set(Gf.Vec3d(*center))

    scale_path = f"{prim_path}/VisualScale"
    offset_path = f"{scale_path}/VisualOffset"
    asset_path = f"{offset_path}/Asset"

    scale_prim = UsdGeom.Xform.Define(stage, scale_path)
    offset_prim = UsdGeom.Xform.Define(stage, offset_path)
    asset_prim = stage.DefinePrim(asset_path)
    asset_prim.GetReferences().AddReference(str(asset_url))

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_],
        useExtentsHint=True,
    )
    local_range = bbox_cache.ComputeLocalBound(asset_prim).GetRange()

    if local_range.IsEmpty():
        raise RuntimeError(
            f"Parcel asset has no measurable bounds: {asset_url}"
        )

    source_min = local_range.GetMin()
    source_max = local_range.GetMax()
    source_center = (source_min + source_max) * 0.5
    source_size = source_max - source_min

    source_dims = tuple(float(source_size[i]) for i in range(3))
    if any(v <= 1.0e-6 for v in source_dims):
        raise RuntimeError(
            f"Invalid parcel asset bounds {source_dims}: {asset_url}"
        )

    scale_dims = tuple(float(v) for v in scale_xyz)
    final_dims = tuple(source_dims[i] * scale_dims[i] for i in range(3))

    UsdGeom.Xformable(scale_prim.GetPrim()).AddScaleOp().Set(
        Gf.Vec3f(*scale_dims)
    )
    UsdGeom.Xformable(offset_prim.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(
            -float(source_center[0]),
            -float(source_center[1]),
            -float(source_center[2]),
        )
    )

    _disable_referenced_physics(asset_prim)

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)

    mass = UsdPhysics.MassAPI.Apply(root.GetPrim())
    mass.CreateMassAttr(float(mass_kg))

    collider = UsdGeom.Cube.Define(stage, f"{prim_path}/PhysicsCollider")
    collider.CreateSizeAttr(1.0)
    collider_xform = UsdGeom.Xformable(collider.GetPrim())
    collider_xform.AddScaleOp().Set(Gf.Vec3f(*final_dims))
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    UsdGeom.Imageable(collider.GetPrim()).MakeInvisible()

    root.GetPrim().CreateAttribute("box_id", Sdf.ValueTypeNames.Int).Set(
        int(box_id)
    )

    print(
        f"[PARCEL] NVIDIA CardBox spawned {prim_path}: "
        f"source={source_dims}, fit={final_dims} m, "
        f"mass={float(mass_kg):.1f} kg, dynamic=True"
    )
