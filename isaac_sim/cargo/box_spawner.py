"""Spawns simple physics-enabled test cubes tagged with a box_id."""

from pxr import Gf, Sdf, UsdGeom, UsdPhysics


def spawn_box_with_id(stage, prim_path, position, box_id, size=0.15, mass_kg=0.5):
    """Spawn one dynamic physics cube carrying a `box_id` custom attribute.

    This file is brought from hwi_conveyor_test as a reusable starting point
    for the wheel-sorter-only demo on hwi_new_sorter.
    """

    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.CreateSizeAttr(1.0)

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3f(size, size, size))

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)

    mass = UsdPhysics.MassAPI.Apply(cube.GetPrim())
    mass.CreateMassAttr(float(mass_kg))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    box_id_attr = cube.GetPrim().CreateAttribute(
        "box_id", Sdf.ValueTypeNames.Int, custom=True
    )
    box_id_attr.Set(int(box_id))

    print(
        f"[BOX] spawned {prim_path} at {tuple(position)} box_id={int(box_id)}"
    )

    return cube.GetPrim()
