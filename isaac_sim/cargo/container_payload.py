"""Procedural box container that follows the IW Hub lift until placement."""

import math

import carb
import omni.usd
from pxr import Gf, UsdGeom

from cargo.container_policy import should_release_payload


def _define_cube(stage, path, translate, scale, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    transform = UsdGeom.Xformable(cube.GetPrim())
    transform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    transform.AddScaleOp().Set(Gf.Vec3f(*scale))


class CargoContainerPayload:
    """Keep a visual cargo container attached to a moving lift prim.

    The fixture intentionally follows the articulation transform instead of
    relying on contact friction.  It is released only after the AMR reaches
    the configured goal and the lift returns to its lowered position.
    """

    def __init__(
        self,
        lift_prim_path,
        goal_xy,
        prim_path="/World/Cargo/container_01",
        lift_offset=(0.0, 0.0, 0.85),
        lifted_threshold=0.15,
        goal_tolerance=0.30,
        lowered_tolerance=0.04,
    ):
        self.stage = omni.usd.get_context().get_stage()
        self.lift_prim_path = lift_prim_path
        self.goal_xy = tuple(float(value) for value in goal_xy)
        self.prim_path = prim_path
        self.attached_prim_path = (
            f"{lift_prim_path}/attached_{prim_path.rsplit('/', 1)[-1]}"
        )
        self.lift_offset = Gf.Vec3d(*lift_offset)
        self.lifted_threshold = float(lifted_threshold)
        self.goal_tolerance = float(goal_tolerance)
        self.lowered_tolerance = float(lowered_tolerance)

        self._lift_prim = self.stage.GetPrimAtPath(lift_prim_path)
        if not self._lift_prim.IsValid():
            raise RuntimeError(
                f"IW Hub lift prim does not exist: {lift_prim_path}"
            )

        UsdGeom.Xform.Define(self.stage, "/World/Cargo")
        root = UsdGeom.Xform.Define(
            self.stage, self.attached_prim_path
        )
        root.ClearXformOpOrder()
        root.AddTranslateOp().Set(self.lift_offset)
        self._attached_root = root.GetPrim()
        self._create_visuals(self.attached_prim_path)

        self.attached = True
        self.lifted_once = False
        self._baseline_lift_z = None
        self.update()
        carb.log_info(
            f"[CARGO] attached {self.attached_prim_path} to "
            f"{lift_prim_path}"
        )

    def _create_visuals(self, root_path):
        # The 1.00 x 0.70 m carrier stays inside the loaded Nav2 footprint.
        base_color = (0.15, 0.35, 0.70)
        wall_color = (0.20, 0.48, 0.88)
        _define_cube(
            self.stage,
            f"{root_path}/base",
            (0.0, 0.0, 0.04),
            (1.00, 0.70, 0.08),
            base_color,
        )
        _define_cube(
            self.stage,
            f"{root_path}/wall_front",
            (0.47, 0.0, 0.23),
            (0.06, 0.70, 0.38),
            wall_color,
        )
        _define_cube(
            self.stage,
            f"{root_path}/wall_rear",
            (-0.47, 0.0, 0.23),
            (0.06, 0.70, 0.38),
            wall_color,
        )
        _define_cube(
            self.stage,
            f"{root_path}/wall_left",
            (0.0, 0.32, 0.23),
            (0.88, 0.06, 0.38),
            wall_color,
        )
        _define_cube(
            self.stage,
            f"{root_path}/wall_right",
            (0.0, -0.32, 0.23),
            (0.88, 0.06, 0.38),
            wall_color,
        )

        boxes = (
            ("box_01", (-0.24, -0.16, 0.19), (0.34, 0.25, 0.22), (0.76, 0.48, 0.18)),
            ("box_02", (0.17, -0.15, 0.17), (0.36, 0.27, 0.18), (0.88, 0.64, 0.25)),
            ("box_03", (-0.05, 0.16, 0.18), (0.48, 0.24, 0.20), (0.67, 0.38, 0.15)),
        )
        for name, translate, scale, color in boxes:
            _define_cube(
                self.stage,
                f"{root_path}/{name}",
                translate,
                scale,
                color,
            )

    def _lift_world_matrix(self):
        cache = UsdGeom.XformCache()
        return cache.GetLocalToWorldTransform(self._lift_prim)

    def _place_payload(self):
        cache = UsdGeom.XformCache()
        world_matrix = cache.GetLocalToWorldTransform(
            self._attached_root
        )
        UsdGeom.Imageable(self._attached_root).MakeInvisible()

        placed_root = UsdGeom.Xform.Define(
            self.stage, self.prim_path
        )
        placed_root.ClearXformOpOrder()
        placed_root.AddTransformOp().Set(world_matrix)
        self._create_visuals(self.prim_path)

    def update(self):
        """Follow the lift and release the payload after a completed place."""

        if not self.attached:
            return

        lift_world_matrix = self._lift_world_matrix()
        lift_position = lift_world_matrix.ExtractTranslation()
        if self._baseline_lift_z is None:
            self._baseline_lift_z = float(lift_position[2])

        lift_delta = max(
            0.0, float(lift_position[2]) - self._baseline_lift_z
        )
        if lift_delta >= self.lifted_threshold:
            self.lifted_once = True

        distance_to_goal = math.hypot(
            float(lift_position[0]) - self.goal_xy[0],
            float(lift_position[1]) - self.goal_xy[1],
        )
        if should_release_payload(
            distance_to_goal=distance_to_goal,
            lifted_once=self.lifted_once,
            lift_delta=lift_delta,
            goal_tolerance=self.goal_tolerance,
            lowered_tolerance=self.lowered_tolerance,
        ):
            self._place_payload()
            self.attached = False
            carb.log_info(
                f"[CARGO] placed {self.prim_path} at "
                f"({lift_position[0]:.2f}, {lift_position[1]:.2f})"
            )
