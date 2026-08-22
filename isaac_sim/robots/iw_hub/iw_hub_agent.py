"""Spawn the default Isaac Sim Idealworks IW Hub Sensor asset."""

import carb
import omni.graph.core as og
import omni.usd
from pxr import Gf, UsdGeom

from robots.base_robot import BaseRobotAgent


_SENSORS_REL = "iw_hub_sensors"


class IwHubAgent(BaseRobotAgent):
    """Spawn IW Hub without changing its built-in ROS graph or sensors."""

    def __init__(self, cfg, world, usd_path):
        super().__init__(cfg, world)
        self.usd_path = str(usd_path)
        self.spawn_xyz = tuple(cfg["spawn_xyz"])
        self.spawn_yaw = float(cfg.get("spawn_yaw", 0.0))
        self.prim_path = f"/World/Robots/{self.name}"
        self.sensor_prim_path = f"{self.prim_path}/{_SENSORS_REL}"
        self.lift_prim_path = f"{self.sensor_prim_path}/lift"

    def setup(self):
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, "/World/Robots")

        prim = stage.DefinePrim(self.prim_path, "Xform")
        prim.GetReferences().AddReference(self.usd_path)

        transform = UsdGeom.Xformable(prim)
        transform.ClearXformOpOrder()
        transform.AddTranslateOp().Set(Gf.Vec3d(*self.spawn_xyz))
        transform.AddRotateXYZOp().Set(
            Gf.Vec3f(0.0, 0.0, self.spawn_yaw)
        )

        stage.Load(self.prim_path)

        carb.log_info(
            f"[IW HUB] spawned {self.name} at {self.spawn_xyz}; "
            "using the asset's default ROS graph and sensors"
        )

    def post_reset(self):
        graph_path = f"{self.sensor_prim_path}/ActionGraph"

        if og.get_graph_by_path(graph_path) is None:
            raise RuntimeError(
                "IW Hub default ActionGraph is missing: "
                f"{graph_path}"
            )

        carb.log_info(
            f"[IW HUB] default embedded graph active: {graph_path}"
        )

    def on_physics_step(self, dt):
        del dt
