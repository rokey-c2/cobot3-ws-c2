"""RTX LiDAR publisher used by the IW Hub Nav2 integration."""

import carb
import omni.kit.commands
import omni.replicator.core as rep
import omni.usd
from pxr import Gf


class IwHubLidarRos2Publisher:
    """Create one near-field 2D RTX LiDAR and keep its ROS writer alive."""

    MIN_RANGE_M = 0.10

    def __init__(self, parent_prim_path, namespace="amr_a"):
        self.parent_prim_path = parent_prim_path.rstrip("/")
        self.namespace = namespace.strip("/")
        self.lidar_prim_path = f"{self.parent_prim_path}/nav_lidar"
        self.sensor_prim = self._get_or_create_sensor()
        self._set_near_range()
        self.render_product = rep.create.render_product(
            self.sensor_prim.GetPath(),
            [1, 1],
            name=f"{self.namespace}_nav_lidar",
        )
        self.writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
        self.writer.initialize(
            topicName=f"/{self.namespace}/scan",
            frameId="base_link",
        )
        self.writer.attach([self.render_product])
        print(
            f"[LIDAR] {self.lidar_prim_path} -> "
            f"/{self.namespace}/scan (near range {self.MIN_RANGE_M:.2f} m)"
        )

    def _get_or_create_sensor(self):
        stage = omni.usd.get_context().get_stage()
        existing = stage.GetPrimAtPath(self.lidar_prim_path)
        if existing.IsValid():
            return existing

        success, sensor_prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=self.lidar_prim_path,
            parent=None,
            config="Example_Rotary_2D",
            translation=(0.0, 0.0, 0.75),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        )
        if not success or sensor_prim is None:
            raise RuntimeError(
                f"RTX LiDAR creation failed: {self.lidar_prim_path}"
            )
        return sensor_prim

    def _set_near_range(self):
        """Override Example_Rotary_2D's 1 m blind zone for AMR safety."""

        near_range = self.sensor_prim.GetAttribute(
            "omni:sensor:Core:nearRangeM"
        )
        if not near_range.IsValid():
            carb.log_warn(
                "[LIDAR] nearRangeM attribute unavailable; using sensor profile default"
            )
            return
        near_range.Set(self.MIN_RANGE_M)

