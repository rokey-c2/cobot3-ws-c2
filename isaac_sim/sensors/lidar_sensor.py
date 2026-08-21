"""RTX LiDAR publisher used by the IW Hub Nav2 integration."""

import omni.kit.commands
import omni.replicator.core as rep
import omni.usd
from pxr import Gf


class IwHubLidarRos2Publisher:
    """Create a 2D RTX LiDAR and keep its ROS writer alive."""

    def __init__(self, parent_prim_path, namespace="amr_a"):
        self.parent_prim_path = parent_prim_path.rstrip("/")
        self.namespace = namespace.strip("/")
        self.lidar_prim_path = f"{self.parent_prim_path}/nav_lidar"
        self.sensor_prim = self._get_or_create_sensor()
        self.render_product = rep.create.render_product(
            self.sensor_prim.GetPath(),
            [1, 1],
            name=f"{self.namespace}_nav_lidar",
        )
        self.writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
        self.writer.initialize(
            topicName=f"/{self.namespace}/scan",
            frameId=f"{self.namespace}/lidar_link",
        )
        self.writer.attach([self.render_product])
        print(
            f"[LIDAR] {self.lidar_prim_path} -> "
            f"/{self.namespace}/scan"
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

