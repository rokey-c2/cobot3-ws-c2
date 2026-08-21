"""ForkliftB에 2D RTX LiDAR를 장착하고 ROS 2 LaserScan을 발행한다."""

import omni.kit.commands
import omni.replicator.core as rep
import omni.usd
from pxr import Gf


LIDAR_MOUNT_X = 0.0
LIDAR_MOUNT_Y = 0.0
LIDAR_MOUNT_Z = 2.0


class ForkliftLidarRos2Publisher:
    """Isaac RTX LiDAR와 ROS 2 LaserScan writer의 수명을 관리한다."""

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
        self.writer = rep.writers.get(
            "RtxLidarROS2PublishLaserScan"
        )
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
            translation=(
                LIDAR_MOUNT_X,
                LIDAR_MOUNT_Y,
                LIDAR_MOUNT_Z,
            ),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        )

        if not success or sensor_prim is None:
            raise RuntimeError(
                f"RTX LiDAR 생성 실패: {self.lidar_prim_path}"
            )

        return sensor_prim
