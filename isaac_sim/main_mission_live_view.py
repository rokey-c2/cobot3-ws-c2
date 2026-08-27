"""Run the existing warehouse mission with a Control Tower top-view camera.

This wrapper intentionally leaves ``main_mission.py`` untouched.  It swaps only
its ROS bridge class so the already-open Isaac stage gets one fixed overhead
camera that publishes ``/top_view/rgb`` for the web Control Tower.
"""

import numpy as np

import main_mission as mission
import omni.usd
from isaacsim.sensors.camera import Camera
from pxr import Gf, UsdGeom
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOP_VIEW_CAMERA_PATH = "/World/ControlTowerTopViewCamera"
TOP_VIEW_IMAGE_TOPIC = "/top_view/rgb"
TOP_VIEW_RESOLUTION = (1280, 720)
TOP_VIEW_PUBLISH_PERIOD_SEC = 0.10

# The Final_Real_Map working area is centred slightly left/below world origin.
# A 32 m high camera with this lens keeps the complete inbound -> sorter ->
# outbound line visible while leaving a small border around the warehouse.
TOP_VIEW_POSITION = Gf.Vec3d(-3.5, -2.0, 32.0)
TOP_VIEW_TARGET = Gf.Vec3d(-3.5, -2.0, 0.0)
TOP_VIEW_UP = Gf.Vec3d(0.0, 1.0, 0.0)
TOP_VIEW_FOCAL_LENGTH_MM = 28.0
TOP_VIEW_HORIZONTAL_APERTURE_MM = 36.0
TOP_VIEW_VERTICAL_APERTURE_MM = 20.25


def _create_top_view_camera():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac USD stage is not available")

    usd_camera = UsdGeom.Camera.Define(stage, TOP_VIEW_CAMERA_PATH)
    xform = UsdGeom.Xformable(usd_camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(TOP_VIEW_POSITION)

    look_at = Gf.Matrix4d().SetLookAt(
        TOP_VIEW_POSITION,
        TOP_VIEW_TARGET,
        TOP_VIEW_UP,
    )
    orientation = look_at.GetInverse().ExtractRotation().GetQuat()
    xform.AddOrientOp().Set(Gf.Quatf(orientation))

    usd_camera.GetFocalLengthAttr().Set(TOP_VIEW_FOCAL_LENGTH_MM)
    usd_camera.GetHorizontalApertureAttr().Set(TOP_VIEW_HORIZONTAL_APERTURE_MM)
    usd_camera.GetVerticalApertureAttr().Set(TOP_VIEW_VERTICAL_APERTURE_MM)

    camera = Camera(
        prim_path=TOP_VIEW_CAMERA_PATH,
        resolution=TOP_VIEW_RESOLUTION,
    )
    camera.initialize()
    return camera


class LiveViewAmrMissionBridge(mission.AmrMissionBridge):
    """Original mission bridge plus a lightweight overhead RGB publisher."""

    def __init__(self, agent):
        super().__init__(agent)
        self._top_view_camera = None
        self._top_view_pub = None

        try:
            self._top_view_camera = _create_top_view_camera()
            self._top_view_pub = self.create_publisher(
                Image,
                TOP_VIEW_IMAGE_TOPIC,
                qos_profile_sensor_data,
            )
            self.create_timer(
                TOP_VIEW_PUBLISH_PERIOD_SEC,
                self._publish_top_view,
            )
            self.get_logger().info(
                "Control Tower top view ready: "
                f"{TOP_VIEW_IMAGE_TOPIC} {TOP_VIEW_RESOLUTION[0]}x{TOP_VIEW_RESOLUTION[1]} "
                f"@ ~{1.0 / TOP_VIEW_PUBLISH_PERIOD_SEC:.0f} Hz"
            )
        except Exception as error:
            # The warehouse mission must stay usable even if the optional web
            # camera cannot be created on a particular Isaac installation.
            self.get_logger().error(f"top-view camera disabled: {error}")

    def _publish_top_view(self):
        if self._top_view_camera is None or self._top_view_pub is None:
            return

        rgba = self._top_view_camera.get_rgba()
        if rgba is None or rgba.size == 0:
            return

        rgb = np.ascontiguousarray(rgba[:, :, :3])
        if rgb.mean() < 1.0:
            return

        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "control_tower_top_view"
        message.height, message.width = rgb.shape[:2]
        message.encoding = "rgb8"
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = rgb.tobytes()
        self._top_view_pub.publish(message)


def main():
    mission.AmrMissionBridge = LiveViewAmrMissionBridge
    mission.main()


if __name__ == "__main__":
    main()
